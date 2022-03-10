/**
 * Reworked program for the Teensy-based petal controller
 * 
 * Uses the following hardware:
 * Teensy 4.1 microncontroller
 * BNO055 on I2C
 * H3LIS331 on I2C
 * BMP280 on I2C
 * Teensy internal SD card reader
 * Teensy internal LED
 * DIP switch to ground on pins 2-9
 * Pushbutton to ground on pin 10
 * FET-driven solenoid output on pin 11
 * FET-driven buzzer on pin 12
 * 
 * Flight state machine looks like this:
 * 
 * READY --> BOOST --> COAST <--> BRAKE 
 *                         |       |
 *                         \/     \/
 *                          DESCENT --> LAND
 */

#include <BMP280_DEV.h>
#include <Device.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_BNO055.h>

#include <SD.h>

// this is ugly but I don't feel like setting up a library rn
#include "VectorMath.cpp"

// mission
#define ALT_TARGET 1300 // m

// rocket specs
#define LIFTOFF_MASS 9.38015 // kg
#define LIFTOFF_THRUST 900.0 // N
#define PROP_MASS 1.140 // kg
// #define BURN_TIME 2700 // ms
#define REF_AREA 0.0082045 // m^2
#define EST_CD 0.61

// environment
#define RHO 1.225
#define G 9.80

// altimeter state threshholds
#define H3L_THRESH 120 // when to switch to high-G, m/s^2
#define LIFTOFF_TIME 200 // time to verify liftoff, ms
#define LIFTOFF_THRESH 30 // minimum liftoff accel, m/s^2
#define BURNOUT_TIME 200 // time to verify burnout, ms
#define APOGEE_THRESH 0 // minimum descent distance for apogee, m
#define APOGEE_TIME 2000 // time to verify apogee, ms
#define LAND_THRESH 10 // maximum variation in altitude for landing, m
#define LAND_TIME 10000 // time to verify landing, ms

// Hardware constants
#define P_LED 13
#define P_FIRST_SW 2 // switch pins continue in numerical order from here
#define P_BUTTON 10
#define P_BUZZER 12
#define P_PETAL 11

// must be a unit vector
const VectorF upVector = VectorF(1, 0, 0); // for as-designed orientation; ideally this will be found dynamically but not right now

unsigned long int minSample = 1000l; // microseconds
unsigned long int lastSample = 0; // microseconds
unsigned long int lastRead = 0; // microseconds

unsigned long int tLiftoff = 0;
unsigned long int tBurnout = 0;
unsigned long int tApogee = 0;
unsigned long int tLand = 0;
float groundAlt = 0;
float maxAlt = 0;
float landAlt = 0;

float speed = LIFTOFF_THRUST / LIFTOFF_MASS * LIFTOFF_TIME / 1000.0;
float Cd = EST_CD;

char filename[13]; // 8 characters + '.' + 3 characters + NULL

#define DEBUG

// get these value by running H3LIS331DL_AdjVal
#define VAL_X_AXIS  139
#define VAL_Y_AXIS  -52
#define VAL_Z_AXIS  118

// 64 byte data frame
struct DataFrame {
  uint32_t time;
  uint8_t state;
  bool accelState;
  uint8_t actuations;
  uint8_t temperature;
  float altitude;
  VectorF accel, ori, gyro;
  float cd;
  float speed;
  float expected;
  uint32_t unassigned_bytes;
  DataFrame() {  }
};


unsigned long int frameIndex = 0;
#define N_FRAMES 1 // 512-read-frame data buffer

enum FlightState {
  READY = 'r',
  BOOST = 'b',
  COAST = 'c',
  BRAKE = 'p',
  DESCENT = 'd',
  LAND = 'l',
  NONE = 'n'
};
enum AccelState {
	BNO = false,
	H3L = true
};
uint8_t actCount = 1;

FlightState currentState = READY;

// Sensor objects
BMP280_DEV baro = BMP280_DEV();
Adafruit_H3LIS331 h3l = Adafruit_H3LIS331();
Adafruit_BNO055 bno = Adafruit_BNO055();

// Data storage variables
struct DataFrame currentFrame;
struct DataFrame frameQueue[N_FRAMES];

bool isSD;

unsigned long int start = 0;
void setup() {
  Serial.begin(115200);
  
  // initialize pins
  pinMode(P_BUTTON, INPUT_PULLUP);
  for(int i = 0; i < 8; i++) {
    pinMode(P_FIRST_SW+i, INPUT_PULLUP);
  }
  pinMode(P_LED, OUTPUT);
  pinMode(P_PETAL, OUTPUT);
  pinMode(P_BUZZER, OUTPUT);

  if(!baro.begin(0x76)) {
    Serial.println("Barometer failed!");
    while(1);
  }
  baro.setTimeStandby(TIME_STANDBY_05MS);
  baro.setPresOversampling(OVERSAMPLING_X2);
  baro.setTempOversampling(OVERSAMPLING_X1); // ~125Hz effective sample rate
  baro.startNormalConversion();

  h3l.begin_I2C();
  h3l.setRange(H3LIS331_RANGE_100_G);
  h3l.setDataRate(LIS331_DATARATE_1000_HZ);
  
  if(!bno.begin()) {
  #ifdef DEBUG
    Serial.println("BNO055 failed!");
    while(1);
  #endif
  }

  isSD = bootSD();

  Serial.println("ready");

  digitalWrite(P_PETAL, HIGH);
  delay(250);
  digitalWrite(P_PETAL, LOW);
  delay(250);
  
  start = millis();
  lastRead = micros();
}

long int count = 0;

void loop() {
  sensors_event_t event;
  
  AccelState accelMode = BNO;
  
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL);
  if(fromAda(event.acceleration).magSq() > H3L_THRESH*H3L_THRESH) { // squaring rather than sqrt is faster
    h3l.getEvent(&event);
	  accelMode = H3L;
  }

  speed += fromAda(event.acceleration).dot(upVector) * (lastRead - micros()) / 1000000.0;
  lastRead = micros();
  
  float temp;
  float pres;
  if(micros()-lastSample > minSample && baro.getMeasurements(temp, pres, currentFrame.altitude)) {
    // Read from non-accel sensors
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&event, Adafruit_BNO055::VECTOR_GYROSCOPE);
    lastSample = micros();

    // organize everything into the frame container
    currentFrame.time = millis();
    currentFrame.state = currentState;
    currentFrame.accelState = accelMode;
    currentFrame.actuations = actCount;
    currentFrame.temperature = temp*4; // fixed point decimal
    
    currentFrame.accel = fromAda(event.acceleration);
    currentFrame.ori = fromAda(event.orientation);
    currentFrame.gyro = fromAda(event.gyro);
    currentFrame.speed = 0;
    currentFrame.cd = 0;
    currentFrame.expected = 0;

    
    switch(currentState) {
      case READY:
        digitalWrite(P_PETAL, LOW);
        getLiftoff();
        // do some funky stuff for a rolling buffer here
      
        break;
      case BOOST:
        digitalWrite(P_PETAL, LOW);
        getBurnout();
        break;
      case COAST:
        digitalWrite(P_PETAL, LOW);
        getApogee();
        break;
      case BRAKE:
        digitalWrite(P_PETAL, HIGH);
        getApogee();
        break;
      case DESCENT:
        digitalWrite(P_PETAL, LOW);
        getLand();
        break;
      case LAND:
        digitalWrite(P_PETAL, LOW);
        break;
      case NONE:
      default:
        digitalWrite(P_PETAL, LOW);
        break;
    } 
    
    
    if(isSD) {
      frameQueue[frameIndex] = currentFrame;
      frameIndex++;

      if(frameIndex == N_FRAMES) {
        File f = SD.open(filename, FILE_WRITE);
        char* dataBytes = (char*) frameQueue;
        f.write(dataBytes, sizeof(struct DataFrame)*N_FRAMES);
        f.close();

        frameIndex = 0;
      }
    }
  }
  
}

/**
 * getLiftoff()
 * 
 * Sets the state to BOOST if acceleration has remained above LIFTOFF_THRESH for LIFTOFF_TIME milliseconds.
 * Should be run frequently.
 */
void getLiftoff() {
  if(currentFrame.accel.dot(upVector) < LIFTOFF_THRESH) {
    tLiftoff = currentFrame.time;
  } else if(currentFrame.time - tLiftoff > LIFTOFF_TIME) {
    currentState = BOOST;
  }
}

/**
 * getBurnout()
 * 
 * Sets the state to COAST if acceleration has remained nonpositive for BURNOUT_TIME milliseconds.
 */
void getBurnout() {
  if(currentFrame.accel.dot(upVector) > 0) {
    tBurnout = currentFrame.time;
  } else if(currentFrame.time - tBurnout > BURNOUT_TIME) {
    currentState = COAST;
  }
}

/**
 * getApogee()
 * 
 * Sets the state to DESCENT if altitude has remained below its maximum by APOGEE_THRESH for APOGEE_TIME milliseconds.
 */
void getApogee() {
  if(maxAlt - currentFrame.altitude < APOGEE_THRESH) {  
    if(currentFrame.altitude > maxAlt) { maxAlt = currentFrame.altitude; }
    tApogee = currentFrame.time;
  } else if(currentFrame.time - tApogee > APOGEE_TIME) {
    currentState = DESCENT;
  }
}

/**
 * getLand()
 * 
 * Sets the state to LAND if altitude has not left a range of +/-LAND_THRESH for LAND_TIME milliseconds.
 */
void getLand() {
  if(abs(currentFrame.altitude - landAlt) > LAND_THRESH) {
    landAlt = currentFrame.altitude;
    tLand = currentFrame.time;
  } else if(currentFrame.time - tLand > LAND_TIME) {
    currentState = LAND;
  }
}

/**
 * bootSD()
 * 
 * Initializes the SD card. Returns true if a file was successfully found, false otherwise
 */
bool bootSD() {
  if (!SD.begin(BUILTIN_SDCARD)) { return false; }

  byte i = 0;
  strcpy(filename, "flight00.txt");
  while (i < 100) { // iterate until we find a file that still hasn't been written
    byte dig1 = i%10;
    filename[7] = '0' + dig1;
    filename[6] = '0' + ((i-dig1)/10) % 10;
    Serial.println(filename);
    if (!SD.exists(filename)) { // if the file is available...
      break; // we're done here
    }
    i++;
  }
  if(i == 100) { return false; }

  File f = SD.open(filename, FILE_WRITE);
  f.close();
  
  return true;
}

/**
 * readSwitches
 * 
 * Reads the whole array of switches; returns a byte bitmask of all switch states. 
 */
byte readSwitches() {
  byte state = 0;
  for(int i = 0; i < 8; i++) {
    state += digitalRead(P_FIRST_SW+i) << i;
  }
  return state;
}

/**
 * fromAda(adaVector)
 * 
 * Converts an Adafruit_Sensor vector into a VectorMath.cpp VectorF.
 */
VectorF fromAda(const sensors_vec_t &adaVector) { return VectorF(adaVector.x, adaVector.y, adaVector.z); }

/**
 * estAlt()
 * 
 * Calculates the rocket's current estimated altitude
 */
float estAlt() {
  // shorten some variables for readability
  const float z = currentFrame.altitude;
  const float m = LIFTOFF_MASS-PROP_MASS;
  const float A = REF_AREA;
  const float v = speed;
  
  // the MAGIC EQUATION courtesy of Maple
  return z + m*(-log(2.0) + log((A*Cd*RHO*v*v + 2.0*G*m)/(G*m))) / (A*Cd*RHO);
}
