#include <BMP280_DEV.h>
#include <Device.h>

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>

#include <SD.h>
#include <SPI.h>

unsigned long int minSample = 1000l; // microseconds
unsigned long int lastSample = 0;
unsigned long int lastRead = 0;

char filename[13]; // 8 characters + '.' + 3 characters + NULL

#define DEBUG

// get these value by running H3LIS331DL_AdjVal
#define VAL_X_AXIS  139
#define VAL_Y_AXIS  -52
#define VAL_Z_AXIS  118


struct DataFrame {
  uint32_t time;
  float altitude;
  // 8 byes
  int16_t temp;
  int16_t accelX;
  int16_t accelY;
  int16_t accelZ;
  // 16 bytes
  float speed;
  float Cd;
  // 24 bytes
  float expAlt;
  char flightState;
  char freespace[1];
  int16_t bufferPos; // maybe
  // 32 bytes
};

// Hardware constants
#define P_LED 13
#define P_FIRST_SW 2 // switch pins continue in numerical order from here
#define P_BUTTON 10

unsigned long int frameIndex = 0;
#define N_FRAMES 1 // 512-read-frame data buffer

// Sensor objects
BMP280_DEV baro = BMP280_DEV();
Adafruit_H3LIS331 accel = Adafruit_H3LIS331();

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

  if(!baro.begin(0x76)) {
    Serial.println("Barometer failed!");
    while(1);
  }
  baro.setTimeStandby(TIME_STANDBY_05MS);
  baro.setPresOversampling(OVERSAMPLING_X2);
  baro.setTempOversampling(OVERSAMPLING_X1); // ~125Hz effective sample rate
  baro.startNormalConversion();

  accel.begin_I2C();
  accel.setRange(H3LIS331_RANGE_100_G);
  accel.setDataRate(LIS331_DATARATE_1000_HZ);

  isSD = bootSD();

  Serial.println("ready");
  
  start = millis();
  lastRead = micros();
}

long int count = 0;

void loop() {
  sensors_event_t event;

  accel.getEvent(&event);
  
  float temp;
  float pres;
  if(micros()-lastSample > minSample && baro.getMeasurements(temp, pres, currentFrame.altitude)) {
 //   Serial.println("sample");
    lastSample = micros();
    
    currentFrame.time = millis();
    currentFrame.temp = temp*256;
    currentFrame.accelX = accel.x;
    currentFrame.accelY = accel.y;
    currentFrame.accelZ = accel.z;
    currentFrame.speed = 0;
    currentFrame.Cd = 0;
    currentFrame.expAlt = 0;
    currentFrame.flightState = 'p';
    
  #ifdef DEBUG  
    Serial.print(currentFrame.time);
    Serial.print("\t");
    Serial.print(currentFrame.temp);
    Serial.print("\t");
    Serial.print(currentFrame.accelX);
    Serial.print("\t");
    Serial.print(currentFrame.accelY);
    Serial.print("\t");
    Serial.print(currentFrame.accelZ);
    Serial.print("\t");
    Serial.println();
  #endif
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
  digitalWrite(P_LED, digitalRead(P_BUTTON));
  
  lastRead = micros();
}

/**
 * bootSD
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
