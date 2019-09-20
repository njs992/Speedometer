#include "quaternionFilters.h"
#include "MPU9250.h"
#include <SFE_MicroOLED.h>  // Include the SFE_MicroOLED library

// MicroOLED Definition
#define PIN_RESET 9  // Connect RST to pin 9
#define PIN_DC    8  // Connect DC to pin 8
#define PIN_CS    10 // Connect CS to pin 10
#define DC_JUMPER 0

// MicroOLED Object Declaration
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); // SPI declaration
//MicroOLED oled(PIN_RESET, DC_JUMPER);    // I2C declaration

int SCREEN_WIDTH = oled.getLCDWidth();
int SCREEN_HEIGHT = oled.getLCDHeight();

float d = 3;
float px[] = { 
  -d,  d,  d, -d, -d,  d,  d, -d };
float py[] = { 
  -d, -d,  d,  d, -d, -d,  d,  d };
float pz[] = { 
  -d, -d, -d, -d,  d,  d,  d,  d };

float p2x[] = {
  0,0,0,0,0,0,0,0};
float p2y[] = {
  0,0,0,0,0,0,0,0};

float r[] = {
  0,0,0};

#define SHAPE_SIZE 600
// Define how fast the cube rotates. Smaller numbers are faster.
// This is the number of ms between draws.
#define ROTATION_SPEED 0

//IMU declarations

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);

double accX, accY, accZ, accMag;

void setup() 
{
//OLED SETUP
  oled.begin();
  oled.clear(ALL);
  oled.display();
  ///////////////todo change the default bitmap

//IMU SETUP
  //I2C AND SERIAL
  Wire.begin();
  Serial.begin(38400);

  //INTERUPT PIN SETUP
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  //READ WHO_AM_I REGISTER TO TEST
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x71) //GOOD
  {
    //RUN SELF TEST
    myIMU.MPU9250SelfTest(myIMU.selfTest);

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

/*
    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    oled.print("                                                            ");
    oled.display();
    oled.setCursor(0,0);
    oled.println("mag bias");
    oled.println(myIMU.magBias[0]);
    oled.println(myIMU.magBias[1]);
    oled.println(myIMU.magBias[2]);
    oled.display();
    delay(5000);

    oled.setCursor(0,0);
    oled.println("mag scale");
    oled.println(myIMU.magScale[0]);
    oled.println(myIMU.magScale[1]);
    oled.println(myIMU.magScale[2]);
    oled.display();
    delay(5000);
    */
    //^^^this test may or may not be valuable...
    
  }
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  ///////////////here is where I'll be able to add the fn stepper motor stuff... WHEN I GET TO IT
}

void loop() 
{
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }

  //ESSENTIAL FOR QUATERNIONS
  myIMU.updateTime();

  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD, myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my, myIMU.mx, myIMU.mz, myIMU.deltat);

  if (myIMU.delt_t > 500) //update every 500 mS
  {
      oled.setCursor(0,0);
      oled.print("Q0: ");
      oled.print(*(getQ()+0), 2);
      oled.setCursor(0,8);
      oled.print("Qx: ");
      oled.print(*(getQ()+1), 2);
      oled.setCursor(0,16);
      oled.print("Qy: ");
      oled.print(*(getQ()+2), 2);
      oled.setCursor(0,24);
      oled.print("Qz:");
      oled.print(*(getQ()+3), 2);     
      oled.display();
   }
  else 
  {
    myIMU.delt_t = millis() - myIMU.count;
  }
}
