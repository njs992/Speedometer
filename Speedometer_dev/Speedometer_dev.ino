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
  //oled:
  oled.begin();
  oled.clear(ALL);
  oled.display();  

  //imu:
  Wire.begin();
  Serial.begin(38400);

    // Set up the interrupt pin, its set as active high, push-pull
  pinMode(intPin, INPUT);
  digitalWrite(intPin, LOW);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // The next call delays for 4 seconds, and then records about 15 seconds of
    // data to calculate bias and scale.
//    myIMU.magCalMPU9250(myIMU.magBias, myIMU.magScale);
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);
//    delay(2000); // Add delay to see results before serial spew of data

    if(SerialDebug)
    {
      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }


  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

}

void loop()
{
//  drawCube();
//  delay(ROTATION_SPEED);

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
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    if (true) //myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
        // Print acceleration values in milligs!
//        Serial.print("X-acceleration: "); Serial.print(1000 * myIMU.ax);
//        Serial.print(" mg ");
//        Serial.print("Y-acceleration: "); Serial.print(1000 * myIMU.ay);
//        Serial.print(" mg ");
//        Serial.print("Z-acceleration: "); Serial.print(1000 * myIMU.az);
//        Serial.println(" mg ");
//
//          oled.print("                                              ");
//          oled.setCursor(0,0);
//          oled.print("X: ");
//          oled.print(1000 * myIMU.ax);
//          oled.setCursor(0,8);
//          oled.print("Y: ");
//          oled.print(1000 * myIMU.ay);
//          oled.setCursor(0,16);
//          oled.print("Z: ");
//          oled.print(1000 * myIMU.az);
//          oled.print("                                              ");
//          oled.circle(LCDWIDTH/2, LCDHEIGHT/2, LCDHEIGHT/2-1);
//          oled.pixel(LCDWIDTH/2 + (myIMU.ax * LCDHEIGHT/2), LCDHEIGHT/2 - (myIMU.ay * LCDHEIGHT/2));
//          oled.display();
//        // Print gyro values in degree/sec
//        Serial.print("X-gyro rate: "); Serial.print(myIMU.gx, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Y-gyro rate: "); Serial.print(myIMU.gy, 3);
//        Serial.print(" degrees/sec ");
//        Serial.print("Z-gyro rate: "); Serial.print(myIMU.gz, 3);
//        Serial.println(" degrees/sec");
//
//        // Print mag values in degree/sec
//        Serial.print("X-mag field: "); Serial.print(myIMU.mx);
//        Serial.print(" mG ");
//        Serial.print("Y-mag field: "); Serial.print(myIMU.my);
//        Serial.print(" mG ");
//        Serial.print("Z-mag field: "); Serial.print(myIMU.mz);
//        Serial.println(" mG");

        myIMU.tempCount = myIMU.readTempData();  // Read the adc values
        // Temperature in degrees Centigrade
        myIMU.temperature = ((float) myIMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
//        Serial.print("Temperature is ");  Serial.print(myIMU.temperature, 1);
//        Serial.println(" degrees C");
      }

      myIMU.count = millis();
    } // if (myIMU.delt_t > 500)
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;

    // update LCD once per half-second independent of read rate
    if (true) //myIMU.delt_t > 500)
    {
      if(SerialDebug)
      {
//        Serial.print("ax = ");  Serial.print((int)1000 * myIMU.ax);
//        Serial.print(" ay = "); Serial.print((int)1000 * myIMU.ay);
//        Serial.print(" az = "); Serial.print((int)1000 * myIMU.az);
//        Serial.println(" mg");
//
//        Serial.print("gx = ");  Serial.print(myIMU.gx, 2);
//        Serial.print(" gy = "); Serial.print(myIMU.gy, 2);
//        Serial.print(" gz = "); Serial.print(myIMU.gz, 2);
//        Serial.println(" deg/s");
//
//        Serial.print("mx = ");  Serial.print((int)myIMU.mx);
//        Serial.print(" my = "); Serial.print((int)myIMU.my);
//        Serial.print(" mz = "); Serial.print((int)myIMU.mz);
//        Serial.println(" mG");
//
//        Serial.print("q0 = ");  Serial.print(*getQ());
//        Serial.print(" qx = "); Serial.print(*(getQ() + 1));
//        Serial.print(" qy = "); Serial.print(*(getQ() + 2));
//        Serial.print(" qz = "); Serial.println(*(getQ() + 3));
      }

// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;
      myIMU.roll  *= RAD_TO_DEG;

//      accX = 1000 * myIMU.ax;
//      accY = 1000 * myIMU.ay;
//      accZ = 1000 * myIMU.az;
      accMag = sqrt(pow(accX, 2) + pow(accY, 2) + pow(accZ, 2)) / 1000;
      
      oled.print("                                              ");//clearscreen
      
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
      
//      oled.setCursor(0,0);
//      oled.print("X: ");
//      oled.print(accX);
//      oled.setCursor(0,8);
//      oled.print("Y: ");
//      oled.print(accY);
//      oled.setCursor(0,16);
//      oled.print("Z: ");
//      oled.print(accZ);
//      oled.setCursor(0,24);
//      oled.print("Magnitude:");
//      oled.setCursor(0,32);
//      oled.print(accMag, 2);
      
//      oled.print("Yw: ");
//      oled.print(myIMU.yaw, 2);
//      oled.setCursor(0,32);
//      oled.print("Rl: ");
//      oled.print(myIMU.roll, 2);
//      oled.setCursor(0,40);
//      oled.print("Pt: ");
//      oled.print(myIMU.pitch, 2);

      oled.display();//pushtodisplay

      if(SerialDebug)
      {
//        Serial.print("Yaw, Pitch, Roll: ");
//        Serial.print(myIMU.yaw, 2);
//        Serial.print(", ");
//        Serial.print(myIMU.pitch, 2);
//        Serial.print(", ");
//        Serial.println(myIMU.roll, 2);
//
//        Serial.print("rate = ");
//        Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
//        Serial.println(" Hz");
      }

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // if (AHRS)
}

void drawCube()
{
  r[0]=r[0]+PI/180.0; // Add a degree
  r[1]=r[1]+PI/180.0; // Add a degree
  r[2]=r[2]+PI/180.0; // Add a degree
  if (r[0] >= 360.0*PI/180.0) r[0] = 0;
  if (r[1] >= 360.0*PI/180.0) r[1] = 0;
  if (r[2] >= 360.0*PI/180.0) r[2] = 0;

  for (int i=0;i<8;i++)
  {
    float px2 = px[i];
    float py2 = cos(r[0])*py[i] - sin(r[0])*pz[i];
    float pz2 = sin(r[0])*py[i] + cos(r[0])*pz[i];

    float px3 = cos(r[1])*px2 + sin(r[1])*pz2;
    float py3 = py2;
    float pz3 = -sin(r[1])*px2 + cos(r[1])*pz2;

    float ax = cos(r[2])*px3 - sin(r[2])*py3;
    float ay = sin(r[2])*px3 + cos(r[2])*py3;
    float az = pz3-150;

    p2x[i] = SCREEN_WIDTH/2+ax*SHAPE_SIZE/az;
    p2y[i] = SCREEN_HEIGHT/2+ay*SHAPE_SIZE/az;
  }

  oled.clear(PAGE);
  for (int i=0;i<3;i++) 
  {
    oled.line(p2x[i],p2y[i],p2x[i+1],p2y[i+1]);
    oled.line(p2x[i+4],p2y[i+4],p2x[i+5],p2y[i+5]);
    oled.line(p2x[i],p2y[i],p2x[i+4],p2y[i+4]);
  }    
  oled.line(p2x[3],p2y[3],p2x[0],p2y[0]);
  oled.line(p2x[7],p2y[7],p2x[4],p2y[4]);
  oled.line(p2x[3],p2y[3],p2x[7],p2y[7]);
  oled.display();
}
