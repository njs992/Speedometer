#include <TinyGPS++.h>
#include <AccelStepper.h>
#include <SFE_MicroOLED.h>
#include <Wire.h>
#include <Serial.h>

//Define GPS thing
TinyGPSPlus gps;

//Define MicroOled pins
#define PIN_RESET 9  // Connect RST to pin 9
#define PIN_DC    8  // Connect DC to pin 8
#define PIN_CS    10 // Connect CS to pin 10
#define DC_JUMPER 0

//MicroOled Object Declaration
MicroOLED oled(PIN_RESET, PIN_DC, PIN_CS); // SPI declaration

int SCREEN_WIDTH = oled.getLCDWidth();
int SCREEN_HEIGHT = oled.getLCDHeight();

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

int pos = 1600;
int stepper_position = 0;
int t = 0;

void setup()
{
  //Serial setup
  Serial.begin(9600);
  
  //Stepper setup 
  stepper.setMaxSpeed(50000);
  stepper.setAcceleration(2000);

  //OLED SETUP
  oled.begin();
  oled.clear(ALL);
  oled.display();
  oled.clear(PAGE);
  Wire.begin();
}

void loop()
{
  oled.clear(PAGE);
  oled.setCursor(0,0);
  stepper_position = stepper.currentPosition();
  oled.println(stepper_position);
  /*
  if (stepper.distanceToGo() == 0)
  {
    delay(500);
    pos = -pos;
    stepper.moveTo(pos);
  }*/
  stepper.moveTo(1600);
  stepper.run();
  oled.display();
}
