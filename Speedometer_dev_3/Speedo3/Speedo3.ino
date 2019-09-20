#include <AccelStepper.h>

// Define a stepper and the pins it will use
AccelStepper stepper(AccelStepper::DRIVER, 2, 3);

int pos = 225;

void setup()
{ 
  stepper.setMaxSpeed(50000);
  stepper.setAcceleration(1000);
}

void loop()
{
  if (stepper.distanceToGo() == 0)
  {
    delay(500);
    pos = -pos;
    stepper.moveTo(pos);\
  }
  stepper.run();
}
