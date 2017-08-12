#include <AccelStepper.h>

#define stp1            25
#define dir1            26
#define joystick_1x A0

AccelStepper achse_1(AccelStepper::DRIVER, stp1, dir1);

void setup()
{
  achse_1.setMaxSpeed(10000);
  Serial.begin(115200);
}

void loop()
{
  long pos = achse_1.currentPosition() / 90;
  int analogValue = analogRead(joystick_1x);
  if (analogValue > 530 + 30)
  {
    int spd_val = map(analogValue, 530, 1024, 20, 4000);
    achse_1.move(10);
    achse_1.setSpeed(spd_val);
    achse_1.runSpeedToPosition();
  }
  if (analogValue < 530 - 30)
  {
    int spd_val = map(analogValue, 0, 530, 4000, 20);
    achse_1.move(-10);
    achse_1.setSpeed(spd_val);
    achse_1.runSpeedToPosition();

  }
  //Serial.println(pos);
}
