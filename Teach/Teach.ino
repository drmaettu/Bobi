#include <Servo.h>
#include <AccelStepper.h>

#define joystick_x1     A0
#define joystick_y1     A1
#define joystick_x2     A2
#define joystick_y2     A3
#define joystick_x3     A4
#define joystick_y3     A5
#define joystick_key1   22
#define joystick_key2   23
#define joystick_key3   24

#define stp1            25
#define dir1            26
#define stp2            27
#define dir2            28
#define stp3            29
#define dir3            30
#define stp4            31
#define dir4            32
#define stp5            33
#define dir5            34
#define stp6            35
#define dir6            36
#define grip            37


AccelStepper achse_1(AccelStepper::DRIVER, stp1, dir1);
AccelStepper achse_2(AccelStepper::DRIVER, stp2, dir2);
AccelStepper achse_3(AccelStepper::DRIVER, stp3, dir3);
AccelStepper achse_4(AccelStepper::DRIVER, stp4, dir4);
AccelStepper achse_5(AccelStepper::DRIVER, stp5, dir5);
AccelStepper achse_6(AccelStepper::DRIVER, stp6, dir6);
AccelStepper achsen[6] = {achse_1, achse_2, achse_3, achse_4, achse_5, achse_6};

int analog_max = 1024;
int analog_min = 0;
int analog_mid[6];
int max_spd = 1000;
int min_spd = 20;
int gripper_state = 1;
int offen = 1;
int geschlossen = 0;
int print_pos = 0;
int joystick_offset = 30;

int spdfaktor[] = {20, 20, 25, 30, 30, 17};                                                   //Geschwindigkeitsfaktor pro Achse
int accfaktor[] = {20, 40, 50, 50, 40, 30};                                                  //Beschleunigung abhängig von der Geschwindigkeit
float winkelfaktor[] = {70, 160, 140, 98, 80, 20};
int analogValue[6];
int spd_val[6];
float posi[6];
float winkel[6];
int steps[6];
unsigned long previousMicros;

Servo gripper;
int closed = 160;
int opened = 20;
int pos = 0;

void setPinsTo(int mode, const int pins[], int count) {
  for (int i = 0; i < count; i++) {
    pinMode(pins[i], mode);
  }
}

int joysticks[] = {joystick_x1, joystick_y1, joystick_x2, joystick_y2, joystick_x3, joystick_y3};
const int stp_pins[] =              {stp1, stp2, stp3, stp4, stp5, stp6};
const int dir_pins[] =              {dir1, dir2, dir3, dir4, dir5, dir6};
const int joystick_key_pins [] =    {joystick_key1, joystick_key2, joystick_key3};

void calibrateJoysticks() {
  for (int i = 0; i < 6; i++) {
    analog_mid[i] = analogRead(joysticks[i]);
  }
}

void setup()
{
  Serial.begin(115200);
  for (int i = 0; i < 6; i++)
  {
    achsen[i].setMaxSpeed(max_spd);
  }

  setPinsTo(OUTPUT, dir_pins, 6);
  setPinsTo(OUTPUT, stp_pins, 6);
  setPinsTo(OUTPUT, (const int[]) {
    grip
  }, 1);
  setPinsTo(INPUT , joystick_key_pins, 3);

  calibrateJoysticks();

  gripper.attach(grip);
  gripper.write(165);

  digitalWrite(joystick_key1, HIGH);
  digitalWrite(joystick_key2, HIGH);
  digitalWrite(joystick_key3, HIGH);
}

void loop()
{
  Position();
  Greifer();
  Fahren();
  for (int i = 0; i < 6; i++)
  {
    posi[i] = achsen[i].currentPosition() / winkelfaktor[i];
  }

  if (digitalRead(joystick_key1) == LOW)
  {
    Home();
  }

  if (digitalRead(joystick_key3) == LOW && digitalRead(joystick_key2) == LOW)
  {
    Zero();
  }
}

void Position()
{
  bool send_pos = true;

  if (millis() - previousMicros >= 1000)
  {
    if (send_pos == true)
    {
      send_pos = false;
      previousMicros = millis();

      //Winkelausgabe:

      Serial.print("Achse 1 ");
      Serial.println(posi[0]);
      Serial.print("Achse 2 ");
      Serial.println(posi[1]);
      Serial.print("Achse 3 ");
      Serial.println(posi[2]);
      Serial.print("Achse 4 ");
      Serial.println(posi[3]);
      Serial.print("Achse 5 ");
      Serial.println(posi[4]);
      Serial.print("Achse 6 ");
      Serial.println(posi[5]);
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
      Serial.println("");
    }
  }
}
void Greifer()
{
  if (digitalRead(joystick_key2) == LOW)
  {
    for (pos = opened; pos <= closed; pos += 2)
    {
      gripper.write(pos);
      delay(5);
    }
  }

  if (digitalRead(joystick_key3) == LOW)
  {
    for (pos = closed; pos >= opened; pos -= 2)
    {
      gripper.write(pos);
      delay(5);
    }
  }
}

void Fahren()
{
  for (int i = 0; i < 6; i++)
  {
    analogValue[i] = analogRead(joysticks[i]);

    if (analogValue[i] > (analog_mid[i] + joystick_offset))
    {
      spd_val[i] = map(analogValue[i], analog_mid[i], analog_max, min_spd, max_spd);
      achsen[i].move(10);
      achsen[i].setSpeed(spd_val[i]);
      achsen[i].runSpeedToPosition();
    }
    if (analogValue[i] < analog_mid[i] - joystick_offset)
    {
      spd_val[i] = map(analogValue[i], analog_min, analog_mid, max_spd, min_spd);
      achsen[i].move(-10);
      achsen[i].setSpeed(spd_val[i]);
      achsen[i].runSpeedToPosition();
    }
  }
}

void Zero()
{
  for (int i = 0; i < 6; i++)
  {
    achsen[i].setCurrentPosition(0);
  }
}

void Home()
{
  for (int i = 0; i < 6; i++)
  {
    achsen[i].setMaxSpeed(10000);
    achsen[i].setAcceleration(10000);
    achsen[i].moveTo(0);
    achsen[i].run();
  }
}

int bewegung(int a_1, int a_2, int a_3, int a_4, int a_5, int a_6, int geschwindigkeit)
{
  winkel[1] = a_1;
  winkel[2] = a_2;
  winkel[3] = a_3;
  winkel[4] = a_4;
  winkel[5] = a_5;
  winkel[6] = a_6;

  achse_1.setMaxSpeed(geschwindigkeit * spdfaktor[1]);
  achse_2.setMaxSpeed(geschwindigkeit * spdfaktor[2]);
  achse_3.setMaxSpeed(geschwindigkeit * spdfaktor[3]);
  achse_4.setMaxSpeed(geschwindigkeit * spdfaktor[4]);
  achse_5.setMaxSpeed(geschwindigkeit * spdfaktor[5]);
  achse_6.setMaxSpeed(geschwindigkeit * spdfaktor[6]);

  achse_1.setAcceleration(geschwindigkeit * accfaktor[1]);
  achse_2.setAcceleration(geschwindigkeit * accfaktor[2]);
  achse_3.setAcceleration(geschwindigkeit * accfaktor[3]);
  achse_4.setAcceleration(geschwindigkeit * accfaktor[4]);
  achse_5.setAcceleration(geschwindigkeit * accfaktor[5]);
  achse_6.setAcceleration(geschwindigkeit * accfaktor[6]);

  achse_1.moveTo(winkel[1] * winkelfaktor[1]);
  achse_2.moveTo(winkel[2] * winkelfaktor[2]);
  achse_3.moveTo(winkel[3] * winkelfaktor[3]);
  achse_4.moveTo(winkel[4] * winkelfaktor[4]);
  achse_5.moveTo(winkel[5] * winkelfaktor[5]);
  achse_6.moveTo(winkel[5] * winkelfaktor[6]);

  do
  {
    achse_1.run();
    achse_2.run();
    achse_3.run();
    achse_4.run();
    achse_5.run();
    achse_6.run();
  }
  while (achse_1.run() or achse_2.run() or achse_3.run() or achse_4.run() or achse_5.run() or achse_6.run());
}
