#include <Servo.h>
#include <AccelStepper.h>
#include <EEPROM.h>

typedef struct
{
  int achsen[6];
  int greifer;
} Position;

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

Servo gripper;

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
int count1 = 0;
int count2 = 0;
int count3 = 0;
int closed = 160;
int opened = 20;
int pos = 20;
int joysticks[] =                   {joystick_x1, joystick_y1, joystick_x2, joystick_y2, joystick_x3, joystick_y3};
const int stp_pins[] =              {stp1, stp2, stp3, stp4, stp5, stp6};
const int dir_pins[] =              {dir1, dir2, dir3, dir4, dir5, dir6};
const int joystick_key_pins [] =    {joystick_key1, joystick_key2, joystick_key3};


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

  gripper.attach(grip);

  digitalWrite(joystick_key1, HIGH);
  digitalWrite(joystick_key2, HIGH);
  digitalWrite(joystick_key3, HIGH);

  calibrateJoysticks();
  Serial.println("Setup gmacht");
  gripper.write(opened);
  Home();
}

void loop()
{
  AusgabePosition();
  manuell_Fahren();
  Tastenabfrage();
}

void setPinsTo(int mode, const int pins[], int count)
{
  for (int i = 0; i < count; i++)
  {
    pinMode(pins[i], mode);
  }
}

void calibrateJoysticks()
{
  for (int i = 0; i < 6; i++)
  {
    analog_mid[i] = analogRead(joysticks[i]);
  }
}

void AusgabePosition()
{
  for (int i = 0; i < 6; i++)
  {
    posi[i] = achsen[i].currentPosition() / winkelfaktor[i];
  }
  bool send_pos = true;

  if (millis() - previousMicros >= 2000)
  {
    if (send_pos == true)
    {
      send_pos = false;
      previousMicros = millis();

      //Winkelausgabe:

      Serial.println("Achsposition in Steps ");
      for (int i = 0; i < 6; i++) {
        Serial.print(i);
        Serial.print(": ");
        Serial.print(achsen[i].currentPosition());
        Serial.println(" ");
      }
      Serial.println();
      AusgabeManual();
      /*
            Serial.print("Moving: ");
            for (int i = 0; i < 6; i++) {
              if (achsen[i].isRunning()) {
                Serial.print(i);
                Serial.print(": ");
                Serial.print(achsen[i].targetPosition());
                Serial.print(",");
                Serial.print(achsen[i].distanceToGo());
                Serial.print(",");
                Serial.print(achsen[i].currentPosition());
                Serial.print(" ");

              }
            }*/
      Serial.println();
    }
  }
}

void Greifer()
{
  Serial.println("Greiferaktion");
  if (gripper_state == 1)
  {
    for (pos = opened; pos <= closed; pos += 2)
    {
      gripper.write(pos);
      delay(5);
    }
    gripper_state = !gripper_state;
    Serial.println("Greifer geschlossen");
  }
  else
  {
    for (pos = closed; pos >= opened; pos -= 2)
    {
      gripper.write(pos);
      delay(5);
    }
    gripper_state = !gripper_state;
    Serial.println("Greifer offen");

  }
}

void Tastenabfrage()
{
  //Taste 1 kurz - Keine Funktion
  //Taste 1 lang - Punkt speichern
  //Taste 2 kurz - Greifer oeffnen/schliessen
  //Taste 2 lang - Home
  //Taste 3 kurz - Keine Funktion
  //Taste 3 lang - Punkt Anfahren
  //Taste 2 & 3 zusammen - Zero

  //Taste 1
  if (digitalRead(joystick_key1) == LOW)
  {
    //Serial.println(count1);
    if (millis() - previousMicros >= 100)
    {
      count1++;
      previousMicros = millis();
      if (count1 >= 10)
      {
        //Aktion für langen Tastendruck
        punkt_speichern();
      }
    }
    if (count1 < 10)
    {
      //Aktion für kurzen Tastendruck
    }
  }
  else
  {
    count1 = 0;
  }

  //Taste 2

  if (digitalRead(joystick_key2) == LOW)
  {
    delay(50);
    if (digitalRead(joystick_key2) == LOW)
    {
      count2++;
      if (count2 >= 20)
      {
        Home();
        count2 = 0;
        do
        {
        }
        while (digitalRead(joystick_key2) == LOW);
      }
    }
    else
    {
      Greifer();
      count2 = 0;
    }
  }

  /*

    if (digitalRead(joystick_key2) == LOW)
    {
      //Serial.println(count2);
      if (millis() - previousMicros >= 100)
      {
        count2++;
        previousMicros = millis();
      }
      if (digitalRead(joystick_key2) == HIGH)
      {
        if (count2 < 10)
        {
          Greifer();
          count2 = 0;
        }
        else
        {
          Home();
          count2 = 0;
        }
      }
    }
  */

  //Taste 3
  if (digitalRead(joystick_key3) == LOW)
  {
    if (millis() - previousMicros >= 100)
    {
      count3++;
      previousMicros = millis();
      if (count3 >= 10)
      {
        //Aktion für langen Tastendruck
        automatisch_Fahren();
      }
      if (count3 < 10)
      {
        //Aktion für kurzen Tastendruck
      }
    }
  }
  else
  {
    count3 = 0;
  }
  if (digitalRead(joystick_key3) == LOW && digitalRead(joystick_key2) == LOW)
  {
    Zero();
  }
}


void punkt_speichern()
{
  Position pos;
  for (int i = 0; i < 6; i++)
  {
    pos.achsen[i] = achsen[i].currentPosition();
    Serial.println("schreiben");
    speichern(0, pos);
    Serial.println("geschrieben");
  }
}

void automatisch_Fahren()
{
  Position pos = { { 0, 0, 0, 0, 0, 0}, 0 };
  Serial.println("wird geladen");
  laden(0, pos);
  Serial.println("geladen");
  pos_anfahren(pos, 10);
  Serial.println("angefahren");
  Serial.println(pos.achsen[0]);
}

void manuell_Fahren()
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
    Serial.println("Alle Achsen genullt");
  }
}

void Home()
{
  Serial.println("Homing im Gange");
  for (int i = 0; i < 6; i++)
  {
    achsen[i].setMaxSpeed(10000);
    achsen[i].setAcceleration(10000);
    achsen[i].moveTo(0);
  }
  int moving = 0;
  do
  {
    moving = 0;
    for (int i = 0; i < 6; i++)
    {
      if (achsen[i].run())
      {
        moving ++;
      }
    }
  }
  while (moving > 0);
}

void pos_anfahren(Position pos, int geschwindigkeit)
{
  Serial.print("Anfahren: ");
  for (int i = 0; i < 6; i++) {
    achsen[i].setMaxSpeed(10000);
    achsen[i].setAcceleration(10000);
    achsen[i].moveTo(pos.achsen[i]);
  }
  int moving = 0;
  do
  {
    moving = 0;
    for (int i = 0; i < 6; i++)
    {
      if (achsen[i].run())
      {
        moving ++;
      }
    }
  }
  while (moving > 0);
}

void speichern(int index, Position pos)
{
  int addr = index * sizeof(Position);
  uint8_t* p_pos = reinterpret_cast<char*>(&pos);
  for (int i = 0; i < sizeof(Position); i++)
  {
    EEPROM.update(addr + i, *(p_pos + i));
  }
}

void laden(int index, Position & pos)
{
  int addr = index * sizeof(Position);
  uint8_t* p_pos = reinterpret_cast<char*>(&pos);
  for (int i = 0; i < sizeof(Position); i++)
  {
    *(p_pos + i) = EEPROM.read(addr + i);
  }
}
void AusgabeManual()
{
  Serial.println("Optionen: ");
  Serial.println("Taste 1 kurz - Keine Funktion");
  Serial.println("Taste 1 lang - Punkt speichern");
  Serial.println("Taste 2 kurz - Greifer öffnen/schliessen");
  Serial.println("Taste 2 lang - Home");
  Serial.println("Taste 3 kurz - Keine Funktion");
  Serial.println("Taste 3 lang - Punkt Anfahren");
  Serial.println("Taste 2 & 3 zusammen - Zero");
}

