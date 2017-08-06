#include <Servo.h>

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

int analog_max = 1025;
int analog_min = 0;
int analog_mid[6];
int spd_val[6];
int max_spd = 20;
int min_spd = 10000;
int gripper_state = 1;
int offen = 1;
int geschlossen = 0;
int print_pos = 0;

float winkel[6];
int steps[6];
float winkelfaktor[] = {33, 160, 70, 23, 40, 40};
unsigned long previousMillis[6];
unsigned long previousMicros;
bool step_val[6];

Servo gripper;
int closed = 180;
int opened = 0;
int pos = 0;

void setPinsTo(int mode, const int pins[], int count) {
  for(int i = 0;i < count; i++) {
    Serial.print("mode of pin ");
    Serial.print(pins[i]);
    Serial.print(": ");
    Serial.println(mode);
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

void setup() {
  Serial.begin(115200);

  setPinsTo(OUTPUT, dir_pins, 6);
  setPinsTo(OUTPUT, stp_pins, 6);
  setPinsTo(OUTPUT, (const int[]){grip}, 1);
  setPinsTo(INPUT , joystick_key_pins, 3);

  calibrateJoysticks();

  gripper.attach(grip);
  gripper.write(165);
}

void a(int dir_pin, int stp_pin, bool dirValue, int spd_val, long unsigned& previousMicros, bool& step_val)
{
  if (micros() - previousMicros >= spd_val)
  {
    previousMicros = micros();
    digitalWrite(dir_pin, dirValue);
    digitalWrite(stp_pin, step_val);
    step_val = !step_val;
    /*
    Serial.print("pin nummer ");
    Serial.print("step-pin ");
    Serial.print(stp_pin);
    Serial.print(" dir-pin ");
    Serial.println(dir_pin);
    */
  }
}

const bool PLUS = HIGH;
const bool MINUS = LOW;

bool achse(int joystick_pin, int index, int offset) {
  int analogValue = analogRead(joystick_pin);
  bool valueChanged = false;
  if (analogValue > analog_mid[index] + offset) {
    int spd_val = map(analogValue, analog_mid[index], analog_max, min_spd, max_spd);
    steps[index] +=1;
    a(dir_pins[index], stp_pins[index], PLUS, spd_val, previousMillis[index], step_val[index]);
    valueChanged = true;
  }
  if (analogValue < analog_mid[index] - offset) {
    int spd_val = map(analogValue, analog_min, analog_mid[index], max_spd, min_spd);
    steps[index] -=1;
    a(dir_pins[index], stp_pins[index], MINUS,  spd_val, previousMillis[index], step_val[index]);
    valueChanged = true;
  }
  return valueChanged;
}

void loop() {

  bool send_pos = true;

  if (millis() - previousMicros >= 1000)
  {
    if (send_pos == true)
    {
      send_pos = false;
      previousMicros = millis();

      /*
            //Analogausgabe:
              Serial.print("Joystick-Wert 1 X ");
              Serial.println(analogRead(joystick_x1));
              Serial.print("Joystick-Wert 1 Y ");
              Serial.println(analogRead(joystick_y1));
              Serial.print("Joystick-Wert 2 X ");
              Serial.println(analogRead(joystick_x2));
              Serial.print("Joystick-Wert 2 Y ");
              Serial.println(analogRead(joystick_y2));
              Serial.print("Joystick-Wert 3 X ");
              Serial.println(analogRead(joystick_x3));
              Serial.print("Joystick-Wert 3 Y ");
              Serial.println(analogRead(joystick_y3));
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
              Serial.println("");
      

      //Winkelausgabe:

      Serial.print("Achse 1 ");
      Serial.println(steps[0] / winkelfaktor[0]);
      Serial.print("Achse 2 ");
      Serial.println(steps[1] / winkelfaktor[1]);
      Serial.print("Achse 3 ");
      Serial.println(steps[2] / winkelfaktor[2]);
      Serial.print("Achse 4 ");
      Serial.println(steps[3] / winkelfaktor[3]);
      Serial.print("Achse 5 ");
      Serial.println(steps[4] / winkelfaktor[4]);
      Serial.print("Achse 6 ");
      Serial.println(steps[5] / winkelfaktor[5]);
      Serial.print("Joystick-Wert 1 X ");
      Serial.println(analogRead(joystick_x1));
      Serial.print("Joystick-Wert 1 Y ");
      Serial.println(analogRead(joystick_y1));
      Serial.print("Joystick-Wert 2 X ");
      Serial.println(analogRead(joystick_x2));
      Serial.print("Joystick-Wert 2 Y ");
      Serial.println(analogRead(joystick_y2));
      Serial.print("Joystick-Wert 3 X ");
      Serial.println(analogRead(joystick_x3));
      Serial.print("Joystick-Wert 3 Y ");
      Serial.println(analogRead(joystick_y3));
      /*
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
            Serial.println("");
      */
    }
  }

  for (int i = 5; i >= 0 ; i--) {
    bool valueChanged = achse(joysticks[i], i, 30);
    send_pos = valueChanged | send_pos;
  }
}
