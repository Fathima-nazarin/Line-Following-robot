/*
  PID Line Follower — ported from Arduino Nano to ESP32
  ------------------------------------------------------
  Library needed (same as before, it works fine on ESP32 with
  Arduino-ESP32 core 3.x, which added native analogWrite support):
    Arduino IDE -> Tools -> Manage Libraries -> search "SparkFun TB6612FNG"
    (SparkFun_TB6612Motor library) -> Install

  Key changes made for ESP32:
  1. Pin numbers changed to valid ESP32 GPIOs (Nano pin numbers like
     3,4,5,6,7,9,10,11,12 don't exist on ESP32 the same way).
  2. Motor driver control pins moved to GPIOs that support digital
     output + PWM (avoided strapping pins 0/2/5/12/15 and
     input-only pins 34-39).
  3. Sensor pins moved to ADC1-capable GPIOs (32,33,34,35,36,39).
     ADC1 is used (not ADC2) because ADC2 is unusable while WiFi is
     active on ESP32 — doesn't matter here since we don't use WiFi,
     but it's good practice.
  4. analogReadResolution(10) is set in setup() so analogRead()
     still returns 0-1023 like the Nano, keeping all your existing
     threshold/PID math valid without rescaling.
  5. Start/stop push-buttons moved to two free GPIOs with
     INPUT_PULLUP (same behavior as before).
  6. Sensor pin access changed from analogRead(1..5) (Nano analog
     pin numbers) to analogRead(sensorPins[i]) using an array of the
     6 ESP32 sensor GPIOs. Indices 0-5 now map to sensors 0-5
     (previously your code only used indices 1-5, leaving index 0
     unused — that sensor's pin is still read during calibration
     for consistency, but note your line-following logic still only
     references indices 1,2,3,4,5, unchanged from the original).

  Wiring reference for this sketch:
    Sensor 0 -> GPIO36 (VP)
    Sensor 1 -> GPIO39 (VN)
    Sensor 2 -> GPIO34
    Sensor 3 -> GPIO35
    Sensor 4 -> GPIO32
    Sensor 5 -> GPIO33
    AIN1     -> GPIO16
    AIN2     -> GPIO17
    PWMA     -> GPIO21
    BIN1     -> GPIO18
    BIN2     -> GPIO19
    PWMB     -> GPIO22
    STBY     -> GPIO23
    Start btn-> GPIO25 (button to GND, uses internal pullup)
    Stop btn -> GPIO26 (button to GND, uses internal pullup)

  If your ESP32 board is a WROVER module using GPIO16/17 for PSRAM,
  change AIN1/AIN2 to two other free GPIOs (e.g. 4 and 13).
*/

#include <SparkFun_TB6612.h>

// ---- Motor driver pins (ESP32 GPIOs) ----
#define AIN1 16
#define BIN1 18
#define AIN2 17
#define BIN2 19
#define PWMA 21
#define PWMB 22
#define STBY 23

// ---- Push buttons ----
#define START_BTN 25
#define STOP_BTN  26

// these constants are used to allow you to make your motor configuration
// line up with function names like forward. Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors. The library will allow you to initialize as many
// motors as you have memory for.
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;
float Kp = 0;
float Kd = 0;
float Ki = 0;

// ---- Sensor pins (ESP32 ADC1-capable GPIOs) ----
// index:      0     1     2     3     4     5
const int sensorPins[6] = {36, 39, 34, 35, 32, 33};

int minValues[6], maxValues[6], threshold[6];

void setup()
{
  Serial.begin(115200);

  // Keep the same 10-bit (0-1023) analogRead range as the Nano so
  // all existing threshold/PID math works unchanged.
  analogReadResolution(10);

  pinMode(START_BTN, INPUT_PULLUP);
  pinMode(STOP_BTN, INPUT_PULLUP);
}

void loop()
{
  while (digitalRead(START_BTN)) {}
  delay(1000);
  calibrate();
  while (digitalRead(STOP_BTN)) {}
  delay(1000);

  while (1)
  {
    if (analogRead(sensorPins[1]) > threshold[1] && analogRead(sensorPins[5]) < threshold[5])
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }
    else if (analogRead(sensorPins[5]) > threshold[5] && analogRead(sensorPins[1]) < threshold[1])
    {
      lsp = lfspeed; rsp = 0;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }
    else if (analogRead(sensorPins[3]) > threshold[3])
    {
      Kp = 0.0006 * (1000 - analogRead(sensorPins[3]));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  }
}

void linefollow()
{
  int error = (analogRead(sensorPins[2]) - analogRead(sensorPins[4]));
  P = error;
  I = I + error;
  D = error - previousError;
  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfspeed - PIDvalue;
  rsp = lfspeed + PIDvalue;

  if (lsp > 255) lsp = 255;
  if (lsp < 0)   lsp = 0;
  if (rsp > 255) rsp = 255;
  if (rsp < 0)   rsp = 0;

  motor1.drive(lsp);
  motor2.drive(rsp);
}

void calibrate()
{
  for (int i = 0; i < 6; i++)
  {
    minValues[i] = analogRead(sensorPins[i]);
    maxValues[i] = analogRead(sensorPins[i]);
  }

  for (int i = 0; i < 3000; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);
    for (int j = 0; j < 6; j++)
    {
      int val = analogRead(sensorPins[j]);
      if (val < minValues[j]) minValues[j] = val;
      if (val > maxValues[j]) maxValues[j] = val;
    }
  }

  for (int i = 0; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();

  motor1.drive(0);
  motor2.drive(0);
}
