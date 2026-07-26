/*
  Simple PID controller — ported from Arduino Nano/Uno to ESP32
  ---------------------------------------------------------------
  Key changes made for ESP32:

  1. INPUT_PIN: "A0" and OUTPUT_PIN: "DD3" aren't valid ESP32
     identifiers (A0 isn't defined the same way on all ESP32 cores,
     and DD3 is an AVR register-style name, not an Arduino pin).
     Replaced with explicit GPIO numbers.
       INPUT_PIN  -> GPIO34 (ADC1-capable, input only, fine for
                     reading a sensor)
       OUTPUT_PIN -> GPIO25 (regular GPIO, supports analogWrite/PWM
                     on Arduino-ESP32 core 3.x)

  2. analogRead() range: ESP32 defaults to 12-bit (0-4095), not the
     classic AVR 10-bit (0-1023). Added analogReadResolution(10) in
     setup() so the existing map(analogRead(...), 0, 1024, 0, 255)
     line keeps working exactly as before, unchanged.

  3. analogWrite() range: Arduino-ESP32 core 3.x's analogWrite
     defaults to 8-bit (0-255) just like AVR, so output range is
     unchanged. If you're on an older ESP32 core without native
     analogWrite, you'd need ledcAttach/ledcWrite instead — core
     3.x (current) doesn't need that.

  4. Added constrain(output, 0, 255) before analogWrite. Not in
     your original, but on ESP32 an out-of-range value passed to
     analogWrite can behave unpredictably, so it's a safe addition
     that doesn't change normal-operation behavior.

  Wiring reference:
    Sensor/feedback signal -> GPIO34 (ADC input)
    PWM control output      -> GPIO25
*/

const int INPUT_PIN = 34;   // ADC1-capable input
const int OUTPUT_PIN = 25;  // PWM-capable output

double dt, last_time;
double integral, previous, output = 0;
double kp, ki, kd;
double setpoint = 75.00;

void setup()
{
  kp = 0.8;
  ki = 0.20;
  kd = 0.001;
  last_time = 0;

  Serial.begin(9600);

  // Keep analogRead() on the classic 0-1023 (10-bit) scale so the
  // existing map() call doesn't need to change.
  analogReadResolution(10);

  analogWrite(OUTPUT_PIN, 0);

  for (int i = 0; i < 50; i++)
  {
    Serial.print(setpoint);
    Serial.print(",");
    Serial.println(0);
    delay(100);
  }
  delay(100);
}

void loop()
{
  double now = millis();
  dt = (now - last_time) / 1000.00;
  last_time = now;

  double actual = map(analogRead(INPUT_PIN), 0, 1024, 0, 255);
  double error = setpoint - actual;
  output = pid(error);
  output = constrain(output, 0, 255);
  analogWrite(OUTPUT_PIN, output);

  // Setpoint VS Actual
  Serial.print(setpoint);
  Serial.print(",");
  Serial.println(actual);
  // Error
  //Serial.println(error);

  delay(300);
}

double pid(double error)
{
  double proportional = error;
  integral += error * dt;
  double derivative = (error - previous) / dt;
  previous = error;
  double output = (kp * proportional) + (ki * integral) + (kd * derivative);
  return output;
}
