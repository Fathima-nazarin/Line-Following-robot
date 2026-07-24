/*
  HELOO ESP32 Line Follower — BASIC VERSION (no PID)
  Hardware: ESP32 dev board, L298N (or similar) motor driver,
            2x DC gear motors, 5-channel IR array (digital output)

  ---------------- WIRING ----------------
  Motor Driver (L298N):
    ENA (Left motor PWM)  -> GPIO 14
    IN1                   -> GPIO 27
    IN2                   -> GPIO 26
    IN3                   -> GPIO 25
    IN4                   -> GPIO 33
    ENB (Right motor PWM) -> GPIO 32
    (Motor driver GND common with ESP32 GND, motor supply from battery,
     NOT from ESP32 5V/3V3 pin)

  IR Array (5 sensors, digital HIGH = line detected, adjust if inverted):
    S1 (far left)   -> GPIO 34
    S2 (left)       -> GPIO 35
    S3 (center)     -> GPIO 32  -- NOTE: change if GPIO32 used by ENB above,
                                   pick any free GPIO, e.g. GPIO 4
    S4 (right)      -> GPIO 39
    S5 (far right)  -> GPIO 36
  ------------------------------------------

  Logic: classic sensor-pattern lookup table (bang-bang control).
  No error math, no PID — just "if this sensor sees the line, do this."
*/

// ---------- Motor driver pins ----------
const int ENA = 14;   // Left motor speed (PWM)
const int IN1 = 27;
const int IN2 = 26;
const int IN3 = 25;
const int IN4 = 33;
const int ENB = 32;   // Right motor speed (PWM)

// ---------- IR sensor pins ----------
const int IR1 = 34;   // far left
const int IR2 = 35;   // left
const int IR3 = 4;    // center
const int IR4 = 39;   // right
const int IR5 = 36;   // far right

// ---------- Speeds ----------
const int BASE_SPEED = 150;   // 0-255
const int TURN_SPEED = 100;
const int SHARP_TURN_SPEED = 60;

// ESP32 core 3.x uses analogWrite() directly (0-255 like Arduino).
// If you're on core 2.x and analogWrite isn't available, swap to
// ledcSetup/ledcAttachPin/ledcWrite instead (see PID file comments).

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  Serial.begin(115200);
}

void loop() {
  int s1 = digitalRead(IR1);
  int s2 = digitalRead(IR2);
  int s3 = digitalRead(IR3);
  int s4 = digitalRead(IR4);
  int s5 = digitalRead(IR5);

  // Assumes HIGH = line detected (black line on white surface).
  // If your sensors are active-low, invert with (!digitalRead(pin)).

  if (s3 == HIGH && s1 == LOW && s2 == LOW && s4 == LOW && s5 == LOW) {
    // Dead center — go straight
    driveMotors(BASE_SPEED, BASE_SPEED);
  }
  else if (s2 == HIGH && s3 == LOW) {
    // Slightly left of line — nudge right
    driveMotors(TURN_SPEED, BASE_SPEED);
  }
  else if (s4 == HIGH && s3 == LOW) {
    // Slightly right of line — nudge left
    driveMotors(BASE_SPEED, TURN_SPEED);
  }
  else if (s1 == HIGH) {
    // Sharp left
    driveMotors(SHARP_TURN_SPEED, BASE_SPEED);
  }
  else if (s5 == HIGH) {
    // Sharp right
    driveMotors(BASE_SPEED, SHARP_TURN_SPEED);
  }
  else if (s1 == LOW && s2 == LOW && s3 == LOW && s4 == LOW && s5 == LOW) {
    // Line lost — stop (or replace with a search routine)
    driveMotors(0, 0);
  }
  else {
    // Fallback — straight
    driveMotors(BASE_SPEED, BASE_SPEED);
  }

  delay(10);
}

void driveMotors(int leftSpeed, int rightSpeed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, constrain(leftSpeed, 0, 255));

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, constrain(rightSpeed, 0, 255));
}

