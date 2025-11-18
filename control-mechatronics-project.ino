// ===== 3-Finger Claw with L298N + Encoder + IR + Joystick =====
// Board: Arduino Uno / Nano
// Driver: L298N (Channel A: ENA, IN1, IN2)
// Motor: 12V DC gearmotor with encoder
// IR: object detection (auto close/open)
// Joystick: manual override control (drive the claw motor)

// ------------ PIN DEFINITIONS ------------

// IR sensor
const int IR_PIN = 4;           // Digital input from IR sensor

// L298N motor driver - Channel A
const int MOTOR_IN1 = 8;        // L298N IN1
const int MOTOR_IN2 = 9;        // L298N IN2
const int MOTOR_ENA = 10;       // L298N ENA (PWM)

// Encoder (single channel)
const int ENCODER_A = 2;        // Interrupt pin (INT0)

// Joystick
const int JOY_X_PIN = A0;       // Joystick X axis (optional)
const int JOY_Y_PIN = A1;       // Joystick Y axis (used for control)
const int JOY_SW_PIN = 5;       // Joystick button (optional)

// Joystick settings
const int JOY_CENTER     = 512;   // Analog center ~512
const int JOY_DEADZONE   = 80;    // +/- range around center considered "neutral"
const int JOY_MIN_SPEED  = 80;    // Minimum motor speed when outside deadzone
const int JOY_MAX_SPEED  = 255;   // Maximum motor speed

volatile long encoderCount = 0; // Updated in ISR

// ------------ CLAW POSITION SETTINGS ------------

// Tune these for your hardware:
const long CLAW_OPEN_POS   = 0;       // Open position (ticks)
const long CLAW_CLOSED_POS = 1200;    // Closed position (ticks) -> adjust
const long POSITION_TOL    = 10;      // Tolerance band (ticks)

// Auto mode speeds
const int CLOSE_SPEED = 200;
const int OPEN_SPEED  = 180;

// IR logic level (change to HIGH if your IR is opposite)
const int IR_TRIGGER_LEVEL = LOW;

// State tracking
bool clawIsClosed   = false;


void setup() {
  Serial.begin(9600);

  // IR input
  pinMode(IR_PIN, INPUT);

  // Motor driver pins
  pinMode(MOTOR_IN1, OUTPUT);
  pinMode(MOTOR_IN2, OUTPUT);
  pinMode(MOTOR_ENA, OUTPUT);

  // Encoder
  pinMode(ENCODER_A, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, RISING);

  // Joystick
  pinMode(JOY_SW_PIN, INPUT_PULLUP);  // Button uses internal pullup

  stopMotor();

  // Assume starting fully open at boot
  encoderCount = 0;
  clawIsClosed = false;

  Serial.println("Claw system with joystick ready. Starting at OPEN position.");
}

// ------------ ENCODER ISR ------------
void encoderISR() {
  encoderCount++;
}

// ------------ MOTOR HELPERS ------------
void setMotor(int speed) {
  if (speed > 0) {
    digitalWrite(MOTOR_IN1, HIGH);
    digitalWrite(MOTOR_IN2, LOW);
    analogWrite(MOTOR_ENA, speed);
  } else if (speed < 0) {
    digitalWrite(MOTOR_IN1, LOW);
    digitalWrite(MOTOR_IN2, HIGH);
    analogWrite(MOTOR_ENA, -speed);
  } else {
    stopMotor();
  }
}

void stopMotor() {
  digitalWrite(MOTOR_IN1, LOW);
  digitalWrite(MOTOR_IN2, LOW);
  analogWrite(MOTOR_ENA, 0);
}

// ------------ POSITION CONTROL (AUTO MODE) ------------
void moveToPosition(long targetPos, int baseSpeed) {
  long error = targetPos - encoderCount;

  while (abs(error) > POSITION_TOL) {
    error = targetPos - encoderCount;

    int direction = (error > 0) ? 1 : -1;

    long absError = abs(error);
    int speed = baseSpeed;

    // Slow down near target
    if (absError < 200) {
      speed = baseSpeed / 2;
      if (speed < 80) speed = 80;
    }

    setMotor(direction * speed);
    delay(5);
  }

  stopMotor();
}

// ------------ JOYSTICK PROCESSING ------------
bool handleJoystick() {
  int joyY = analogRead(JOY_Y_PIN);
  int joyX = analogRead(JOY_X_PIN);     // Not used yet, but read if you want to debug
  int joyBtn = digitalRead(JOY_SW_PIN); // LOW when pressed (with INPUT_PULLUP)

  int deltaY = joyY - JOY_CENTER;
  int absDeltaY = abs(deltaY);

  // Check if joystick is moved enough to count as manual control
  bool manualActive = (absDeltaY > JOY_DEADZONE);

  // You could also require button pressed to enable manual:
  // manualActive = manualActive && (joyBtn == LOW);

  if (manualActive) {
    // Map joystick deflection to motor speed
    int speed = map(absDeltaY,
                    JOY_DEADZONE, 512,
                    JOY_MIN_SPEED, JOY_MAX_SPEED);
    // Clamp speed
    if (speed > JOY_MAX_SPEED) speed = JOY_MAX_SPEED;

    // Direction: push up (lower analog value) vs down (higher)
    // Typical joystick: Up = smaller value, Down = larger
    int direction = (deltaY < 0) ? 1 : -1; // Up = close, Down = open

    setMotor(direction * speed);

    // Debug:
    // Serial.print("Manual Joystick: Y=");
    // Serial.print(joyY);
    // Serial.print(" dir=");
    // Serial.print(direction);
    // Serial.print(" speed=");
    // Serial.println(speed);

    return true;  // manual control in effect
  } else {
    // No manual input -> make sure motor isn't driven by joystick
    return false;
  }
}

// ------------ MAIN LOOP ------------
void loop() {
  // 1. Check joystick first: manual override
  bool manualControl = handleJoystick();

  if (manualControl) {
    // In manual mode: do NOT run IR auto logic, just let joystick drive the motor.
    // Encoder is still counting, useful if you want to sync positions later.
    delay(20);
    return;
  } else {
    // If joystick is neutral, stop motor from joystick side
    stopMotor();
  }

  // 2. Auto mode: IR + position control
  int irVal = digitalRead(IR_PIN);
  bool objectDetected = (irVal == IR_TRIGGER_LEVEL);

  // Serial.print("IR: "); Serial.print(irVal);
  // Serial.print(" | Enc: "); Serial.println(encoderCount);

  if (objectDetected && !clawIsClosed) {
    Serial.println("Auto mode: Object detected → closing claw...");
    moveToPosition(CLAW_CLOSED_POS, CLOSE_SPEED);
    clawIsClosed = true;
    Serial.print("Claw closed at encoder count: ");
    Serial.println(encoderCount);
  }
  else if (!objectDetected && clawIsClosed) {
    Serial.println("Auto mode: No object → opening claw...");
    moveToPosition(CLAW_OPEN_POS, OPEN_SPEED);
    clawIsClosed = false;
    Serial.print("Claw opened at encoder count: ");
    Serial.println(encoderCount);
  }

  delay(20);
}
