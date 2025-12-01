#include <Servo.h>

const int SERVO_PIN = 9;

const int NUM_SENSORS = 4;
const int fsrPins[NUM_SENSORS] = {A0, A1, A2, A3};  // FSR inputs

// These are analog values (0–1023 on 5V Arduino)
const int TARGET_LOW  = 300;   // lower bound of desired force range
const int TARGET_HIGH = 700;   // upper bound of desired force range
const int DEADBAND    = 20;    // small band to avoid jitter around thresholds

const int MIN_ANGLE   = 0;     // hard limit for servo
const int MAX_ANGLE   = 180;   // hard limit for servo
const int START_ANGLE = 45;    // starting at 45 degrees
const int STEP_ANGLE  = 2;     // how many degrees to move per correction step

Servo myServo;

void setup() {
  Serial.begin(9600);

  myServo.attach(SERVO_PIN);
  myServo.write(START_ANGLE);  // start at45
  delay(500);
}

int readAverageFSR() {
  long sum = 0;
  for (int i = 0; i < NUM_SENSORS; i++) {
    int reading = analogRead(fsrPins[i]);
    sum += reading;
  }
  int avg = sum / NUM_SENSORS;
  return avg;
}

void loop() {
  int avgFSR = readAverageFSR();
  int currentAngle = myServo.read();  // current servo angle

  // Print for debugging
  Serial.print("Avg FSR: ");
  Serial.print(avgFSR);
  Serial.print(" | Angle: ");
  Serial.println(currentAngle);

  // Control logic:
  // If pressure is too low -> move one direction
  // If pressure is too high -> move opposite direction
  // If inside range -> do nothing (or very small changes if you want)
  if (avgFSR < (TARGET_LOW - DEADBAND)) {
    // Too little force -> move servo to try to increase it
    currentAngle -= STEP_ANGLE;
  } else if (avgFSR > (TARGET_HIGH + DEADBAND)) {
    // Too much force -> move servo to try to decrease it
    currentAngle += STEP_ANGLE;
  } else {
    // Within desired range; keep angle
    // (You could also add tiny corrections here if you want)
  }

  // Clamp angle to safe servo range
  currentAngle = constrain(currentAngle, MIN_ANGLE, MAX_ANGLE);

  myServo.write(currentAngle);

  delay(50);
}
