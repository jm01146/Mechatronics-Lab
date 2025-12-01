#include <Servo.h>

Servo myservo;  // create Servo object to control a servo

void setup() {
  myservo.attach(9);  // attaches the servo on pin 9 to the Servo object
}

void loop() {
  myservo.write(0);                  // go to zero
  //myservo.write(180);                // go to 180
  delay(15);                           // waits for the servo to get there
}
