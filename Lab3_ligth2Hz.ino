#include <Arduino.h>
#include <Servo.h>

int ledPin = 13;

void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(ledPin, HIGH); // Turn LED on
  delay(250);                 // Wait 250 milliseconds (ON duration)

  digitalWrite(ledPin, LOW);  // Turn LED off
  delay(250);                 // Wait 250 milliseconds (OFF duration)
}
