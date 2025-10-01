#include <Arduino.h>

// Teensy edge counter for IR phototransistor output
// Counts each time light turns ON (LOW -> HIGH)

const uint8_t SENSOR_PIN = 2;        // any interrupt-capable digital pin
const uint8_t LED_PIN = LED_BUILTIN;

volatile unsigned long hitCount = 0;
volatile uint32_t lastEdgeUs = 0;
const uint32_t LOCKOUT_US = 1000;    // 1 ms min edge spacing (tune to your signal)

void onLightRise() {
  uint32_t now = micros();
  if (now - lastEdgeUs >= LOCKOUT_US) { // simple deglitch
    hitCount++;
    lastEdgeUs = now;
    digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN)); // blink/toggle on each hit (optional)
    }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // If your sensor has an open-collector output, make sure there's a pull-up on the line.
  // Teensy 4.x supports internal pulldown too, but you likely want a clean external pull-up in hardware.
  pinMode(SENSOR_PIN, INPUT); // or INPUT_PULLDOWN / INPUT_PULLUP depending on your wiring

  attachInterrupt(digitalPinToInterrupt(SENSOR_PIN), onLightRise, RISING);

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {} // brief wait for USB
  Serial.println("IR light ON counter ready.");
}

void loop() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 500) {
    noInterrupts();
    unsigned long c = hitCount;
    interrupts();
    Serial.printf("Count = %lu\r\n", c);
    lastPrint = millis();
  }
}

