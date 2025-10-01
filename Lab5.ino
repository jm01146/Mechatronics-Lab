// Teensy simple analog counter with timing qualify
// Counts once per event when the signal rises above THRESHOLD_UP and stays there for QUALIFY_US.
// Won't re-arm until it falls below THRESHOLD_DOWN. Also has a small LOCKOUT_US.

const uint8_t SENSOR_PIN = A0;
const uint8_t LED_PIN    = LED_BUILTIN;

const int ADC_BITS = 12;       
const int THRESHOLD_UP = 850;     // set between your dark/lit levels
const int THRESHOLD_DOWN = 800;     // a little below THRESHOLD_UP
const uint32_t QUALIFY_US = 2000;     // must remain above THRESHOLD_UP this long to count
const uint32_t LOCKOUT_US = 2000;     // ignore new events within this time after a count

unsigned long count = 0;

enum State { BELOW, ABOVE_PENDING, ABOVE_CONFIRMED };
State state = BELOW;

uint32_t tRiseCandidateUs = 0;  // when we first saw "above"
uint32_t lastCountUs = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  analogReadResolution(ADC_BITS);
  analogReadAveraging(4);        // light smoothing

  Serial.begin(115200);
  while (!Serial && millis() < 2000) {}
  Serial.println("Analog counter with qualify + hysteresis");
}

void loop() {
  int val = analogRead(SENSOR_PIN);
  uint32_t now = micros();

  switch (state) {
    case BELOW:
      if (val > THRESHOLD_UP && (now - lastCountUs) >= LOCKOUT_US) {
        // potential rise detected; start qualify timer
        tRiseCandidateUs = now;
        state = ABOVE_PENDING;
      }
      break;

    case ABOVE_PENDING:
      if (val <= THRESHOLD_UP) {
        // dipped back below before qualify time—false alarm
        state = BELOW;
      } else if (now - tRiseCandidateUs >= QUALIFY_US) {
        // stayed high long enough: count it
        count++;
        lastCountUs = now;
        state = ABOVE_CONFIRMED;
        digitalWriteFast(LED_PIN, !digitalReadFast(LED_PIN));  // toggle LED on each count
        Serial.printf("Count = %lu\r\n", count);
      }
      break;

    case ABOVE_CONFIRMED:
      // wait until it clearly falls below the lower threshold to re-arm
      if (val <= THRESHOLD_DOWN) {
        state = BELOW;
      }
      break;
  }

  // light pacing
  delayMicroseconds(500);  // ~2 kHz sampling
}
