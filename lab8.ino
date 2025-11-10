//PART 1
#define LED1_PIN    2
#define LED2_PIN    3
#define SWITCH2_PIN 4
#define SWITCH3_PIN 5

void setup() {
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(SWITCH2_PIN, INPUT_PULLUP);
  pinMode(SWITCH3_PIN, INPUT_PULLUP);

  // Start with both off
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);
}

void loop() {
  // Buttons are active-LOW with INPUT_PULLUP
  if (digitalRead(SWITCH2_PIN) == LOW) {
    digitalWrite(LED1_PIN, HIGH);
    digitalWrite(LED2_PIN, LOW);
  }

  if (digitalRead(SWITCH3_PIN) == LOW) {
    digitalWrite(LED1_PIN, LOW);
    digitalWrite(LED2_PIN, HIGH);
  }
}



// PART 2
#define BTN_A 2
#define BTN_B 3

volatile uint32_t tA_us = 0, tB_us = 0;
volatile bool first_latched = false;
volatile uint32_t lastA_us = 0, lastB_us = 0;   // debounce guards
const uint32_t DEBOUNCE_US = 3000;              // ~3 ms debounce

float fastest_ms = 1e9; // silly large

void IRAM_ATTR isrA() {
  uint32_t now = micros();
  if (now - lastA_us < DEBOUNCE_US) return;
  lastA_us = now;

  tA_us = now;
  first_latched = true;
}

void IRAM_ATTR isrB() {
  uint32_t now = micros();
  if (now - lastB_us < DEBOUNCE_US) return;
  lastB_us = now;

  if (first_latched) {
    tB_us = now;
    // do the light work here; heavy prints in loop
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(BTN_A, INPUT_PULLUP);
  pinMode(BTN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_A), isrA, FALLING);
  attachInterrupt(digitalPinToInterrupt(BTN_B), isrB, FALLING);

  Serial.println("Part 2: press A then B (two motions of one finger).");
}

void loop() {
  static uint32_t last_report = 0;

  noInterrupts();
  bool have_pair = first_latched && tB_us >= tA_us && (tB_us - tA_us) < 5UL * 1000000UL;
  uint32_t ta = tA_us, tb = tB_us;
  if (have_pair) {
    // consume this pair
    first_latched = false;
    tA_us = 0; tB_us = 0;
  }
  interrupts();

  if (have_pair) {
    float delta_ms = (tb - ta) / 1000.0f;
    if (delta_ms < fastest_ms) fastest_ms = delta_ms;

    Serial.print("Delta: ");
    Serial.print(delta_ms, 3);
    Serial.print(" ms   | Fastest: ");
    Serial.print(fastest_ms, 3);
    Serial.println(" ms");
  }

  // gentle heartbeat print every ~2s
  if (millis() - last_report > 2000) {
    last_report = millis();
    Serial.println("(waiting for A then B...)");
  }
}

// PART 3
#include <IntervalTimer.h>

#define PWM_PIN 10
#define POT_PIN A0

// Set your PWM base frequency here (Hz)
const float PWM_HZ = 1000.0f; // 1 kHz
// Computed total period (us)
const uint32_t PERIOD_US = (uint32_t)(1000000.0f / PWM_HZ);

IntervalTimer pwmTimer;

volatile bool outHigh = false;
volatile bool useFirstDuty = true; // alternate dc1/dc2 each cycle
volatile uint32_t nextIntervalUs = 100; // initialized later

// Shared (updated in loop)
volatile float dc1 = 0.30f;  // 30% default
volatile float dc2 = 0.70f;  // 70% default

void scheduleNext(bool goingHigh) {
  // When going HIGH, we need a high-time; when going LOW, we need a low-time.
  // The "period-2" alternation means: the high-time alternates between dc1*P and dc2*P,
  // and the low-time is (P - that high-time).
  float duty = useFirstDuty ? dc1 : dc2;
  uint32_t tHigh = (uint32_t)(PERIOD_US * duty);
  if (tHigh < 2) tHigh = 2;                       // guard against 0
  if (tHigh > PERIOD_US - 2) tHigh = PERIOD_US-2; // guard against 100%

  uint32_t tLow = PERIOD_US - tHigh;

  if (goingHigh) nextIntervalUs = tHigh;
  else           nextIntervalUs = tLow;
}

void pwmISR() {
  // Toggle output
  outHigh = !outHigh;
  digitalWriteFast(PWM_PIN, outHigh ? HIGH : LOW);

  // If we just went LOW, we completed a full "cycle"—alternate duty for next one
  if (!outHigh) {
    useFirstDuty = !useFirstDuty;
  }

  // Schedule next edge based on current phase
  scheduleNext(outHigh);
  pwmTimer.update(nextIntervalUs);
}

void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWriteFast(PWM_PIN, LOW);

  // Prime first interval using current dc1
  scheduleNext(true); // first edge will be HIGH for dc1*P
  pwmTimer.begin(pwmISR, nextIntervalUs);

  Serial.begin(115200);
  analogReadResolution(10); // 0..1023
  Serial.println("Part 3: Alternating-duty PWM. Turn the pot to change base duty (0..50%).");
}

void loop() {
  // Base x% from pot in [0, 50%]
  int raw = analogRead(POT_PIN);           // 0..1023
  float x = (raw / 1023.0f) * 0.50f;       // 0..0.50
  float new_dc1 = x;
  float new_dc2 = 1.0f - x;

  // Update atomically
  noInterrupts();
  dc1 = new_dc1;
  dc2 = new_dc2;
  interrupts();

  static uint32_t t0 = 0;
  if (millis() - t0 > 500) {
    t0 = millis();
    Serial.print("x = ");
    Serial.print(100.0f*new_dc1, 1);
    Serial.print("% | alternating with ");
    Serial.print(100.0f*new_dc2, 1);
    Serial.println("%");
  }
}


// PART 4
#define ENC_A 2
#define ENC_B 3

// --- Optional LCD ---
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>
// LiquidCrystal_I2C lcd(0x27, 16, 2);  // change address if needed
// bool lcd_ok = false;

volatile int32_t position = 0;
// Track last 2-bit state (A<<1 | B)
volatile uint8_t lastAB = 0;

// Transition table for quadrature (returns -1, 0, +1 per state change)
// Index: (prev<<2) | curr
// Valid forward steps: 01->11->10->00->01  (and reverse opposite)
const int8_t qtab[16] = {
  0,  -1,  +1,   0,
  +1,  0,   0,  -1,
  -1,  0,   0,  +1,
   0,  +1, -1,   0
};

inline uint8_t readAB() {
  uint8_t a = digitalReadFast(ENC_A);
  uint8_t b = digitalReadFast(ENC_B);
  return (a << 1) | b;
}

void handleQuad() {
  uint8_t curr = readAB();
  uint8_t idx = (lastAB << 2) | curr;
  int8_t step = qtab[idx];
  if (step != 0) position += step;
  lastAB = curr;
}

void IRAM_ATTR isrA() { handleQuad(); }
void IRAM_ATTR isrB() { handleQuad(); }

void setup() {
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  lastAB = readAB();

  attachInterrupt(digitalPinToInterrupt(ENC_A), isrA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENC_B), isrB, CHANGE);

  Serial.begin(115200);
  Serial.println("Part 4: Quadrature position on Serial (optional LCD code inside).");

  // --- Optional LCD init ---
  // Wire.begin();
  // if (lcd.begin()) {
  //   lcd.backlight();
  //   lcd.clear();
  //   lcd.print("Quat Decoder");
  //   lcd.setCursor(0,1);
  //   lcd.print("Pos: 0");
  //   lcd_ok = true;
  // }
}

void loop() {
  static int32_t last_shown = INT32_MIN;
  noInterrupts();
  int32_t pos = position;
  interrupts();

  if (pos != last_shown) {
    last_shown = pos;
    Serial.print("Position: ");
    Serial.println(pos);

    // --- Optional LCD update ---
    // if (lcd_ok) {
    //   lcd.clear();
    //   lcd.setCursor(0,0); lcd.print("Quadrature Cnt");
    //   lcd.setCursor(0,1); lcd.print("Pos: "); lcd.print(pos);
    // }
  }
}
