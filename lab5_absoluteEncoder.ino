// - Absolute 2-bit  Encoder
const uint8_t PIN_A = 2;               // interrupt-capable pin
const uint8_t PIN_B = 3;               // interrupt-capable pin
const uint32_t QUALIFY_US = 2000;      // must be stable this long to accept a change (2 ms)

const bool INVERT_A = false;          
const bool INVERT_B = false;

// Orientation names in clockwise order starting at index 0.
const char* ORIENT[4] = {"UP", "RIGHT", "DOWN", "LEFT"};
const uint8_t ZERO_OFFSET = 0;         // 0..3 rotate mapping to match your wheel

// ----- internals -----
volatile uint8_t lastRaw = 0;
volatile uint8_t pendingRaw = 0;
volatile bool    pending = false;
volatile uint32_t tPendingUs = 0;

uint8_t currentIdx = 255;              // unknown at start

static inline uint8_t rdAB() {
  int a = digitalReadFast(PIN_A);
  int b = digitalReadFast(PIN_B);
  if (INVERT_A) a = !a;
  if (INVERT_B) b = !b;
  return (a ? 1 : 0) | (b ? 2 : 0);    // bit0=A, bit1=B  -> 00,01,10,11
}

// Convert 2-bit (00,01,11,10) to index 0..3
static inline uint8_t gray2idx(uint8_t g) {
  // reorder to classic Gray: bit1..bit0
  uint8_t G = ((g & 2) >> 1) << 1 | (g & 1); // [B A]
  uint8_t b = G ^ (G >> 1);                  // Gray->binary
  return b & 3;                              // 0..3
}

void onEdge() {
  uint8_t r = rdAB();
  if (r == lastRaw) return;
  lastRaw = r;
  pendingRaw = r;
  tPendingUs = micros();
  pending = true;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 1500) {}

  pinMode(PIN_A, INPUT_PULLUP);
  pinMode(PIN_B, INPUT_PULLUP);

  // Prime state
  lastRaw = rdAB();

  attachInterrupt(digitalPinToInterrupt(PIN_A), onEdge, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_B), onEdge, CHANGE);

  // Announce initial orientation after a brief settle
  delay(5);
  uint8_t idx = (gray2idx(rdAB()) + ZERO_OFFSET) & 3;
  currentIdx = idx;
  Serial.printf("Orientation: %s\n", ORIENT[currentIdx]);
}

void loop() {
  if (pending) {
    uint32_t now = micros();
    if ((now - tPendingUs) >= QUALIFY_US) {
      uint8_t nowRaw = rdAB();
      if (nowRaw == pendingRaw) {
        uint8_t idx = (gray2idx(nowRaw) + ZERO_OFFSET) & 3;
        if (idx != currentIdx) {
          currentIdx = idx;
          Serial.printf("Orientation: %s\n", ORIENT[currentIdx]);
        }
      }
      pending = false;
    }
  }
}
