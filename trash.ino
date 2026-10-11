// Smart trash bin — MG996R lid (single file). Works with smart-bin.html
//
// Lid OPENS when a hand stays within handDistanceCm for 5 s or more
// (a quick pass of the hand does NOT open it).
//
// Lid closes ONLY when:
//   - a hand was detected and has been gone for CLOSE_DELAY_MS, or
//   - the "Close" button is clicked on the web page
//
// Sensor 1 (D4/D5) = HAND sensor, on the outside/front of the bin
// Sensor 2 (D2/D3) = FILL sensor, at the rim, pointing down into the bin
// MG996R servo on D9 = lid
//
// Sends to PC:   D,<hand_cm>,<fill_cm>,<lid 0/1>     (-1 = out of range)
//                # ...   debug lines (shown in "Raw serial data" on the web page)
// Commands:      O        open lid (stays open until C)
//                C        close lid now
//                H:<cm>   set hand detection distance

#include <Servo.h>

// ================= PINS =================
const int HAND_TRIG = 4, HAND_ECHO = 5;
const int FILL_TRIG = 2, FILL_ECHO = 3;
const int LID_SERVO_PIN = 9;

// ================ AUTO-OPEN ================
float handDistanceCm = 20.0;                // open when hand is closer than this (web page can change it)
const unsigned long HOLD_TO_OPEN_MS = 2500; // hand must stay this long before the lid opens
const unsigned long DROPOUT_MS = 300;       // ignore sensor misses shorter than this while holding
const unsigned long CLOSE_DELAY_MS = 2000;   // close 2s s after the hand leaves

// ================ SERVO TYPE ================
// false = POSITIONAL servo: goes to an angle and holds it.
//         (If your lid opened and then went straight back, you have this type.)
// true  = CONTINUOUS rotation servo: spins, stops at STOP_US.
const bool CONTINUOUS_SERVO = false;

// --- Positional servo settings ---
const int CLOSED_ANGLE = 0;     // lid closed
const int OPEN_ANGLE   = 120;    // lid open. For a 360° positional servo, 45 here ≈ 90° real
const unsigned long MOVE_TIME_MS = 700;   // time to let the servo reach its angle

// --- Continuous servo settings (only used if CONTINUOUS_SERVO = true) ---
const int STOP_US        = 1500;  // adjust 1480–1520 if motor creeps
const int SPEED_US       = 200;   // 0–500, higher = faster
const int OPEN_DIRECTION = 1;     // -1 if lid turns the wrong way
const unsigned long OPEN_TIME_MS  = 1200;  // time to turn 90° open
const unsigned long CLOSE_TIME_MS = 1200;  // time to turn 90° back

const unsigned long SETTLE_TIME = 1000;   // ms after closing before measuring fill

// ================ STATE ================
enum LidState { CLOSED, OPENING, OPEN, CLOSING };
LidState lidState = CLOSED;
const char* STATE_NAMES[] = { "CLOSED", "OPENING", "OPEN", "CLOSING" };

Servo lidServo;
unsigned long moveStart = 0;
unsigned long moveDuration = 0;
unsigned long lastHandSeen = 0;
bool handSeen = false;
bool manualOpen = false;          // opened by web button, no timeout
unsigned long closedAt = 0;

bool handPresent = false;         // hand currently being held near the sensor
unsigned long handStart = 0;      // when the current hold began
unsigned long lastNear = 0;       // last reading within range
bool holdReported = false;

float lastHand = -1;
float fillCm = -1;
String cmd = "";

void setState(LidState s) {
  lidState = s;
  Serial.print("# lid ");
  Serial.println(STATE_NAMES[s]);
}

// ================ MOTOR ================
void motorStop() {
  if (CONTINUOUS_SERVO) lidServo.writeMicroseconds(STOP_US);
  // positional servo: do nothing, it holds its angle
}
void motorOpen() {
  if (CONTINUOUS_SERVO) lidServo.writeMicroseconds(STOP_US + OPEN_DIRECTION * SPEED_US);
  else lidServo.write(OPEN_ANGLE);
}
void motorClose() {
  if (CONTINUOUS_SERVO) lidServo.writeMicroseconds(STOP_US - OPEN_DIRECTION * SPEED_US);
  else lidServo.write(CLOSED_ANGLE);
}

unsigned long openTime()  { return CONTINUOUS_SERVO ? OPEN_TIME_MS  : MOVE_TIME_MS; }
unsigned long closeTime() { return CONTINUOUS_SERVO ? CLOSE_TIME_MS : MOVE_TIME_MS; }

void startOpening(unsigned long duration) {
  motorOpen();
  moveStart = millis();
  moveDuration = duration;
  setState(OPENING);
}

void startClosing() {
  motorClose();
  moveStart = millis();
  moveDuration = closeTime();
  setState(CLOSING);
}

// Should the lid be open right now?
bool wantOpen() {
  bool handRecently = handSeen && (millis() - lastHandSeen < CLOSE_DELAY_MS);
  return handRecently || manualOpen;
}

// Lid cycle: CLOSED -> OPENING -> OPEN -> CLOSING -> CLOSED
void updateLid() {
  unsigned long elapsed = millis() - moveStart;

  switch (lidState) {
    case CLOSED:
      if (wantOpen()) startOpening(openTime());
      break;

    case OPENING:
      if (elapsed >= moveDuration) { motorStop(); setState(OPEN); }
      break;

    case OPEN:
      if (!wantOpen()) startClosing();
      break;

    case CLOSING:
      // Always finish closing (no reopen while closing)
      if (elapsed >= moveDuration) {
        motorStop();
        closedAt = millis();
        handStart = millis();      // a hand still there must hold a fresh 5 s to reopen
        holdReported = false;
        setState(CLOSED);
      }
      break;
  }
}

// ================ SENSORS ================
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 25000);  // ~4 m max, keeps loop fast
  if (duration == 0) return -1;
  return duration * 0.0343 / 2;
}

// ================ SERIAL ================
void handleCommand(String c) {
  c.trim();
  if (c == "O") {
    manualOpen = true;
    Serial.println("# button OPEN");
  } else if (c == "C") {
    manualOpen = false;
    handSeen = false;
    Serial.println("# button CLOSE");
  } else if (c.startsWith("H:")) {
    float v = c.substring(2).toFloat();
    if (v >= 3 && v <= 100) handDistanceCm = v;
  }
}

void readSerial() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n') { handleCommand(cmd); cmd = ""; }
    else if (ch != '\r' && cmd.length() < 20) cmd += ch;
  }
}

void waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateLid();
    readSerial();
  }
}

// ================ MAIN ================
void setup() {
  Serial.begin(9600);
  pinMode(HAND_TRIG, OUTPUT); pinMode(HAND_ECHO, INPUT);
  pinMode(FILL_TRIG, OUTPUT); pinMode(FILL_ECHO, INPUT);

  lidServo.attach(LID_SERVO_PIN);
  if (CONTINUOUS_SERVO) motorStop();   // lid must be closed at power on
  else lidServo.write(CLOSED_ANGLE);   // move to closed angle
  lidState = CLOSED;
  closedAt = millis();
}

void loop() {
  readSerial();

  // 1. Hand sensor
  lastHand = readDistance(HAND_TRIG, HAND_ECHO);
  unsigned long now = millis();
  bool near = lastHand > 0 && lastHand <= handDistanceCm;

  // Track how long the hand has stayed near (short sensor misses are ignored)
  if (near) {
    if (!handPresent) {
      handPresent = true;
      handStart = now;
      holdReported = false;
      if (lidState == CLOSED) Serial.println("# hand detected, hold 5 s to open");
    }
    lastNear = now;
  } else if (handPresent && now - lastNear > DROPOUT_MS) {
    handPresent = false;
    if (lidState == CLOSED && !holdReported) Serial.println("# hand left before 5 s, not opening");
  }

  bool held = handPresent && now - handStart >= HOLD_TO_OPEN_MS;

  if (lidState == CLOSED) {
    // Closed: open only after the hand has been held for 5 s
    if (held) {
      if (!holdReported) { Serial.println("# hand held 5 s, opening"); holdReported = true; }
      handSeen = true;
      lastHandSeen = now;
      manualOpen = false;
    }
  } else if (near && lidState != CLOSING) {
    // Opening or open: any hand keeps it open; the close countdown restarts
    handSeen = true;
    lastHandSeen = now;
    manualOpen = false;        // from now on, closes CLOSE_DELAY_MS after the hand leaves
  }

  // 2. Lid
  updateLid();

  waitMs(40);  // avoid crosstalk between sensors

  // 3. Fill level — only when lid is fully closed and settled
  if (lidState == CLOSED && millis() - closedAt > SETTLE_TIME) {
    fillCm = readDistance(FILL_TRIG, FILL_ECHO);
  }
  updateLid();

  // 4. Report to web page
  Serial.print("D,");
  Serial.print(lastHand); Serial.print(",");
  Serial.print(fillCm);   Serial.print(",");
  Serial.println(lidState == CLOSED ? 0 : 1);

  waitMs(60);
}
