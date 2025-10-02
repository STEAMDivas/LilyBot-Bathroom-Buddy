
/* ===========================================================
   RobotDiva Bathroom Buddy — Voice + Line + Intelligent Avoid
   Flow:
     - "WAKE"(0): LED ON, Music ON, follow to bathroom pad (black square)
     - Wait at bathroom pad for "BED"(1)
     - On "BED": U-turn, follow back to home pad
     - Home pad: Music OFF, LED OFF, STOP
     - Emergency Stop on FREEZE(2)/PING(3)/CHECK(10)

   Intelligent Obstacle Avoidance (HC-SR04 D5/D6):
     - AVOID_INIT -> choose side away from current line edge
     - AVOID_ARC  -> forward curved drive; adapt curvature by distance
     - AVOID_RECOVER -> scan to re-acquire line if needed

   Sensors: HIGH on BLACK (sensor LEDs OFF on black)
   Gentle follow + in-place scan + curved avoidance

   Pins:
     TB6612: PWMA=11, AIN1=13, AIN2=12 (Left)
             PWMB=10, BIN1=8,  BIN2=9  (Right)
             STBY tied HIGH
     IR sensors: LEFT=A1, MID=A2, RIGHT=A3 (analog)
     LED: D4 (through 330Ω)
     Relay (music): A0  (assume active-LOW)
     Voice (Elechouse VR3): TX->D2, RX->D3 (SoftwareSerial)
     Sonar (HC-SR04): TRIG=D5, ECHO=D6
   =========================================================== */

#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

//////////////////// Declarations ////////////////////
// Speeds (0..255) — keep low with high power Power Supply
const int BASE_SPEED        = 85;
const int BIAS_SOFT         = 18;
const int SCAN_PIVOT_SPEED  = 70;
const int SCAN_SWITCH_MS    = 300;

// Sensor thresholding (HIGH on BLACK)
const int THRESHOLD         = 700;   // > THRESHOLD => on black (This is a measured threshold value)
const int HYST              = 30;

// Black marker detection (all three ON for this long)
const unsigned long MARKER_HOLD_MS = 300;

// U-turn parameters
const int  UTURN_PIVOT_SPEED       = 80;
const int  UTURN_DURATION_MS       = 800;

// LED & Relay
const int LED_PIN                  = 4;
const int RELAY_PIN                = A0;
const bool RELAY_ACTIVE_LOW        = true;   

// -------- Sonar (Obstacle Avoidance) --------
const int SONAR_TRIG_PIN           = 5;
const int SONAR_ECHO_PIN           = 6;
// Thresholds (cm)
const int OBST_CM_ENTER            = 18;     // start avoidance when <= this
const int OBST_CM_TIGHTEN          = 14;     // tighten arc when closer
const int OBST_CM_EMERGENCY        = 10;     // too close -> brief brake
// Arc parameters
const int ARC_OUTER_SPEED          = 90;     // outer wheel forward
const int ARC_INNER_SPEED_LOOSE    = 70;     // inner wheel forward (loose curve)
const int ARC_INNER_SPEED_TIGHT    = 50;     // tighter curve when close
const int ARC_INNER_SPEED_EMERG    = 0;      // nearly pivot if very close
const unsigned long AVOID_MAX_MS   = 1800;   // max time to spend in arc
const unsigned long AVOID_BRAKE_MS = 80;     // small brake when entering avoidance
// Poll sonar rate
const unsigned long SONAR_POLL_MS  = 60;
///////////////////////////////////////////////////////

///////////// MOTOR DRIVER ////////////////////////////
const int PWMA = 11; const int AIN1 = 13; const int AIN2 = 12; // Left
const int PWMB = 10; const int BIN1 = 8;  const int BIN2 = 9;  // Right

///////////// SENSORS /////////////////////////////////
const int S_LEFT  = A1;
const int S_MID   = A2;
const int S_RIGHT = A3;

///////////// VOICE (Elechouse VR3) ///////////////////
VR myVR(2,3);    // 2:RX 3:TX

const uint8_t REC_WAKE   = 0;
const uint8_t REC_BED    = 1;
const uint8_t REC_FREEZE = 2;
const uint8_t REC_PING   = 3;
const uint8_t REC_CHECK  = 10;
uint8_t vrBuf[64];

//////////////////// STATE ////////////////////////////
enum Mode {
  IDLE,
  OUT_FOLLOW, OUT_SCAN,
  WAIT_AT_BATH,
  RETURN_FOLLOW, RETURN_SCAN,
  // New avoidance states
  AVOID_INIT, AVOID_ARC, AVOID_RECOVER,
  STOPPED
};
Mode mode = IDLE;

bool L_on=false, M_on=false, R_on=false;
int  lastDirection = 0;   // -1 left, 0 center, +1 right
int  scanDir = -1;
unsigned long lastFlip = 0;
unsigned long allOnSince = 0;

unsigned long lastSonarPoll = 0;
long distanceCm = 999;

// Avoidance runtime
int avoidDir = +1; // -1 arc-left (left wheel slower), +1 arc-right
unsigned long avoidStartMs = 0;
///////////////////////////////////////////////////////

void setup() {
  pinMode(PWMA,OUTPUT); pinMode(AIN1,OUTPUT); pinMode(AIN2,OUTPUT);
  pinMode(PWMB,OUTPUT); pinMode(BIN1,OUTPUT); pinMode(BIN2,OUTPUT);

  pinMode(S_LEFT,INPUT); pinMode(S_MID,INPUT); pinMode(S_RIGHT,INPUT);
  pinMode(LED_PIN,OUTPUT); digitalWrite(LED_PIN,LOW);
  pinMode(RELAY_PIN,OUTPUT); musicOff();

  pinMode(SONAR_TRIG_PIN,OUTPUT);
  pinMode(SONAR_ECHO_PIN,INPUT);
  digitalWrite(SONAR_TRIG_PIN,LOW);

  Serial.begin(115200);
  delay(50);
  Serial.println(F("Bathroom Buddy — Intelligent Avoidance Build"));

  myVR.begin(9600);
  delay(100);
  myVR.clear();
  myVR.load(REC_WAKE);
  myVR.load(REC_BED);
  myVR.load(REC_FREEZE);
  myVR.load(REC_PING);
  myVR.load(REC_CHECK);

  setMotors(0,0);
}

void loop() {
  // 0) Voice emergency stop always wins
  int cmd = pollVoice(15);
  if (cmd == (int)REC_FREEZE || cmd == (int)REC_PING || cmd == (int)REC_CHECK) {
    emergencyStop();
    return;
  }

  // 1) Sensors
  int lRaw = analogRead(S_LEFT);
  int mRaw = analogRead(S_MID);
  int rRaw = analogRead(S_RIGHT);
  L_on = hysteresisUpdate(L_on, lRaw, THRESHOLD, HYST);
  M_on = hysteresisUpdate(M_on, mRaw, THRESHOLD, HYST);
  R_on = hysteresisUpdate(R_on, rRaw, THRESHOLD, HYST);

  // Marker
  bool allOn = (L_on && M_on && R_on);
  if (allOn) { if (allOnSince==0) allOnSince = millis(); }
  else allOnSince = 0;
  bool markerDetected = (allOnSince && (millis()-allOnSince >= MARKER_HOLD_MS));

  // 2) Sonar (rate-limited)
  if (millis()-lastSonarPoll >= SONAR_POLL_MS) {
    lastSonarPoll = millis();
    distanceCm = readDistanceCm();
  }
  bool obstacleClose = (distanceCm > 0 && distanceCm <= OBST_CM_ENTER);

  // 3) State machine
  switch (mode) {
    case IDLE: {
      setMotors(0,0);
      digitalWrite(LED_PIN, LOW);
      musicOff();
      if (cmd == (int)REC_WAKE) {
        digitalWrite(LED_PIN, HIGH);
        musicOn();
        mode = M_on ? OUT_FOLLOW : OUT_SCAN;
      }
      break;
    }

    case OUT_FOLLOW: {
      digitalWrite(LED_PIN, HIGH);
      if (markerDetected) { setMotors(0,0); mode = WAIT_AT_BATH; break; }
      if (obstacleClose)  { enterAvoidance(); break; }
      if (!M_on)          { mode = OUT_SCAN; break; }
      followBehavior();
      break;
    }

    case OUT_SCAN: {
      digitalWrite(LED_PIN, HIGH);
      if (markerDetected) { setMotors(0,0); mode = WAIT_AT_BATH; break; }
      if (obstacleClose)  { enterAvoidance(); break; }
      if (M_on)           { mode = OUT_FOLLOW; break; }
      scanBehavior();
      break;
    }

    case WAIT_AT_BATH: {
      setMotors(0,0);
      digitalWrite(LED_PIN, HIGH);
      if (cmd == (int)REC_BED) {
        doUTurn(UTURN_PIVOT_SPEED, UTURN_DURATION_MS);
        mode = RETURN_SCAN;
      }
      break;
    }

    case RETURN_FOLLOW: {
      digitalWrite(LED_PIN, HIGH);
      if (markerDetected) { setMotors(0,0); musicOff(); digitalWrite(LED_PIN,LOW); mode = STOPPED; break; }
      if (obstacleClose)  { enterAvoidance(); break; }
      if (!M_on)          { mode = RETURN_SCAN; break; }
      followBehavior();
      break;
    }

    case RETURN_SCAN: {
      digitalWrite(LED_PIN, HIGH);
      if (markerDetected) { setMotors(0,0); musicOff(); digitalWrite(LED_PIN,LOW); mode = STOPPED; break; }
      if (obstacleClose)  { enterAvoidance(); break; }
      if (M_on)           { mode = RETURN_FOLLOW; break; }
      scanBehavior();
      break;
    }

    // ---------- Intelligent Avoidance ----------
    case AVOID_INIT: {
      // Small brake for stability
      setMotors(0,0); delay(AVOID_BRAKE_MS);

      // Decide arc direction: away from current line edge if we see one
      if (L_on && !R_on)      avoidDir = +1; // arc right (right wheel outer)
      else if (R_on && !L_on) avoidDir = -1; // arc left
      else {
        // No strong edge — use opposite of lastDirection to bias toward open space
        avoidDir = (lastDirection <= 0) ? +1 : -1;
      }

      avoidStartMs = millis();
      mode = AVOID_ARC;
      break;
    }

    case AVOID_ARC: {
      // If we re-encounter the marker during avoidance, stop the trip appropriately
      if (markerDetected) {
        setMotors(0,0);
        // Decide which phase we were in to choose next mode:
        // If we were outbound, treat as bathroom; if returning, treat as home.
        // We detect by checking if we had passed WAIT_AT_BATH already.
        // Stop and go to AVOID_RECOVER and let normal logic decide after re-acquire.
        // Future Design improvement, use 2 different types of Pad that is distinguishable as Bath and Home pad
        mode = AVOID_RECOVER;
        break;
      }

      // Distance-based curvature
      int inner = ARC_INNER_SPEED_LOOSE;
      if (distanceCm > 0 && distanceCm <= OBST_CM_TIGHTEN) inner = ARC_INNER_SPEED_TIGHT;
      if (distanceCm > 0 && distanceCm <= OBST_CM_EMERGENCY) inner = ARC_INNER_SPEED_EMERG;

      int leftCmd, rightCmd;
      if (avoidDir < 0) {
        // arc left: left=inner, right=outer
        leftCmd  = inner;
        rightCmd = ARC_OUTER_SPEED;
      } else {
        // arc right: left=outer, right=inner
        leftCmd  = ARC_OUTER_SPEED;
        rightCmd = inner;
      }

      setMotors(leftCmd, rightCmd);

      // Exit conditions:
      // 1) Middle sees black (aligned again)
      if (M_on) {
        // small settle
        delay(60);
        mode = AVOID_RECOVER; // one pass of scan to cleanly re-center
        break;
      }
      // 2) Timeout — fall back to recover/scan
      if (millis() - avoidStartMs >= AVOID_MAX_MS) {
        setMotors(0,0);
        mode = AVOID_RECOVER;
        break;
      }
      // 3) If we get dangerously close, do a brief brake before continuing
      if (distanceCm > 0 && distanceCm <= OBST_CM_EMERGENCY) {
        setMotors(0,0); delay(100);
      }
      break;
    }

    case AVOID_RECOVER: {
      // Re-acquire and return to the appropriate follow/scan mode based on journey phase
      // Journey phase would be decided based on by looking at where we came from:
      // If we were outbound (OUT_*), go to OUT_SCAN; if returning (RETURN_*), go to RETURN_SCAN.

      bool outbound = isMusicOn();
      if (M_on) {
        mode = outbound ? OUT_FOLLOW : RETURN_FOLLOW;
      } else {
        mode = outbound ? OUT_SCAN : RETURN_SCAN;
      }
      break;
    }

    case STOPPED: {
      setMotors(0,0);
      break;
    }
  }

  delay(8);
}

/* ---------------- Voice ---------------- */
int pollVoice(uint16_t window_ms) {
  int ret = myVR.recognize(vrBuf, window_ms);
  if (ret > 0) {
    uint8_t rec = vrBuf[1];
    if (rec==REC_WAKE || rec==REC_BED || rec==REC_FREEZE || rec==REC_PING || rec==REC_CHECK) return (int)rec;
  }
  return -1;
}

/* ---------------- Follow / Scan ---------------- */
void followBehavior() {
  if (!L_on && M_on && !R_on) {
    setMotors(BASE_SPEED, BASE_SPEED);
    lastDirection = 0;
  }
  else if (L_on && !R_on) {
    setMotors(BASE_SPEED - BIAS_SOFT, BASE_SPEED + BIAS_SOFT);
    lastDirection = -1;
  }
  else if (R_on && !L_on) {
    setMotors(BASE_SPEED + BIAS_SOFT, BASE_SPEED - BIAS_SOFT);
    lastDirection = +1;
  }
  else {
    if (lastDirection < 0) {
      setMotors(BASE_SPEED - 10, BASE_SPEED + 10);
    } else if (lastDirection > 0) {
      setMotors(BASE_SPEED + 10, BASE_SPEED - 10);
    } else {
      setMotors(BASE_SPEED, BASE_SPEED);
    }
  }
}

void scanBehavior() {
  unsigned long now = millis();
  if (L_on && !R_on) scanDir = -1;
  else if (R_on && !L_on) scanDir = +1;
  else if (!L_on && !R_on) {
    if (lastDirection < 0) scanDir = -1;
    else if (lastDirection > 0) scanDir = +1;
  }
  if (now - lastFlip >= (unsigned long)SCAN_SWITCH_MS) {
    lastFlip = now; scanDir = -scanDir;
  }
  if (scanDir < 0) setMotors(-SCAN_PIVOT_SPEED, +SCAN_PIVOT_SPEED);
  else             setMotors(+SCAN_PIVOT_SPEED, -SCAN_PIVOT_SPEED);
}

/* ---------------- Avoidance Helpers ---------------- */
void enterAvoidance() {
  setMotors(0,0);
  mode = AVOID_INIT;
}

long readDistanceCm() {
  digitalWrite(SONAR_TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(SONAR_TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(SONAR_TRIG_PIN, LOW);
  unsigned long dur = pulseIn(SONAR_ECHO_PIN, HIGH, 25000UL);
  if (dur == 0) return -1;
  return (long)(dur / 58UL);
}

/* ---------------- Emergency Stop ---------------- */
void emergencyStop() {
  setMotors(0,0);
  musicOff();
  digitalWrite(LED_PIN, LOW);
  mode = STOPPED;
  for (int i=0;i<6;i++){ digitalWrite(LED_PIN,(i%2)); delay(120); }
  digitalWrite(LED_PIN, LOW);
}

/* ---------------- U-Turn ---------------- */
void doUTurn(int pivotSpeed, int durationMs) {
  int dir = (lastDirection >= 0) ? +1 : -1;
  int ls = (dir < 0) ? -pivotSpeed : +pivotSpeed;
  int rs = (dir < 0) ? +pivotSpeed : -pivotSpeed;
  setMotors(ls, rs);
  delay(durationMs);
  setMotors(0,0);
  delay(80);
  lastDirection = -lastDirection;
}

/* ---------------- Music / LED ---------------- */
void musicOn()  { relayWrite(true);  }
void musicOff() { relayWrite(false); }

void relayWrite(bool on) {
  if (RELAY_ACTIVE_LOW) digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  else                  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
}

bool isMusicOn() {
  // If active-LOW relay: LOW = ON; HIGH = OFF
  int pinState = digitalRead(RELAY_PIN);
  return RELAY_ACTIVE_LOW ? (pinState==LOW) : (pinState==HIGH);
}

/* ---------------- Hysteresis ---------------- */
bool hysteresisUpdate(bool currentOn, int val, int thr, int H) {
  int onLevel  = thr + (H >> 1);
  int offLevel = thr - (H >> 1);
  if (currentOn) {
    if (val < offLevel) return false;
    return true;
  } else {
    if (val > onLevel) return true;
    return false;
  }
}

/* ---------------- Motor Helpers ---------------- */
void setMotors(int leftSpeed, int rightSpeed) {
  setOneMotor(leftSpeed, AIN1, AIN2, PWMA);
  setOneMotor(rightSpeed, BIN1, BIN2, PWMB);
}
void setOneMotor(int speed, int IN1, int IN2, int PWM) {
  speed = constrain(speed, -255, 255);
  if (speed > 0) { digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW);  analogWrite(PWM, speed); }
  else if (speed < 0) { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); analogWrite(PWM, -speed); }
  else { digitalWrite(IN1,LOW); digitalWrite(IN2,LOW); analogWrite(PWM,0); }
}
