#include <Arduino.h>
#include <string.h>


//==================== 1) PIN MAP ====================
// MUX
#define MUX_SIG 34
#define S0 16
#define S1 17
#define S2 18
#define S3 19

// TB6612FNG
// Motor A (RIGHT)
#define AIN1 27
#define AIN2 14
#define PWMA 25
// Motor B (LEFT)
#define BIN1 26
#define BIN2 33
#define PWMB 32
// Standby
#define STBY 4
// LED
#define LED 2

//==================== 2) USER TUNING ====================
#define PWM_BITS 10
#define PWM_MAX 80 //105
#define PWM_MAX_F 80 //105
#define PWM_MAX_P 110 //105
#define PWM_FREQ 20000  // ~20kHz

// --- Speed / Mixing (ตามที่ผู้ใช้ระบุ) ---
int BASE_SPEED = 50;    // 0..1023 (สำหรับ 10-bit)
int MAX_DELTA = 100;    // PID correction mix
int BOOST_SPEED = 170;  // ไม่ใช้แล้ว (เก็บไว้เผื่อกลับไปโหมดเดิม)
int REVERSE_DUTY = 80;  // ไม่ใช้แล้ว (เก็บไว้เผื่อกลับไปโหมดเดิม)

// --- Trigger สำหรับเลี้ยวแบบถอยล้อด้านใน ---
float EDGE_TRIG = 0.30f;  // |err| >= นี้ -> เข้าโหมดถอยล้อด้านใน 0.7f
// ใช้ตัวแปรเดียวกันเป็นเกณฑ์ (TURN_TRIG)
float TURN_TRIG = EDGE_TRIG;

// --- กำลังของโหมดถอยล้อด้านใน ---
int TURN_FWD = 260;  // was 200 ล้อด้านนอกวิ่งหน้าแรง
int TURN_REV = 160;  // was 100 ล้อด้านในถอยหลัง

// PID gains (เริ่มต้น)
float Kp = 1.4f;
float Ki = 0.02f;
float Kd = 3.00f;

// MUX read settings
const uint8_t USE_FROM = 2;  // was ใช้ CH2..CH13 เท่านั้น
const uint8_t USE_TO = 13;
const uint8_t N_USED = (USE_TO - USE_FROM + 1);
const int MUX_SETTLE_US = 5;  //40;  // was 6, give time to settle
const int ADC_SAMPLES = 3;

//==================== 3) GLOBALS ====================
uint16_t adcRaw[16];
float lastError = 0.0f;
float integral = 0.0f;
uint32_t tLastPID = 0;
uint32_t tStart = 0;

// --- add globals (top of file near other globals) ---
uint8_t binUsed[N_USED];  // 0/1 per sensor within CH2..CH13
float lastThresh = 0;     // for debugging/analysis
// Fixed thresholds with hysteresis (tune around your 500 baseline)
const int TH_HI = 1640;     // must rise above this to turn ON
const int TH_LO = 1560;     // must drop below this to turn OFF
uint8_t binState[N_USED];  // latched on/off per used sensor (init to 0)

// ---hair pin add globals (top of file) ---
enum Mode { FOLLOW,
            PIVOT };
Mode mode = FOLLOW;
int pivotDir = +1;  // +1 = pivot right, −1 = pivot left
uint32_t tLost = 0;

int left = 0;
int right = 0;

// --- Tuning constants (near your other constants) ---
const uint32_t LOST_MS = 5
;   // was 40 → pivot earlier
const float ERR_HARD = 0.02f;  // was 0.1  // smaller mean pivot easier
int PIVOT_SPEED = 100;  // was ~65 → spin faster 

// ---- EXIT conditions tuned for hairpin ----
const float ERR_RECENTER = 0.14f;   // keep or tweak slightly
const uint8_t K_HITS = 2;           // already defined above; keep your value or set 3
const uint8_t CENTER_MIN = 2;       // need at least N center sensors ON
const uint8_t EXIT_HOLD = 2;        // need condition for N consecutive loops
static uint8_t recenterStreak = 0;  // hysteresis counter

// ---- search memory ----
int8_t lastSeenSide = 0;  // +1 = line to LEFT, -1 = RIGHT, 0 = unknown
uint32_t tPivotExit = 0;
uint32_t tPivotEnter = 0;
uint32_t tFollowEnter = 0;
const uint32_t PIVOT_COOLDOWN_MS = 80;  // prevent immediate opposite re-trigger

bool invertIR = true;             // false: ค่าสูง=เส้น (ค่าเดิม), true: ค่าต่ำ=เส้น (หลัง flip)



//==================== 4) UTILITIES ====================
void muxSelect(uint8_t ch) {
  digitalWrite(S0, (ch >> 0) & 1);
  digitalWrite(S1, (ch >> 1) & 1);
  digitalWrite(S2, (ch >> 2) & 1);
  digitalWrite(S3, (ch >> 3) & 1);
}


uint16_t readMuxChannel(uint8_t ch) {
  muxSelect(ch);
  delayMicroseconds(MUX_SETTLE_US);
  (void)analogRead(MUX_SIG);  // throwaway sample after switch
  uint32_t acc = 0;
  for (int i = 0; i < ADC_SAMPLES; ++i) acc += analogRead(MUX_SIG);
  return (uint16_t)(acc / ADC_SAMPLES);
}

void readAllChannels() {
  for (uint8_t ch = 0; ch < 16; ++ch) {
    adcRaw[ch] = readMuxChannel(ch);
  }
}

// Center window ~ indices 4..7 (adjust if your array geometry differs)
uint8_t countCenterOn() {
  uint8_t c = 0;
  for (int i = 4; i <= 7; ++i) {
    if (binUsed[i]) c++;
  }
  return c;
}

// --- SHORT BRAKE helpers (TB6612FNG truth table) ---
void rightShortBrake() {
  digitalWrite(AIN1, HIGH);
  digitalWrite(AIN2, HIGH);
}
void leftShortBrake() {
  digitalWrite(BIN1, HIGH);
  digitalWrite(BIN2, HIGH);
}
void bothShortBrake(uint16_t us) {
  rightShortBrake();
  leftShortBrake();
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);
  delayMicroseconds(us);
}

// มอเตอร์รับค่า signed: + เดินหน้า, - ถอยหลัง, 0 หยุด
void motorRightSigned(int duty) {  // Motor A
  int a = abs(duty);
  // a = constrain(a, 0, PWM_MAX);
  if (mode == FOLLOW){
    a = constrain(a, 0, PWM_MAX_F);
  } else{
    a = constrain(a, 0, PWM_MAX_P);
  }
  if (duty > 0) {  // forward
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, a);
  } else if (duty < 0) {  // reverse
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, a);
  } else {
    analogWrite(PWMA, 0);
  }
}

void motorLeftSigned(int duty) {  // Motor B
  int a = abs(duty);
  // a = constrain(a, 0, PWM_MAX);
  if (mode == FOLLOW){
    a = constrain(a, 0, PWM_MAX_F);
  } else{
    a = constrain(a, 0, PWM_MAX_P);
  }
  if (duty > 0) {  // forward
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, a);
  } else if (duty < 0) {  // reverse
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, a);
  } else {
    analogWrite(PWMB, 0);
  }
}

void driveLRsigned(int left, int right) {
  motorLeftSigned(left);
  motorRightSigned(right);
}

void stopAll() {
  motorLeftSigned(0);
  motorRightSigned(0);
}

//==================== 5) ERROR COMPUTATION ====================
// Returns: err in [-1..+1]; fills binUsed[] and hitsOut (count of ON sensors)
float computeError(bool &sawLine, int &hitsOut) {
  // --- smooth centroid accumulators ---
  double wsum = 0.0, vsum = 0.0;
  int hits = 0;

  // --- iterate used channels (CH2..CH13 => idx 0..11) ---
  for (uint8_t ch = USE_FROM; ch <= USE_TO; ++ch) {
    int idx = ch - USE_FROM;
    float v = (float)adcRaw[ch];

// --- 1) hysteresis-latched ON/OFF for hairpin logic (binUsed) ---
uint8_t on = binState[idx];
if (!invertIR) {
  // โหมดเดิม: ค่าสูง = เส้น
  if (!on && v >= TH_HI) on = 1;
  else if (on && v <= TH_LO) on = 0;
} else {
  // โหมดกลับขั้ว: ค่าต่ำ = เส้น
  if (!on && v <= TH_LO) on = 1;
  else if (on && v >= TH_HI) on = 0;
}
binState[idx] = on;
binUsed[idx] = on;
if (on) hits++;

// --- 2) smooth weight for centroid ---
float w = 0.0f;
if (!invertIR) {
  // เดิม: สูง→1, ต่ำ→0
  if (v > TH_LO) {
    w = (v - TH_LO) / (float)(TH_HI - TH_LO + 1e-6f);
    if (w < 0) w = 0; if (w > 1) w = 1;
  }
} else {
  // กลับขั้ว: ต่ำ→1, สูง→0
  if (v < TH_HI) {
    w = (TH_HI - v) / (float)(TH_HI - TH_LO + 1e-6f);
    if (w < 0) w = 0; if (w > 1) w = 1;
  }
}


    // --- sensor physical position: −5.5 .. +5.5 across 12 sensors ---
    float pos = -5.5f + (float)idx;

    vsum += w;
    wsum += w * pos;
  }

  hitsOut = hits;

  // --- 3) declare "see the line" only if enough sensors are ON or weight present ---
  // Using either condition makes it tolerant of thick/thin lines.
  sawLine = (hitsOut >= 1) || (vsum > 1e-3f);
  if (!sawLine) return lastError;  // keep previous heading briefly

  // --- 4) weighted centroid (QTR readLine-style) normalized to [-1..+1] ---
  if (vsum <= 1e-6f) return lastError;  // guard divide-by-zero
  float pos = (float)(wsum / vsum);     // −5.5 .. +5.5
  float err = pos / 5.5f;               // −1 .. +1  (left +, right −)
  return err;
}

/***** === START: LINE COUNTER MODULE (ไม่แก้ของเก่า) === *****/
// ===== 1) CONFIG =====
static const int LINE_THRESH = 1200;   // เกณฑ์ ADC: ค่าน้อย = เจอเส้น (ปรับตามเซ็นเซอร์ของคุณ)
static const uint8_t CH0_IDX  = 0;     // adcRaw[0] = CH0
static const uint8_t CH1_IDX  = 1;     // adcRaw[1] = CH1
static const uint8_t CH14_IDX = 14;    // adcRaw[14] = CH14
static const uint8_t CH15_IDX = 15;    // adcRaw[15] = CH15

// ===== 2) STATE =====
static volatile uint8_t ch01_hits = 0;   // จำนวนครั้งที่ CH0/1 ชนเส้นแบบถูกต้อง
static bool prev01_onLine = false;       // สถานะก่อนหน้า (edge detect)
// uint32_t pivot_timestamp_ms = 0;      // เผื่อใช้แทน tPivotExit

inline bool onLine(uint16_t v) {
  // IR ค่าน้อย = เส้น
  return (v <= LINE_THRESH);
}

// ===== 3) HOOK หยุดมอเตอร์ =====
// ถ้าคุณมีฟังก์ชันเดิม เช่น stopAllMotors() ให้ประกาศโปรโตไทป์ไว้ตรงนี้ แล้วแทนที่ใน STOP_MOTORS()
/*
extern void stopAllMotors();
#define STOP_MOTORS() do { stopAllMotors(); } while(0)
*/

// ถ้ายังไม่มี ให้ใช้ fallback แบบไม่แตะโค้ดเดิม: ปิด STBY เพื่อตัด TB6612FNG
#ifndef STOP_MOTORS
void __stopMotorsFallback__() {
  Serial.println("STOP");
  #ifdef STBY
    pinMode(STBY, OUTPUT);
    digitalWrite(STBY, LOW);       // เข้า standby => มอเตอร์หยุด
  #endif
  #ifdef PWMA
    analogWrite(PWMA, 0);
  #endif
  #ifdef PWMB
    analogWrite(PWMB, 0);
  #endif
  #ifdef AIN1
    pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
    digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW);
  #endif
  #ifdef BIN1
    pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
    digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW);
  #endif
}
#define STOP_MOTORS() __stopMotorsFallback__()
#endif

// ===== 4) MAIN LOGIC =====
// เรียกฟังก์ชันนี้ "หลังจาก" คุณอ่านค่าเซ็นเซอร์ลง adcRaw[] แล้วใน loop()
void updateStartFinishCounter(const uint16_t* adcRaw) {
  const bool ch0  = onLine(adcRaw[CH0_IDX]);
  const bool ch1  = onLine(adcRaw[CH1_IDX]);
  const bool ch14 = onLine(adcRaw[CH14_IDX]);
  const bool ch15 = onLine(adcRaw[CH15_IDX]);

  const bool now01     = (ch0 || ch1);     // CH0/1 กำลังเจอเส้น?
  const bool gate1415  = (ch14 || ch15);   // กันการบวกถ้า CH14/15 ก็เจอพร้อมกัน

  // Rising edge detection เฉพาะตอน "เพิ่ง" แตะเส้นครั้งใหม่
  const bool centerSeen = (/* นับจาก CH4..CH7 แบบที่คุณมีอยู่ */ countCenterOn() >= 3);
  if (now01 && !prev01_onLine) {
    if (!gate1415||centerSeen) {
      uint32_t tRightCheck = millis();
      // if ((tRightCheck - tPivotExit > 3000) && (tRightCheck - tPivotEnter > 3500) && (mode == FOLLOW) && (tRightCheck - tFollowEnter > 400)) {
      if ( (mode == FOLLOW) && (tRightCheck - tFollowEnter > 500) && (abs(left-right) < 20)) {
      // if ( (mode == FOLLOW)) {
      // if ( (mode == FOLLOW) && (abs(left-right) < 5)) {
        ch01_hits++;
         Serial.println("CH+");
      }
      Serial.printf("[LINE] hit #%u (CH0=%u CH1=%u) gate1415=%d\n",
                                          ch01_hits, adcRaw[CH0_IDX], adcRaw[CH1_IDX], gate1415);//(ถ้าต้องการ debug) 
    }
    // ถ้า CH14/15 แตะพร้อมกัน => ไม่บวก (ตามโจทย์)
  }

  prev01_onLine = now01;

  // ครบ 2 ครั้ง => หยุดมอเตอร์
  if (ch01_hits >= 2) {
    STOP_MOTORS();
  }
}

// ===== 5) OPTIONAL: ยูทิลิตี้รีเซ็ตเคาน์เตอร์/อ่านค่า =====
void resetLineCounterCH01() { ch01_hits = 0; prev01_onLine = false; }
uint8_t getLineCounterCH01() { return ch01_hits; }

/***** === END: LINE COUNTER MODULE === *****/



//==================== 6) SETUP / LOOP ====================
void setup() {
  Serial.begin(115200);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(MUX_SIG, INPUT);

  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(LED, OUTPUT);

  // Enable TB6612
  digitalWrite(STBY, HIGH);

  // PWM config
  analogWriteResolution(PWMA, PWM_BITS);
  analogWriteResolution(PWMB, PWM_BITS);
  analogWriteFrequency(PWMA, PWM_FREQ);
  analogWriteFrequency(PWMB, PWM_FREQ);

  // Warm-up read
  readAllChannels();

    // หน่วงเริ่ม 2 วินาที
  delay(2000);

  tStart = millis();
  tLastPID = millis();

  tFollowEnter = millis();

  // CH0,1 (ปีกขวา) และ CH14,15 (ปีกซ้าย) — ยังไม่ใช้
  // TODO: นำมาใช้เป็น start/stop & curve check ภายหลัง
}

#include <math.h>  // for fabsf


void loop() {
  uint32_t now = millis();

  // Stop condition

  // 1. Read sensors
  readAllChannels();

    // ... โค้ดเดิมของคุณที่อ่าน MUX/IR ลงใน adcRaw[0..15] เสร็จแล้ว ...
  updateStartFinishCounter(adcRaw);  // เรียกหลังอ่านค่า adcRaw[  

  // 2. Compute error, hits, and time delta
  bool sawLine;
  int hitsOut;
  float err = computeError(sawLine, hitsOut);
  float dt = (now - tLastPID) / 1000.0f;
  tLastPID = now;
  if (dt <= 0) dt = 0.001f;

  // 3. Derivative (only use in FOLLOW mode)
  float prevErr = lastError;
  float deriv = (err - prevErr) / dt;
  Serial.print(lastSeenSide);
  Serial.print(" ");
  Serial.println(prevErr); 

  // 4. Remember last seen side when in FOLLOW
  if (mode == FOLLOW && sawLine && fabsf(err) > 0.15f) {
    lastSeenSide = (err >= 0) ? +1 : -1;
  }

  // 5. Hairpin trigger: only when line lost long enough
  if (mode == FOLLOW) {
    Serial.print("F ");
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.println(right);
    Serial.print(sawLine);
    Serial.print(" ");
    if (!sawLine) { 
      if (tLost == 0) tLost = now;
      if ((now - tLost) > LOST_MS && (now - tPivotExit) > PIVOT_COOLDOWN_MS && (fabsf(prevErr) > ERR_HARD || lastSeenSide != 0)) {
        mode = PIVOT;
        pivotDir = (lastSeenSide != 0)
                     ? lastSeenSide
                     : ((prevErr >= 0) ? +1 : -1);
        integral = 0;
      }
    } else {
      tLost = 0;
    }
  }

  // 6. PIVOT mode (spin + center gating + hysteresis exit)
  if (mode == PIVOT) {
    Serial.print("P ");
    // Serial.print(left);
    // Serial.print(" ");
    // Serial.println(right);
    tPivotEnter = millis();
    tFollowEnter = millis();
    static uint32_t tLastBrake = 0;
    if (now - tLastBrake > 30) {
      bothShortBrake(500);
      tLastBrake = now;
    }
    left = -pivotDir * PIVOT_SPEED;
    right = pivotDir * PIVOT_SPEED;

    // driveLRsigned(-pivotDir * PIVOT_SPEED, pivotDir * PIVOT_SPEED);
    driveLRsigned(left, right);

    uint8_t centerOn = countCenterOn();
    bool good = (sawLine) &&  // NEW: must truly see line
                (hitsOut >= K_HITS) && (centerOn >= CENTER_MIN) && (fabsf(err) < ERR_RECENTER);

    recenterStreak = good ? (recenterStreak + 1) : 0;

    if (recenterStreak >= EXIT_HOLD) {
      bothShortBrake(700);
      mode = FOLLOW;
      tLost = 0;
      recenterStreak = 0;
      tPivotExit = now;
      tFollowEnter = millis();
      if (sawLine && fabsf(err) > 0.15f) {
        lastSeenSide = (err >= 0) ? +1 : -1;
      }
    }

    return;  // skip PID during PIVOT
  }

  // 7. FOLLOW mode: PID
  if (!sawLine)
    integral *= 0.95f;
  else
    integral += err * dt;
  integral = constrain(integral, -2.0f, 2.0f);

  float corr = Kp * err + Ki * integral + Kd * deriv;
  lastError = err;

  // 8. Counter-rotation for sharp deviations
  if (err >= TURN_TRIG) {
    motorRightSigned(TURN_FWD);
    motorLeftSigned(-TURN_REV);
    return;
  }
  if (err <= -TURN_TRIG) {
    motorLeftSigned(TURN_FWD);
    motorRightSigned(-TURN_REV);
    return;
  }

  // 9. Normal PID mixing
  int delta = (int)(corr * MAX_DELTA);
  // int left = BASE_SPEED - delta;
  // int right = BASE_SPEED + delta;
  left = BASE_SPEED - delta;
  right = BASE_SPEED + delta;
  left = max(left, 0);
  right = max(right, 0);
  driveLRsigned(left, right);




  // 10. Debug output
  // static uint32_t tLog = 0;
  // if (now - tLog >= 50) {
  //   tLog = now;
    // Serial.print("err=");
    // Serial.print(err, 3);
    // Serial.print(" L=");
    // Serial.print(left);
    // Serial.print(" R=");
    // Serial.print(right);
    // Serial.print(" mode=");
    // Serial.print(mode == FOLLOW ? "F" : "P");
    // Serial.print(" hits=");
    // Serial.print(hitsOut);
    // Serial.print(" centerOn=");
    // Serial.print(countCenterOn());
    // Serial.print("CH");Serial.print(adcRaw[5]);
    // Serial.print(" streak=");
    // Serial.print(recenterStreak);
    // Serial.print(adcRaw[0]);
    // Serial.print(" ");
    // Serial.print(binUsed[4]);
    // Serial.print(" ");
    // Serial.print(binUsed[5]);
    // Serial.print(" ");
    // Serial.print(binUsed[6]);
    // Serial.print(" ");
    // Serial.print(binUsed[7]);
    // Serial.print(" ");
    // Serial.print(binUsed[8]);
    // Serial.print(" ");
    // Serial.print(binUsed[9]);
    // Serial.print(" ");
    // Serial.print(binUsed[10]);
    // Serial.print(" ");
    // Serial.println();
    // for (uint8_t ch = 0; ch < 16; ch++) {
    // Serial.print("CH"); Serial.print(ch); Serial.print(": ");
  //   Serial.println(adcRaw[ch]);
  // }
  // // Serial.println("----");
  // }
}
