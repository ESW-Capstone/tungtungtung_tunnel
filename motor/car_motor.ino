#include <Wire.h>
#include <Encoder.h>
#include <Servo.h>

// ===== I2C =====
#define I2C_ADDR 0x06        // 아두이노 슬레이브 주소

// ===== 모터 핀 =====
#define IN1 4
#define IN2 5
#define ENA 6
#define IN3 8
#define IN4 9
#define ENB 11
#define LED_R 10
#define LED_B 12

// ===== 엔코더 핀 =====
#define LEFT_Y 2
#define LEFT_G 7

// ===== 서보 핀 =====
#define SERVO_PIN 3   // 모터 PWM과 겹치지 않는 핀 사용

Encoder myEnc(LEFT_Y, LEFT_G);
Servo servo;

// ===== 파라미터 =====
int SPEED = 87;
int BIGSPEED = 120;
int SMALLSPEED = 20;

// ===== I2C 수신 버퍼/상태 =====
volatile bool g_stopRequested = false;  // STOP 즉시 처리용
volatile bool g_hasCmd = false;         // 새 명령 도착 표시
String g_lastCmd = "";                  // 마지막 명령(메인 루프에서 처리)

// ====== 프로토타입 ======
void forward();
void backward();
void stopMotors();
void right();
void left();
void moveServoToAndBack(int angle);
void handleCommand(const String& cmd);

// ===== I2C 콜백: 명령 수신 =====
void onI2CReceive(int howMany) {
  // Wire 버퍼는 기본 32바이트
  char buf[33];
  int i = 0;
  while (Wire.available() && i < 32) {
    buf[i++] = (char)Wire.read();
  }
  buf[i] = '\0';

  // 문자열 정리
  String cmd = String(buf);
  cmd.trim();
  if (!cmd.length()) return;

  // STOP 즉시 처리 플래그
  if (cmd.equalsIgnoreCase("STOP")) {
    g_stopRequested = true;
    return;
  }

  // 새 명령 등록 (메인 루프에서 처리)
  g_lastCmd = cmd;
  g_hasCmd = true;
}

// (선택) 요청 시 응답: 필요하면 상태 반환 로직 작성
void onI2CRequest() {
  // 예: 현재 속도 보고
  // char msg[16]; snprintf(msg, sizeof(msg), "SPD=%d", SPEED);
  // Wire.write((uint8_t*)msg, strlen(msg));
}

void setup() {
  // I2C 슬레이브 시작
  Wire.begin(I2C_ADDR);
  Wire.onReceive(onI2CReceive);
  Wire.onRequest(onI2CRequest); // 필요 없으면 제거해도 됨

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(LED_R, OUTPUT); pinMode(LED_B, OUTPUT);

  servo.attach(SERVO_PIN);
  servo.write(0);

  stopMotors();
}

void loop() {
  // I2C로 새 명령 들어왔으면 처리
  if (g_hasCmd) {
    noInterrupts();
    String cmd = g_lastCmd;  // 복사
    g_hasCmd = false;
    interrupts();
    handleCommand(cmd);
  }

  // 필요 시 다른 주기 작업...
}

/* =========================
 *     명령 파서
 * ========================= */
void handleCommand(const String& cmd) {
  if (!cmd.length()) return;

  // ---- 모터 제어 ----
  if (cmd.equalsIgnoreCase("w")) { forward(); return; }
  if (cmd.equalsIgnoreCase("s")) { backward(); return; }
  if (cmd.equalsIgnoreCase("a")) { left(); return; }
  if (cmd.equalsIgnoreCase("d")) { right(); return; }
  if (cmd.equalsIgnoreCase("q")) { stopMotors(); return; }

  // ---- 속도 설정: V<number> ----
  if (cmd.length() >= 2 && (cmd[0]=='V' || cmd[0]=='v')) {
    int v = cmd.substring(1).toInt();
    SPEED = constrain(v, 0, 255);
    return;
  }

  // ---- 서보 왕복: B<number> 또는 숫자만 ----
  if (cmd[0]=='B' || cmd[0]=='b') {
    int deg = cmd.substring(1).toInt();
    moveServoToAndBack(deg);
    return;
  }

  bool allDigits = true;
  for (uint16_t i=0;i<cmd.length();++i) {
    char c = cmd[i];
    if (!(c>='0' && c<='9')) { allDigits = false; break; }
  }
  if (allDigits) {
    moveServoToAndBack(cmd.toInt());
    return;
  }
}

/* =========================
 *     모터 동작 함수
 * ========================= */
void forward() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, SPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, SPEED);
  digitalWrite(LED_B, HIGH); digitalWrite(LED_R, LOW);
}
void backward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  analogWrite(ENA, SPEED);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  analogWrite(ENB, SPEED);
  digitalWrite(LED_B, LOW); digitalWrite(LED_R, HIGH);
}
void stopMotors() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  analogWrite(ENA, 0);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  analogWrite(ENB, 0);
  digitalWrite(LED_B, LOW); digitalWrite(LED_R, HIGH);
}
void right() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, BIGSPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, SMALLSPEED);
}
void left() {
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); analogWrite(ENA, SMALLSPEED);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); analogWrite(ENB, BIGSPEED);
}

/* =========================
 *     서보 제어 (왕복)
 *     - I2C STOP 즉시 반영
 * ========================= */
void moveServoToAndBack(int angle) {
  angle = constrain(angle, 0, 180);

  // 0 -> angle
  for (int a=0; a<=angle; ++a) {
    if (g_stopRequested) { g_stopRequested = false; return; }
    servo.write(a);
    delay(15);
  }
  // angle -> 0
  for (int a=angle; a>=0; --a) {
    if (g_stopRequested) { g_stopRequested = false; return; }
    servo.write(a);
    delay(15);
  }
}
