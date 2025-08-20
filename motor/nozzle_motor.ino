// ============================
// Arduino nozzle Code (Sensor + Solenoid)
// ============================
#include <Stepper.h>

const int REV = 2048;
const float DEG_PER_STEP = 360.0 / REV;
const float CM_PER_STEP = 1.0 / 450.0;
const float MIN_DELAY_MS = 2.0;

const int trigPin = 2;
const int echoPin = 3;
const int solenoidPin = 12;

Stepper m1(REV, 8, 10, 9, 11);  // r motor
Stepper m2(REV, 4, 6, 5, 7);    // theta motor

String input = "";
String sendBuffer = "";  // 출력 버퍼

bool ready1 = false, ready2 = false;
int remain1 = 0, remain2 = 0;
int dir1 = 0, dir2 = 0;
float delay1 = 0, delay2 = 0;
unsigned long lastStep1 = 0, lastStep2 = 0;

bool measuring = true;
bool triggered = false;
unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(9600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(solenoidPin, OUTPUT);
  digitalWrite(solenoidPin, LOW);
}

void loop() {
  readSerial();
  updateMotors();
  if (measuring) sendDistance();    //measuring이 True일 경우에만 초음파데이터 송신

  // === 출력 처리 ===
  if (sendBuffer != "") {
    Serial.println(sendBuffer);
    sendBuffer = "";
  }
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (input == "STOP") {        //"STOP"일 경우 측정하지 않음, 초음파 데이터값을 추출하지 않음
        measuring = false;
      } else if (input == "GO" || input == "ABNORMAL") {    //"GO" 또는 "ABNORMAL"의 경우 초음파 데이터 값을 추출함
        measuring = true;
      } else if (input == "HIT") {   //"HIT"의 경우 0.4초 기다리고 솔레노이드로 타일 타격
        digitalWrite(solenoidPin, HIGH);
        delay(400);
        digitalWrite(solenoidPin, LOW);
      } else {
        processCommand(input);    //모터제어 명령 받아옴
      }
      input = "";
    } else {
      input += c;
    }
  }
}

void processCommand(String cmd) {       //모터id, 방향, 속도, 이동거리
  int i1 = cmd.indexOf(',');
  int i2 = cmd.indexOf(',', i1 + 1);
  int i3 = cmd.indexOf(',', i2 + 1);
  if (i1 < 0 || i2 < 0 || i3 < 0) return;
  int id = cmd.substring(0, i1).toInt();
  char dir = cmd.charAt(i1 + 1);
  float speed = cmd.substring(i2 + 1, i3).toFloat();
  float distance = cmd.substring(i3 + 1).toFloat();
  if (id == 1) {
    remain1 = int(distance / CM_PER_STEP + 0.5);            // 거리/속도 => 시간
    delay1 = max(1000.0 * CM_PER_STEP / speed, MIN_DELAY_MS);
    dir1 = (dir == 'F') ? -1 : 1;
    lastStep1 = millis(); ready1 = true;
  } else if (id == 2) {
    remain2 = int(distance / DEG_PER_STEP + 0.5);
    delay2 = max(1000.0 * DEG_PER_STEP / speed, MIN_DELAY_MS);
    dir2 = (dir == 'F') ? -1 : 1;
    lastStep2 = millis(); ready2 = true;
  }
}

void updateMotors() {         //남은 시간이 있다면 계속 돌도록 아닌 경우 멈춤
  unsigned long now = millis();
  if (ready1 && remain1 > 0 && (now - lastStep1) >= delay1) {
    m1.step(dir1); remain1--; lastStep1 = now;
  }
  if (ready2 && remain2 > 0 && (now - lastStep2) >= delay2) {
    m2.step(dir2); remain2--; lastStep2 = now;
  }
  if (remain1 <= 0) ready1 = false;
  if (remain2 <= 0) ready2 = false;

  // === 전원 차단 ===
  if (!ready1 && !ready2) {
    for (int i = 4; i <= 11; i++) {
      digitalWrite(i, LOW);
    }
  }
}

void sendDistance() {         //초음파 데이터 보내는 함수(DIST,~로 보냄)
  if (millis() - lastSendTime > 100) {
    long duration;
    float distance;
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duration = pulseIn(echoPin, HIGH, 20000);
    distance = duration * 0.034 / 2.0;
    Serial.print("DIST,"); Serial.println(distance, 2);
    lastSendTime = millis();
  }
}
