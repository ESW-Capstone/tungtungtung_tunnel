#include <Stepper.h>

const int REV = 2048;
const float DEG_PER_STEP = 360.0 / REV;
const float CM_PER_STEP = 1.0 / 450.0;  //CM별 스텝 수(늘릴수록 더 많이 감)
const float MIN_DELAY_MS = 2.0;  // 너무 빠른 속도 방지

Stepper m1(REV, 8, 10, 9, 11); // r 모터
Stepper m2(REV, 4, 6, 5, 7);  // theta 모터

String input = "";

bool ready1 = false, ready2 = false;

int remain1 = 0, remain2 = 0;
int dir1 = 0, dir2 = 0;
float delay1 = 0, delay2 = 0;
unsigned long lastStep1 = 0, lastStep2 = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  readSerial();
  updateMotors();
}

void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      processCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

void processCommand(String cmd) {
  int i1 = cmd.indexOf(',');
  int i2 = cmd.indexOf(',', i1 + 1);
  int i3 = cmd.indexOf(',', i2 + 1);
  if (i1 < 0 || i2 < 0 || i3 < 0) return;

  int id = cmd.substring(0, i1).toInt();
  char dir = cmd.charAt(i1 + 1);
  float speed = cmd.substring(i2 + 1, i3).toFloat();     // cm/s or deg/s
  float distance = cmd.substring(i3 + 1).toFloat();      // cm or deg

  if (id == 1) {
    remain1 = int(distance / CM_PER_STEP + 0.5);
    delay1 = (speed > 0) ? max(1000.0 * CM_PER_STEP / speed, MIN_DELAY_MS) : 0;
    dir1 = (dir == 'F') ? -1 : 1;
    lastStep1 = millis();
    ready1 = true;
  } else if (id == 2) {
    remain2 = int(distance / DEG_PER_STEP + 0.5);
    delay2 = (speed > 0) ? max(1000.0 * DEG_PER_STEP / speed, MIN_DELAY_MS) : 0;
    dir2 = (dir == 'F') ? -1 : 1;
    lastStep2 = millis();
    ready2 = true;
  }
}

void updateMotors() {
  if (!ready1 && !ready2) return;
 
  unsigned long now = millis();

  if (remain1 > 0 && (now - lastStep1) >= delay1) {
    m1.step(dir1);
    remain1--;
    lastStep1 = now;
  }

  if (remain2 > 0 && (now - lastStep2) >= delay2) {
    m2.step(dir2);
    remain2--;
    lastStep2 = now;
  }

  // delay==0이거나 remain==0인 모터는 다른 모터가 끝날 때까지 대기 유지
  if (remain1 <= 0 && delay1 == 0) lastStep1 = now;
  if (remain2 <= 0 && delay2 == 0) lastStep2 = now;

  if (remain1 <= 0 && remain2 <= 0) {
    ready1 = false;
    ready2 = false;
    Serial.println("[모터] 동시 이동 완료");
  }
}
