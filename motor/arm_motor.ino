#include <Stepper.h>

const int REV = 2048;
const float CM_PER_STEP = 1.0 / 450.0;
const float MIN_DELAY_MS = 2.0;

Stepper m1(REV, 4, 6, 5, 7); // r 모터

int remain1 = 0;
int dir1 = 0;
float delay1 = 0;
unsigned long lastStep1 = 0;

void setup() {
  // 시리얼 제거됨
}

void loop() {
  static bool forward = true;
  static bool inMotion = false;
  static unsigned long lastDoneTime = 0;
  const unsigned long waitAfterDone = 1000;

  // 모터 명령 전송
  if (!inMotion && millis() - lastDoneTime > waitAfterDone) {
    remain1 = int(10.0 / CM_PER_STEP + 0.5); // 5cm 거리
    delay1 = max(1000.0 * CM_PER_STEP / 2.0, MIN_DELAY_MS); // 2cm/s 속도
    dir1 = (forward) ? -1 : 1;
    lastStep1 = millis();
    forward = !forward;
    inMotion = true;
  }

  // 모터 한 스텝씩 이동
  if (inMotion && remain1 > 0 && (millis() - lastStep1) >= delay1) {
    m1.step(dir1);
    remain1--;
    lastStep1 = millis();
  }

  // 이동 완료 처리
  if (inMotion && remain1 == 0) {
    inMotion = false;
    lastDoneTime = millis();
  }
}
