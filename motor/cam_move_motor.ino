#include <Servo.h>

Servo myServo;         // 서보 객체 생성
int angle = 0;         // 각도 변수
bool increasing = true; // 각도 증가 여부

void setup() {
  myServo.attach(8);   // 서보모터를 디지털 9번 핀에 연결
}

void loop() {
  myServo.write(angle);    // 현재 각도로 서보모터 회전
  delay(40);               // 서보모터가 도달할 시간을 줌

  if (increasing) {
    angle++;
    if (angle >= 30) increasing = false;
  } else {
    angle--;
    if (angle <= 0) increasing = true;
  }
}
