#include <Encoder.h>

// === 핀 정의 ===
#define IN1 4
#define IN2 5
#define ENA 6
#define IN3 8
#define IN4 9
#define ENB 10

#define ENCODER_Y 2
#define ENCODER_G 3

// === 엔코더 객체 ===
Encoder myEnc(ENCODER_Y, ENCODER_G);

// === 상태 변수 ===
String input = "";
long oldPosition = -999;
int goSpeed = 200;
int backSpeed = 100;
int bigSpeed = 80;
int smallSpeed = 50;
int pwmSpeed = 125;  // 기본 속도 (0~255)

void setup() {
  Serial.begin(9600);

  // 모터 제어 핀 설정
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // 초기 상태 정지
  stopMotors();

  Serial.println("입력: go / stop / back / right / left");
}

void loop() {
  // === 엔코더 위치 출력 ===
  long newPosition = myEnc.read();
  if (newPosition != oldPosition) {
    oldPosition = newPosition;
    Serial.print("엔코더 위치: ");
    Serial.println(newPosition);
  }

  // === 명령어 입력 처리 ===
  if (Serial.available()) {
    input = Serial.readStringUntil('\n');
    input.trim(); // 개행 문자 제거

    if (input == "go") {
      forward();
    } else if (input == "stop") {
      stopMotors();
    } else if (input == "back") {
      backward();
    } else if (input == "right") {
      right();
    } else if (input == "left") {
      left();
    }
  }
}

// === 함수 정의 ===

void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwmSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwmSpeed);

  Serial.println("정회전 시작");
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pwmSpeed);

  Serial.println("역회전 시작");
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 0);

  Serial.println("모터 정지");
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, bigSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, smallSpeed);

  Serial.println("오른쪽 회전");
}

void left() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, smallSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, bigSpeed);

  Serial.println("왼쪽 회전");
}
