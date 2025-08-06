#include <Encoder.h>

// === 핀 정의 ===
#define IN1 4
#define IN2 5
#define ENA 6
#define IN3 8
#define IN4 9
#define ENB 10

#define LEFT_Y 2
#define LEFT_G 3

// === 엔코더 객체 ===
Encoder myEnc(LEFT_Y, LEFT_G);

// === 상태 변수 ===
String input = "";
long oldPosition = -999;
int goSpeed = 70;
int backSpeed = 60;
int pwmSpeed = 150;  // 회전 속도 (0~255)
int bigSpeed = 150;
int smallSpeed = 20;

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

  Serial.println("입력: w / a / s / d / q");
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

    if (input == "w") {
      forward();
    } else if (input == "q") {
      stopMotors();
    } else if (input == "s") {
      backward();
    } else if (input == "d") {
      right();
    } else if (input == "a") {
      left();
    }
  }
}

// === 함수 정의 ===

void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, goSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, goSpeed);

  Serial.println("정회전 시작");
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, backSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, backSpeed);

  Serial.println("역회전 시작");
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 0);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 0);

  Serial.println("모터 정지");
}

void right() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, pwmSpeed);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwmSpeed);

  Serial.println("오른쪽 회전");
}

void left() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, pwmSpeed);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, pwmSpeed);

  Serial.println("왼쪽 회전");
}
