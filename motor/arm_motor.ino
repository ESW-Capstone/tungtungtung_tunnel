// ============================
// Arduino2 Code (Slider + Interconnect)
// ============================
#include <Stepper.h>
#include <SoftwareSerial.h>

// === 핀 및 상수 설정 ===
SoftwareSerial softSerial(2, 3);  // RX, TX
const int REV = 2048;
const float CM_PER_STEP = 1.0 / 450.0;
const float MIN_DELAY_MS = 2.0;
Stepper motor(REV, 8, 10, 9, 11);

// === 제어 상태 변수 ===
String input = "";
bool moving = false;
bool go_mode = false;
bool abnormal_mode = false;
int steps = 0, dir = 1;
float delayMs = 0;
unsigned long lastStep = 0;
float total_moved_cm = 0.0;
float distance_cm = 999.0;
const float target_distance = 5.0;

// === 초기화 ===
void setup() {
  Serial.begin(9600);
  softSerial.begin(9600);
}

// === 메인 루프 ===
void loop() {
  readSerial();
  readArduino1Serial();

  if (moving) {
    if (go_mode && distance_cm < target_distance) {
      moving = false;
      go_mode = false;
      Serial.println("STOP");
      softSerial.println("STOP");
      stopMotor();
      Serial.println("HIT_READY");
      softSerial.println("HIT");
    } else if (abnormal_mode && distance_cm > 10.0) {
      moving = false;
      abnormal_mode = false;
      Serial.println("READY");
      stopMotor();
    } else {
      updateMotor();
    }
  }
}

// === 라즈베리파이 명령 수신 ===
void readSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      handlePiCommand(input);
      input = "";
    } else {
      input += c;
    }
  }
}

// === 명령 해석 및 처리 ===
void handlePiCommand(String cmd) {
  if (cmd == "GO") {
    dir = 1;
    delayMs = 5;
    moving = true;
    go_mode = true;
    abnormal_mode = false;
    lastStep = millis();
    distance_cm = 999.0;
    softSerial.println("GO");
  } else if (cmd == "ABNORMAL") {
    dir = -1;
    delayMs = 5;
    steps = 2000;
    moving = true;
    abnormal_mode = true;
    go_mode = false;
    lastStep = millis();
    softSerial.println("ABNORMAL");
  } else if (cmd == "NORMAL") {
    softSerial.println("STOP");
    dir = -1;
    delayMs = 5;
    steps = int(abs(total_moved_cm) / CM_PER_STEP);
    moving = true;
    lastStep = millis();
  } else if (cmd == "QUIT") {
    moving = false;
    go_mode = false;
    abnormal_mode = false;
    steps = 0;
    total_moved_cm = 0.0;
    stopMotor();
    softSerial.println("STOP");
    Serial.println("QUIT: 모든 모터 정지");
  } else if (cmd.startsWith("1,") || cmd.startsWith("2,")) {
    softSerial.println(cmd);  // Arduino1에 중계
  }
}

// === 아두이노1로부터 거리 수신 ===
String lastCommand = "";

void readArduino1Serial() {
  static String distInput = "";
  while (softSerial.available()) {
    char c = softSerial.read();
    if (c == '\n' || c == '\r') {
      distInput.trim();
      lastCommand = distInput;
      distInput = "";
    } else {
      distInput += c;
    }
  }

  // 수신 후 처리: 송신은 여기서만!
  if (lastCommand == "CHECK") {
    Serial.println("CHECK");
    lastCommand = "";
  } else if (lastCommand.startsWith("DIST,")) {
    distance_cm = lastCommand.substring(5).toFloat();
    lastCommand = "";
  }
}


// === 슬라이드 모터 구동 ===
void updateMotor() {
  unsigned long now = millis();
  if ((now - lastStep) >= delayMs) {
    motor.step(dir);
    if (go_mode || abnormal_mode)
      total_moved_cm += dir * CM_PER_STEP;
    if (!go_mode && !abnormal_mode && steps > 0)
      steps--;
    lastStep = now;
  }

  if (!go_mode && !abnormal_mode && steps == 0) {
    moving = false;
    total_moved_cm = 0.0;
    stopMotor();
  }
}

// === 모터 드라이버 OFF (LOW 출력) ===
void stopMotor() {
  for (int i = 8; i < 12; i++) {
    digitalWrite(i, LOW);
  }
}
