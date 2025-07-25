const int controlPin = 2;
const int trigPin = 11;
const int echoPin = 12;
const int IN1 = 6;
const int IN2 = 7;
const int ENA = 9;

float first_distance = 0;
const float margin = 2.0;

void setup() {
  Serial.begin(9600);
  pinMode(controlPin, INPUT);  // 내부 풀업 저항 활성화
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  digitalWrite(ENA, HIGH); // 모터 항상 구동 가능 상태

  delay(500);  // 센서 안정화 대기

  // controlPin이 LOW일 때까지 대기 (신호가 들어올 때까지)
  while (digitalRead(controlPin) != LOW) {
    delay(10);
  }

  first_distance = measureDistance();  // 초기 거리 저장
}

void loop() {
  int control = digitalRead(controlPin);
  float distance = measureDistance();

  // LOW일 때 'a' 신호 = 전진
  if (control == HIGH) {
    if (distance < 10.0) {
      stopMotor();
    } else {
      runMotor();
    }
  } else {  // HIGH = 'b' 신호 = 후진
    if (distance >= first_distance - margin) {
      stopMotor();
    } else {
      backMotor();
    }
  }

  Serial.println(distance);
  Serial.println(control);
  delay(1000);
}

float measureDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  float distance_cm = duration * 0.034 / 2;
  return distance_cm;
}

void runMotor() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
}

void stopMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
}

void backMotor() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
}
