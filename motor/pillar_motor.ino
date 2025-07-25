const int controlPin1 = 2;
const int controlPin2 = 4;
const int ENA = 9;
const int IN1 = 6;
const int IN2 = 7;

char currentCommand = 'q'; // 초기값: 정방향

void setup() {
  pinMode(controlPin1, INPUT);
  pinMode(controlPin2, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);  
  digitalWrite(ENA, HIGH);
}

void loop() {
  int signal1 = digitalRead(controlPin1);
  int signal2 = digitalRead(controlPin2);

  digitalWrite(IN1, signal1);
  digitalWrite(IN2, signal2);

}
