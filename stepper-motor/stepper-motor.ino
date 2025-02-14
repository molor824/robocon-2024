const int EN_PIN = 8;
const int DIR_PIN = 5;
const int PULSE_PIN = 2;

void setup() {
  pinMode(EN_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(PULSE_PIN, OUTPUT);

  digitalWrite(EN_PIN, LOW);
}

#define MIN(a, b) ((a) < (b) ? (a) : (b))
const int PULSE_DELAY = 2000;
void loop() {
  digitalWrite(DIR_PIN, LOW);
  for (int i = 0; i < 200; i++) {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(PULSE_DELAY);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(PULSE_DELAY);
  }
  delay(1000);
  digitalWrite(DIR_PIN, HIGH);
  for (int i = 0; i < 200; i++) {
    digitalWrite(PULSE_PIN, HIGH);
    delayMicroseconds(PULSE_DELAY);
    digitalWrite(PULSE_PIN, LOW);
    delayMicroseconds(PULSE_DELAY);
  }
  delay(1000);
}
