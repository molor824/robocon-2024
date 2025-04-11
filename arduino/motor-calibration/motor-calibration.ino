const bool WHEEL_PINSTATE_FORWARDS[] = { 1, 0, 0 };
const int WHEEL_PIN_DIRS[] = { 39, 37, 47 };
const int WHEEL_PIN_PWMS[] = { 2, 4, 3 };

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  for (int i = 0; i < 3; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
    setMotorSpeed(0, i);
  }
}

void setMotorSpeed(int speed, int index) {
  digitalWrite(WHEEL_PIN_DIRS[index], speed >= 0 ? WHEEL_PINSTATE_FORWARDS[index] : !WHEEL_PINSTATE_FORWARDS[index]);
  analogWrite(WHEEL_PIN_PWMS[index], min(abs(speed), 255));
}
void loop() {
  // put your main code here, to run repeatedly:
  if (Serial.available()) {
    int index = Serial.parseInt();
    int speed = Serial.parseInt();
    setMotorSpeed(speed, index);
  }
}
