const bool WHEEL_FORWARDS[] = { 1, 0, 0 };
const int WHEEL_PIN_DIRS[] = { 39, 37, 47 };
const int WHEEL_PIN_PWMS[] = { 2, 4, 3 };
const int EXTEND_PIN = 51;
const int THROW_PIN = 53;
const int CATCH_PIN = 49;

const uint8_t EXTEND_BIT = 1;
const uint8_t THROW_BIT = 1 << 1;
const uint8_t CATCH_BIT = 1 << 2;

void setMotorSpeed(int16_t speed, int index) {
  bool forward = WHEEL_FORWARDS[index];
  digitalWrite(WHEEL_PIN_DIRS[index], speed >= 0 ? forward : !forward);
  analogWrite(WHEEL_PIN_PWMS[index], constrain(abs(speed), 0, 255));
}
void setup() {
  SerialUSB.begin(115200);
  Serial2.begin(500000);
  for (int i = 0; i < 3; i++) {
    pinMode(WHEEL_PIN_DIRS[i], OUTPUT);
    pinMode(WHEEL_PIN_PWMS[i], OUTPUT);
    digitalWrite(WHEEL_PIN_PWMS[i], LOW);
  }
  pinMode(EXTEND_PIN, OUTPUT);
  pinMode(THROW_PIN, OUTPUT);
  digitalWrite(EXTEND_PIN, LOW);
  digitalWrite(THROW_PIN, LOW);

  while (Serial2.available() > 0) Serial2.read();
}
struct SerialInput {
  int16_t omni[3];
  uint8_t cylinder;
};
void loop() {
  Serial2.write((uint8_t)0);

  SerialInput input;
  Serial2.readBytes((uint8_t*)&input, sizeof(SerialInput));
  for (int i = 0; i < 3; i++) {
    setMotorSpeed(input.omni[i], i);
  }
  digitalWrite(EXTEND_PIN, input.cylinder & EXTEND_BIT ? HIGH : LOW);
  digitalWrite(THROW_PIN, input.cylinder & THROW_BIT ? HIGH : LOW);
}
