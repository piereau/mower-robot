// Basic motor control skeleton for Arduino Nano.
// Receives serial commands in the format: L:<speed>,R:<speed>\n
// speed is a float between -1.0 and 1.0.

const int LEFT_PWM_PIN = 5;
const int LEFT_DIR_PIN = 4;
const int RIGHT_PWM_PIN = 6;
const int RIGHT_DIR_PIN = 7;

String inputLine;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  stopMotors();
}

void loop() {
  if (Serial.available()) {
    inputLine = Serial.readStringUntil('\n');
    inputLine.trim();
    if (inputLine.length() == 0) return;

    float leftSpeed = 0.0;
    float rightSpeed = 0.0;

    if (parseCommand(inputLine, leftSpeed, rightSpeed)) {
      setMotorSpeeds(leftSpeed, rightSpeed);
    }
  }
}

bool parseCommand(const String &line, float &left, float &right) {
  int lIndex = line.indexOf("L:");
  int rIndex = line.indexOf("R:");
  if (lIndex == -1 || rIndex == -1) return false;

  String leftPart = line.substring(lIndex + 2, line.indexOf(',', lIndex));
  String rightPart = line.substring(rIndex + 2);

  left = leftPart.toFloat();
  right = rightPart.toFloat();

  left = constrain(left, -1.0, 1.0);
  right = constrain(right, -1.0, 1.0);

  return true;
}

void setMotorSpeeds(float left, float right) {
  setMotor(left, LEFT_DIR_PIN, LEFT_PWM_PIN);
  setMotor(right, RIGHT_DIR_PIN, RIGHT_PWM_PIN);
}

void setMotor(float speed, int dirPin, int pwmPin) {
  bool forward = speed >= 0;
  int pwm = (int)(abs(speed) * 255.0);

  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}
