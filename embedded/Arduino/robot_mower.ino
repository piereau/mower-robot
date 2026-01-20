// Basic motor control skeleton for Arduino Nano.
// Receives serial commands in the format: L:<speed>,R:<speed>\n
// speed is a float between -1.0 and 1.0.

// Pont H wiring (Arduino -> driver)
// D12 -> IN4
// D11 -> IN3
// D8  -> IN2
// D7  -> IN1
// D6  -> ENB
// D5  -> ENA

const int LEFT_PWM_PIN = 5;    // ENA
const int LEFT_IN1_PIN = 7;    // IN1
const int LEFT_IN2_PIN = 8;    // IN2
const int RIGHT_PWM_PIN = 6;   // ENB
const int RIGHT_IN3_PIN = 11;  // IN3
const int RIGHT_IN4_PIN = 12;  // IN4

String inputLine;

void setup() {
  Serial.begin(115200);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_IN3_PIN, OUTPUT);
  pinMode(RIGHT_IN4_PIN, OUTPUT);

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
  setMotor(left, LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN);
  setMotor(right, RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN);
}

void setMotor(float speed, int in1Pin, int in2Pin, int pwmPin) {
  bool forward = speed >= 0;
  int pwm = (int)(abs(speed) * 255.0);

  if (pwm == 0) {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
  } else if (forward) {
    digitalWrite(in1Pin, HIGH);
    digitalWrite(in2Pin, LOW);
  } else {
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, HIGH);
  }

  analogWrite(pwmPin, pwm);
}

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}
