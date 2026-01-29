/**
 * @file motor_control.cpp
 * @brief PID motor control implementation with Timer1 interrupt.
 */

#include "motor_control.h"
#include "encoder.h"

// =============================================================================
// Global Variables
// =============================================================================

volatile bool g_controlLoopFlag = false;
MotorControl motorControl;

// =============================================================================
// PID Controller Implementation
// =============================================================================

PIDController::PIDController()
    : _kp(DEFAULT_KP),
      _ki(DEFAULT_KI),
      _kd(DEFAULT_KD),
      _integral(0.0f),
      _prevError(0.0f),
      _firstRun(true) {}

void PIDController::begin(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  reset();
}

float PIDController::compute(float setpoint, float measurement, float dt) {
  if (dt <= 0.0f) {
    return 0.0f;
  }

  float error = setpoint - measurement;

  // Proportional term
  float pTerm = _kp * error;

  // Integral term with anti-windup
  _integral += error * dt;
  _integral = constrain(_integral, PID_INTEGRAL_MIN, PID_INTEGRAL_MAX);
  float iTerm = _ki * _integral;

  // Derivative term
  float dTerm = 0.0f;
  if (!_firstRun) {
    float derivative = (error - _prevError) / dt;
    dTerm = _kd * derivative;
  }
  _firstRun = false;
  _prevError = error;

  return pTerm + iTerm + dTerm;
}

void PIDController::reset() {
  _integral = 0.0f;
  _prevError = 0.0f;
  _firstRun = true;
}

void PIDController::setGains(float kp, float ki, float kd) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void PIDController::getGains(float& kp, float& ki, float& kd) const {
  kp = _kp;
  ki = _ki;
  kd = _kd;
}

// =============================================================================
// Differential Drive Kinematics
// =============================================================================

WheelVelocities differentialDriveKinematics(float linear, float angular,
                                            float wheelSeparation) {
  WheelVelocities result;

  // Differential drive equations:
  // v_left  = v_linear - (angular * wheelSeparation / 2)
  // v_right = v_linear + (angular * wheelSeparation / 2)

  float angularComponent = angular * wheelSeparation / 2.0f;

  result.left = linear - angularComponent;
  result.right = linear + angularComponent;

  // Clamp to max wheel speed
  result.left = constrain(result.left, -MAX_WHEEL_SPEED_MS, MAX_WHEEL_SPEED_MS);
  result.right = constrain(result.right, -MAX_WHEEL_SPEED_MS, MAX_WHEEL_SPEED_MS);

  return result;
}

// =============================================================================
// Motor Control Implementation
// =============================================================================

MotorControl::MotorControl()
    : _prevLeftTicks(0),
      _prevRightTicks(0),
      _lastUpdateTime(0),
      _ticksPerRev(ENCODER_TICKS_PER_REV) {
  _targetVelocity.left = 0.0f;
  _targetVelocity.right = 0.0f;
  _measuredVelocity.left = 0.0f;
  _measuredVelocity.right = 0.0f;
}

void MotorControl::begin() {
  // Initialize motor pins
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(LEFT_IN1_PIN, OUTPUT);
  pinMode(LEFT_IN2_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_IN3_PIN, OUTPUT);
  pinMode(RIGHT_IN4_PIN, OUTPUT);

  // Start with motors stopped
  emergencyStop();

  // Initialize PID controllers
  _leftPID.begin(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
  _rightPID.begin(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);

  _lastUpdateTime = millis();
}

void MotorControl::setVelocity(float linear, float angular) {
  _targetVelocity = differentialDriveKinematics(linear, angular, WHEEL_SEPARATION_M);
}

void MotorControl::update(int32_t leftEncoderTicks, int32_t rightEncoderTicks) {
  unsigned long now = millis();
  unsigned long deltaTime = now - _lastUpdateTime;

  if (deltaTime == 0) {
    return;
  }

  // Calculate actual wheel velocities from encoder feedback
  int32_t leftDelta = leftEncoderTicks - _prevLeftTicks;
  int32_t rightDelta = rightEncoderTicks - _prevRightTicks;

  _measuredVelocity.left = calculateWheelVelocity(leftDelta, deltaTime);
  _measuredVelocity.right = calculateWheelVelocity(rightDelta, deltaTime);

  // Update previous values
  _prevLeftTicks = leftEncoderTicks;
  _prevRightTicks = rightEncoderTicks;
  _lastUpdateTime = now;

  // Calculate PID outputs
  float dt = (float)deltaTime / 1000.0f;

  float leftOutput = _leftPID.compute(_targetVelocity.left, _measuredVelocity.left, dt);
  float rightOutput = _rightPID.compute(_targetVelocity.right, _measuredVelocity.right, dt);

  // Combine target velocity with PID correction for feedforward + feedback
  // Normalize to -1.0 to 1.0 range for motor output
  float leftCommand = (_targetVelocity.left + leftOutput) / MAX_WHEEL_SPEED_MS;
  float rightCommand = (_targetVelocity.right + rightOutput) / MAX_WHEEL_SPEED_MS;

  // Clamp outputs
  leftCommand = constrain(leftCommand, -1.0f, 1.0f);
  rightCommand = constrain(rightCommand, -1.0f, 1.0f);

  // Set motor outputs
  setMotor(leftCommand, LEFT_IN1_PIN, LEFT_IN2_PIN, LEFT_PWM_PIN);
  setMotor(rightCommand, RIGHT_IN3_PIN, RIGHT_IN4_PIN, RIGHT_PWM_PIN);
}

void MotorControl::emergencyStop() {
  // Immediately set all PWM to 0
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);

  // Set motor directions to brake (both LOW)
  digitalWrite(LEFT_IN1_PIN, LOW);
  digitalWrite(LEFT_IN2_PIN, LOW);
  digitalWrite(RIGHT_IN3_PIN, LOW);
  digitalWrite(RIGHT_IN4_PIN, LOW);

  // Clear targets and reset PID
  _targetVelocity.left = 0.0f;
  _targetVelocity.right = 0.0f;
  _measuredVelocity.left = 0.0f;
  _measuredVelocity.right = 0.0f;

  resetPID();
}

void MotorControl::stop() {
  _targetVelocity.left = 0.0f;
  _targetVelocity.right = 0.0f;
}

bool MotorControl::isStopped() const {
  return (_targetVelocity.left == 0.0f && _targetVelocity.right == 0.0f);
}

WheelVelocities MotorControl::getMeasuredVelocities() const {
  return _measuredVelocity;
}

WheelVelocities MotorControl::getTargetVelocities() const {
  return _targetVelocity;
}

void MotorControl::setPIDGains(float kp, float ki, float kd) {
  _leftPID.setGains(kp, ki, kd);
  _rightPID.setGains(kp, ki, kd);
}

void MotorControl::resetPID() {
  _leftPID.reset();
  _rightPID.reset();
}

void MotorControl::setMotor(float speed, int in1Pin, int in2Pin, int pwmPin) {
  bool forward = speed >= 0.0f;
  int pwm = (int)(abs(speed) * 255.0f);
  pwm = constrain(pwm, 0, 255);

  if (pwm == 0) {
    // Coast mode (both LOW)
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

float MotorControl::calculateWheelVelocity(int32_t deltaTicks, unsigned long deltaTimeMs) {
  if (deltaTimeMs == 0 || _ticksPerRev == 0) {
    return 0.0f;
  }

  // Convert ticks to distance
  float revolutions = (float)deltaTicks / (float)_ticksPerRev;
  float distance = revolutions * WHEEL_CIRCUMFERENCE_M;

  // Convert to velocity (m/s)
  float timeSeconds = (float)deltaTimeMs / 1000.0f;
  return distance / timeSeconds;
}

// =============================================================================
// Timer1 Interrupt for 50Hz Control Loop
// =============================================================================

void initControlLoopTimer() {
  // Disable interrupts during setup
  noInterrupts();

  // Clear Timer1 control registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // Set compare match value for 50Hz (20ms period)
  // Timer1 is 16-bit, with 1024 prescaler:
  // 16MHz / 1024 = 15625 ticks per second
  // 15625 / 50 = 312.5 ticks per interrupt
  // Use 312 for close approximation
  OCR1A = 312;

  // CTC mode (Clear Timer on Compare Match)
  TCCR1B |= (1 << WGM12);

  // 1024 prescaler
  TCCR1B |= (1 << CS12) | (1 << CS10);

  // Enable Timer1 compare interrupt
  TIMSK1 |= (1 << OCIE1A);

  // Re-enable interrupts
  interrupts();
}

// Timer1 Compare Match A ISR
ISR(TIMER1_COMPA_vect) {
  g_controlLoopFlag = true;
}
