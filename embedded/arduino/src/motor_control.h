/**
 * @file motor_control.h
 * @brief PID motor control with differential drive kinematics.
 *
 * Implements:
 * - PID velocity controller for each wheel
 * - Differential drive conversion (linear/angular → left/right wheel speeds)
 * - 50Hz control loop via Timer1 interrupt
 */

#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <Arduino.h>

// =============================================================================
// Robot Physical Parameters
// =============================================================================

// Wheel separation (distance between wheels in meters)
// Adjust based on your robot's dimensions
#define WHEEL_SEPARATION_M     0.30f

// Wheel diameter in meters
#define WHEEL_DIAMETER_M       0.10f

// Wheel circumference
#define WHEEL_CIRCUMFERENCE_M  (WHEEL_DIAMETER_M * 3.14159265f)

// Maximum wheel speed (m/s)
#define MAX_WHEEL_SPEED_MS     1.0f

// =============================================================================
// PID Tuning Parameters
// =============================================================================

// Default PID gains (adjust through testing)
#define DEFAULT_KP  1.0f
#define DEFAULT_KI  0.1f
#define DEFAULT_KD  0.01f

// Integral windup limits
#define PID_INTEGRAL_MAX  100.0f
#define PID_INTEGRAL_MIN  -100.0f

// =============================================================================
// Control Loop Configuration
// =============================================================================

// Control loop frequency in Hz (architecture requirement: 50Hz)
#define CONTROL_LOOP_FREQ_HZ  50

// Control loop period in milliseconds
#define CONTROL_LOOP_PERIOD_MS  (1000 / CONTROL_LOOP_FREQ_HZ)

// =============================================================================
// Motor Pin Configuration (from existing code)
// =============================================================================

// Left motor
#define LEFT_PWM_PIN   5   // ENA
#define LEFT_IN1_PIN   7   // IN1
#define LEFT_IN2_PIN   8   // IN2

// Right motor
#define RIGHT_PWM_PIN  6   // ENB
#define RIGHT_IN3_PIN  11  // IN3
#define RIGHT_IN4_PIN  12  // IN4

// =============================================================================
// PID Controller Class
// =============================================================================

class PIDController {
public:
  PIDController();

  /**
   * @brief Initialize PID with gains.
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param kd Derivative gain.
   */
  void begin(float kp, float ki, float kd);

  /**
   * @brief Compute PID output.
   * @param setpoint Target value.
   * @param measurement Current measured value.
   * @param dt Time delta in seconds.
   * @return Control output.
   */
  float compute(float setpoint, float measurement, float dt);

  /**
   * @brief Reset PID state (integral accumulator and previous error).
   */
  void reset();

  /**
   * @brief Set PID gains.
   */
  void setGains(float kp, float ki, float kd);

  /**
   * @brief Get current gains.
   */
  void getGains(float& kp, float& ki, float& kd) const;

private:
  float _kp, _ki, _kd;
  float _integral;
  float _prevError;
  bool _firstRun;
};

// =============================================================================
// Differential Drive Kinematics
// =============================================================================

struct WheelVelocities {
  float left;   // Left wheel velocity (m/s)
  float right;  // Right wheel velocity (m/s)
};

/**
 * @brief Convert linear and angular velocity to wheel velocities.
 * @param linear Linear velocity (m/s, positive = forward).
 * @param angular Angular velocity (rad/s, positive = counter-clockwise).
 * @param wheelSeparation Distance between wheels in meters.
 * @return WheelVelocities structure with left and right wheel speeds.
 *
 * Differential drive equations:
 *   v_left  = v_linear - (angular * wheelSeparation / 2)
 *   v_right = v_linear + (angular * wheelSeparation / 2)
 */
WheelVelocities differentialDriveKinematics(float linear, float angular,
                                            float wheelSeparation);

// =============================================================================
// Motor Control Class
// =============================================================================

class MotorControl {
public:
  MotorControl();

  /**
   * @brief Initialize motor control hardware and PID controllers.
   */
  void begin();

  /**
   * @brief Set target velocity (linear and angular).
   * @param linear Linear velocity (m/s).
   * @param angular Angular velocity (rad/s).
   */
  void setVelocity(float linear, float angular);

  /**
   * @brief Execute one PID control loop iteration.
   *
   * Call this at CONTROL_LOOP_FREQ_HZ (50Hz) for consistent control.
   * Uses encoder feedback to compute PID corrections.
   *
   * @param leftEncoderTicks Current left encoder tick count.
   * @param rightEncoderTicks Current right encoder tick count.
   */
  void update(int32_t leftEncoderTicks, int32_t rightEncoderTicks);

  /**
   * @brief Emergency stop - immediately sets PWM to 0.
   */
  void emergencyStop();

  /**
   * @brief Stop motors gracefully (sets target velocity to 0).
   */
  void stop();

  /**
   * @brief Check if motors are currently stopped.
   * @return true if target velocity is zero.
   */
  bool isStopped() const;

  /**
   * @brief Get current measured velocities.
   */
  WheelVelocities getMeasuredVelocities() const;

  /**
   * @brief Get current target velocities.
   */
  WheelVelocities getTargetVelocities() const;

  /**
   * @brief Set PID gains for both wheels.
   */
  void setPIDGains(float kp, float ki, float kd);

  /**
   * @brief Reset PID controllers.
   */
  void resetPID();

private:
  PIDController _leftPID;
  PIDController _rightPID;

  WheelVelocities _targetVelocity;
  WheelVelocities _measuredVelocity;

  // Previous encoder values for velocity calculation
  int32_t _prevLeftTicks;
  int32_t _prevRightTicks;
  unsigned long _lastUpdateTime;

  // Encoder ticks per revolution (for velocity calculation)
  uint16_t _ticksPerRev;

  /**
   * @brief Set motor PWM and direction.
   * @param speed Normalized speed (-1.0 to 1.0).
   * @param in1Pin Direction pin 1.
   * @param in2Pin Direction pin 2.
   * @param pwmPin PWM output pin.
   */
  void setMotor(float speed, int in1Pin, int in2Pin, int pwmPin);

  /**
   * @brief Calculate velocity from encoder tick change.
   */
  float calculateWheelVelocity(int32_t deltaTicks, unsigned long deltaTimeMs);
};

// Global motor control instance
extern MotorControl motorControl;

// =============================================================================
// Timer Interrupt for 50Hz Control Loop
// =============================================================================

/**
 * @brief Initialize Timer1 for 50Hz interrupt.
 *
 * Call this once in setup() to enable the control loop timer.
 */
void initControlLoopTimer();

/**
 * @brief Flag set by timer ISR to indicate control loop should run.
 */
extern volatile bool g_controlLoopFlag;

#endif // MOTOR_CONTROL_H
