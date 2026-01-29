/**
 * @file encoder.h
 * @brief Quadrature encoder reading with pin change interrupts.
 *
 * Supports two encoders (left and right) using Arduino Nano's
 * PCINT (Pin Change Interrupt) feature for accurate counting.
 */

#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

// =============================================================================
// Encoder Pin Definitions (Arduino Nano)
// =============================================================================

// Left encoder pins (using PCINT for interrupt)
// Adjust these based on your hardware wiring
#define LEFT_ENCODER_A_PIN   2   // INT0 - hardware interrupt
#define LEFT_ENCODER_B_PIN   4   // Digital pin for direction

// Right encoder pins
#define RIGHT_ENCODER_A_PIN  3   // INT1 - hardware interrupt
#define RIGHT_ENCODER_B_PIN  A0  // Analog pin as digital for direction

// =============================================================================
// Encoder Configuration
// =============================================================================

// Ticks per revolution (adjust based on your encoder specifications)
// Common values: 20, 40, 64 for motor encoders
// Set to 0 for simulation mode (no physical encoder)
#define ENCODER_TICKS_PER_REV 20

// Enable/disable encoder hardware (set to 0 for testing without encoders)
#define ENCODER_HARDWARE_ENABLED 1

// =============================================================================
// Encoder Class
// =============================================================================

class Encoder {
public:
  Encoder();

  /**
   * @brief Initialize encoder hardware and interrupts.
   * @param leftAPin Channel A pin for left encoder.
   * @param leftBPin Channel B pin for left encoder.
   * @param rightAPin Channel A pin for right encoder.
   * @param rightBPin Channel B pin for right encoder.
   */
  void begin(int leftAPin, int leftBPin, int rightAPin, int rightBPin);

  /**
   * @brief Initialize with default pin configuration.
   */
  void begin();

  /**
   * @brief Get current tick count for left encoder.
   * @return Accumulated ticks (positive = forward).
   */
  int32_t getLeftTicks() const;

  /**
   * @brief Get current tick count for right encoder.
   * @return Accumulated ticks (positive = forward).
   */
  int32_t getRightTicks() const;

  /**
   * @brief Reset both encoder counts to zero.
   */
  void reset();

  /**
   * @brief Reset left encoder count only.
   */
  void resetLeft();

  /**
   * @brief Reset right encoder count only.
   */
  void resetRight();

  /**
   * @brief Calculate velocity from encoder ticks.
   * @param deltaTicks Change in ticks since last call.
   * @param deltaTimeMs Time interval in milliseconds.
   * @param ticksPerRev Encoder ticks per wheel revolution.
   * @param wheelCircumference Wheel circumference in meters.
   * @return Velocity in meters per second.
   */
  static float calculateVelocity(int32_t deltaTicks, uint32_t deltaTimeMs,
                                 uint16_t ticksPerRev, float wheelCircumference);

  /**
   * @brief Simulate encoder ticks (for testing without hardware).
   * @param leftTicks Ticks to add to left encoder.
   * @param rightTicks Ticks to add to right encoder.
   */
  void simulateTicks(int32_t leftTicks, int32_t rightTicks);

  // Interrupt handlers (must be accessible from ISR)
  static void leftEncoderISR();
  static void rightEncoderISR();

private:
  int _leftAPin;
  int _leftBPin;
  int _rightAPin;
  int _rightBPin;
};

// Global encoder instance (needed for ISR access)
extern Encoder encoder;

// Volatile tick counters (accessed from ISR)
extern volatile int32_t g_leftEncoderTicks;
extern volatile int32_t g_rightEncoderTicks;

#endif // ENCODER_H
