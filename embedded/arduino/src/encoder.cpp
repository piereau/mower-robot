/**
 * @file encoder.cpp
 * @brief Quadrature encoder implementation with interrupt-based counting.
 */

#include "encoder.h"

// =============================================================================
// Global Variables (for ISR access)
// =============================================================================

volatile int32_t g_leftEncoderTicks = 0;
volatile int32_t g_rightEncoderTicks = 0;

// Pin state storage for direction detection
static int g_leftBPin = LEFT_ENCODER_B_PIN;
static int g_rightBPin = RIGHT_ENCODER_B_PIN;

// =============================================================================
// Encoder Class Implementation
// =============================================================================

Encoder::Encoder()
    : _leftAPin(LEFT_ENCODER_A_PIN),
      _leftBPin(LEFT_ENCODER_B_PIN),
      _rightAPin(RIGHT_ENCODER_A_PIN),
      _rightBPin(RIGHT_ENCODER_B_PIN) {}

void Encoder::begin() {
  begin(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN,
        RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);
}

void Encoder::begin(int leftAPin, int leftBPin, int rightAPin, int rightBPin) {
  _leftAPin = leftAPin;
  _leftBPin = leftBPin;
  _rightAPin = rightAPin;
  _rightBPin = rightBPin;

  // Store B pins globally for ISR access
  g_leftBPin = leftBPin;
  g_rightBPin = rightBPin;

#if ENCODER_HARDWARE_ENABLED
  // Configure encoder pins as inputs with pullups
  pinMode(_leftAPin, INPUT_PULLUP);
  pinMode(_leftBPin, INPUT_PULLUP);
  pinMode(_rightAPin, INPUT_PULLUP);
  pinMode(_rightBPin, INPUT_PULLUP);

  // Attach hardware interrupts for channel A of each encoder
  // Using CHANGE for both edges to improve resolution
  attachInterrupt(digitalPinToInterrupt(_leftAPin), leftEncoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(_rightAPin), rightEncoderISR, CHANGE);
#endif

  // Reset counters
  reset();
}

int32_t Encoder::getLeftTicks() const {
  // Disable interrupts briefly to get consistent reading
  noInterrupts();
  int32_t ticks = g_leftEncoderTicks;
  interrupts();
  return ticks;
}

int32_t Encoder::getRightTicks() const {
  noInterrupts();
  int32_t ticks = g_rightEncoderTicks;
  interrupts();
  return ticks;
}

void Encoder::reset() {
  noInterrupts();
  g_leftEncoderTicks = 0;
  g_rightEncoderTicks = 0;
  interrupts();
}

void Encoder::resetLeft() {
  noInterrupts();
  g_leftEncoderTicks = 0;
  interrupts();
}

void Encoder::resetRight() {
  noInterrupts();
  g_rightEncoderTicks = 0;
  interrupts();
}

float Encoder::calculateVelocity(int32_t deltaTicks, uint32_t deltaTimeMs,
                                 uint16_t ticksPerRev, float wheelCircumference) {
  if (deltaTimeMs == 0 || ticksPerRev == 0) {
    return 0.0f;
  }

  // revolutions = ticks / ticksPerRev
  // distance = revolutions * circumference
  // velocity = distance / time

  float revolutions = (float)deltaTicks / (float)ticksPerRev;
  float distance = revolutions * wheelCircumference;
  float timeSeconds = (float)deltaTimeMs / 1000.0f;

  return distance / timeSeconds;
}

void Encoder::simulateTicks(int32_t leftTicks, int32_t rightTicks) {
  noInterrupts();
  g_leftEncoderTicks += leftTicks;
  g_rightEncoderTicks += rightTicks;
  interrupts();
}

// =============================================================================
// Interrupt Service Routines
// =============================================================================

void Encoder::leftEncoderISR() {
  // Read both channels
  bool aState = digitalRead(LEFT_ENCODER_A_PIN);
  bool bState = digitalRead(g_leftBPin);

  // Determine direction based on quadrature relationship
  // If A and B are same, we're going one direction; different = other direction
  if (aState == bState) {
    g_leftEncoderTicks++;
  } else {
    g_leftEncoderTicks--;
  }
}

void Encoder::rightEncoderISR() {
  bool aState = digitalRead(RIGHT_ENCODER_A_PIN);
  bool bState = digitalRead(g_rightBPin);

  // Right wheel may need inverted direction depending on mounting
  // Adjust sign if right wheel counts backwards
  if (aState == bState) {
    g_rightEncoderTicks++;
  } else {
    g_rightEncoderTicks--;
  }
}

// Global encoder instance
Encoder encoder;
