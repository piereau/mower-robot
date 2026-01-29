/**
 * @file watchdog.h
 * @brief Software watchdog timer for motor safety.
 *
 * If no valid command is received within WATCHDOG_TIMEOUT_MS,
 * motors are automatically stopped (fail-safe behavior).
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include <Arduino.h>

// Watchdog timeout in milliseconds (ARCH4 requirement: 200ms)
#define WATCHDOG_TIMEOUT_MS 200

/**
 * @brief Software watchdog for motor safety.
 *
 * Usage:
 *   Watchdog wd;
 *   wd.begin();
 *
 *   // In command handler:
 *   wd.feed();
 *
 *   // In main loop:
 *   if (wd.check()) {
 *     stopMotors();
 *   }
 */
class Watchdog {
public:
  Watchdog();

  /**
   * @brief Initialize the watchdog timer.
   */
  void begin();

  /**
   * @brief Feed the watchdog (call when valid command received).
   *
   * This resets the timeout counter and clears the triggered flag
   * if it was set.
   */
  void feed();

  /**
   * @brief Check if watchdog has timed out.
   * @return true if timeout has occurred and motors should be stopped.
   */
  bool check();

  /**
   * @brief Check if watchdog was triggered since last feed.
   * @return true if watchdog timeout has occurred.
   */
  bool isTriggered() const;

  /**
   * @brief Get time since last valid command in milliseconds.
   * @return Milliseconds since last feed() call.
   */
  unsigned long timeSinceLastCommand() const;

private:
  unsigned long _lastValidCommandTime;
  bool _triggered;
};

// =============================================================================
// Inline Implementation
// =============================================================================

inline Watchdog::Watchdog()
    : _lastValidCommandTime(0), _triggered(false) {}

inline void Watchdog::begin() {
  _lastValidCommandTime = millis();
  _triggered = false;
}

inline void Watchdog::feed() {
  _lastValidCommandTime = millis();
  _triggered = false;
}

inline bool Watchdog::check() {
  if (millis() - _lastValidCommandTime > WATCHDOG_TIMEOUT_MS) {
    _triggered = true;
    return true;
  }
  return false;
}

inline bool Watchdog::isTriggered() const {
  return _triggered;
}

inline unsigned long Watchdog::timeSinceLastCommand() const {
  return millis() - _lastValidCommandTime;
}

#endif // WATCHDOG_H
