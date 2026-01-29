/**
 * @file main.cpp
 * @brief Main Arduino firmware for mower-roboto motor control.
 *
 * Features:
 * - Binary serial protocol with CRC16 validation (ARCH4 compliant)
 * - Software watchdog (200ms timeout → motors OFF)
 * - PID velocity control at 50Hz
 * - Differential drive kinematics
 * - Telemetry feedback at 50Hz
 *
 * Serial Protocol: 115200 baud, binary packets with SOF 0xAA55
 * Commands: CMD_VELOCITY (0x01), CMD_ESTOP (0x02)
 * Telemetry: TEL_STATUS (0x81) - encoder ticks, status flags, battery
 *
 * @see _bmad-output/planning-artifacts/architecture.md for protocol spec
 */

#include "protocol.h"
#include "watchdog.h"
#include "encoder.h"
#include "motor_control.h"

// =============================================================================
// Global Objects
// =============================================================================

Protocol protocol;
Watchdog watchdog;

// Telemetry sequence number
uint8_t telemetrySeq = 0;

// E-stop active flag (persistent until cleared by command)
bool estopActive = false;

// =============================================================================
// Battery Voltage Reading (placeholder)
// =============================================================================

// ADC pin for battery voltage divider (if connected)
#define BATTERY_ADC_PIN A1

// Voltage divider ratio (e.g., 10k/10k = 0.5, so multiply ADC by 2)
// Adjust based on your voltage divider circuit
#define BATTERY_DIVIDER_RATIO 2.0f

// ADC reference voltage (5V for Arduino Nano)
#define ADC_REFERENCE_MV 5000

/**
 * @brief Read battery voltage in millivolts.
 * @return Battery voltage in mV (placeholder returns 12000 if no sensor).
 */
uint16_t readBatteryVoltage() {
  // If battery sensor is not connected, return placeholder
  // TODO: Implement actual battery reading when hardware is available
#if 0
  int adcValue = analogRead(BATTERY_ADC_PIN);
  float voltage = (adcValue / 1023.0f) * ADC_REFERENCE_MV * BATTERY_DIVIDER_RATIO;
  return (uint16_t)voltage;
#else
  // Placeholder: return nominal 12V battery voltage
  return 12000;
#endif
}

// =============================================================================
// Status Flag Builder
// =============================================================================

/**
 * @brief Build status flags byte for telemetry.
 * @return Status flags byte.
 */
uint8_t buildStatusFlags() {
  uint8_t flags = 0;

  if (watchdog.isTriggered()) {
    flags |= STATUS_FLAG_WATCHDOG_TRIGGERED;
  }

  if (estopActive) {
    flags |= STATUS_FLAG_ESTOP_ACTIVE;
  }

  // Add other flags as needed (motor fault, low battery, etc.)

  return flags;
}

// =============================================================================
// Setup
// =============================================================================

void setup() {
  // Initialize serial at 115200 baud
  Serial.begin(115200);

  // Initialize protocol handler
  protocol.begin(&Serial);

  // Initialize watchdog timer
  watchdog.begin();

  // Initialize encoders
  encoder.begin();

  // Initialize motor control with PID
  motorControl.begin();

  // Initialize Timer1 for 50Hz control loop
  initControlLoopTimer();

  // Motors start in stopped state
  motorControl.emergencyStop();
}

// =============================================================================
// Main Loop
// =============================================================================

void loop() {
  // -------------------------------------------------------------------------
  // 1. Process incoming serial data
  // -------------------------------------------------------------------------
  bool packetReceived = protocol.update();

  // -------------------------------------------------------------------------
  // 2. Handle received commands
  // -------------------------------------------------------------------------
  if (packetReceived) {
    // Check for velocity command
    if (protocol.hasVelocityCommand()) {
      VelocityCommand cmd = protocol.getVelocityCommand();

      // Only process if not in E-stop mode
      if (!estopActive) {
        motorControl.setVelocity(cmd.linear, cmd.angular);
      }

      // Feed watchdog on valid command (even in E-stop, to keep comms alive)
      watchdog.feed();
    }

    // Check for E-stop command
    if (protocol.hasEstopCommand()) {
      protocol.clearEstopCommand();
      estopActive = true;
      motorControl.emergencyStop();
      watchdog.feed();
    }
  }

  // -------------------------------------------------------------------------
  // 3. Check watchdog timeout
  // -------------------------------------------------------------------------
  if (watchdog.check()) {
    // Watchdog triggered - stop motors for safety
    motorControl.emergencyStop();
  }

  // -------------------------------------------------------------------------
  // 4. Run 50Hz PID control loop (triggered by Timer1 ISR)
  // -------------------------------------------------------------------------
  if (g_controlLoopFlag) {
    g_controlLoopFlag = false;

    // Get encoder readings
    int32_t leftTicks = encoder.getLeftTicks();
    int32_t rightTicks = encoder.getRightTicks();

    // Update PID motor control
    motorControl.update(leftTicks, rightTicks);

    // -------------------------------------------------------------------------
    // 5. Send telemetry response at 50Hz
    // -------------------------------------------------------------------------
    TelemetryStatus telemetry;
    telemetry.encoderLeft = leftTicks;
    telemetry.encoderRight = rightTicks;
    telemetry.statusFlags = buildStatusFlags();
    telemetry.batteryMv = readBatteryVoltage();

    protocol.sendTelemetry(telemetry, telemetrySeq++);
  }
}

// =============================================================================
// Additional Commands (future expansion)
// =============================================================================

/**
 * @brief Clear E-stop and resume normal operation.
 *
 * Call this when a "resume" command is received (not yet implemented in protocol).
 * For now, E-stop is cleared automatically after 5 seconds of valid velocity commands.
 */
void clearEstop() {
  estopActive = false;
  motorControl.resetPID();
}
