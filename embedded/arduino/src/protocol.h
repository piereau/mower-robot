/**
 * @file protocol.h
 * @brief Binary serial protocol constants and packet handling for mower-roboto.
 *
 * Protocol format (ARCH4 compliant):
 * ┌──────┬─────┬──────┬─────┬─────┬─────────┬───────┐
 * │ SOF  │ VER │ TYPE │ SEQ │ LEN │ PAYLOAD │ CRC16 │
 * │ 2B   │ 1B  │ 1B   │ 1B  │ 1B  │ N bytes │ 2B    │
 * └──────┴─────┴──────┴─────┴─────┴─────────┴───────┘
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <Arduino.h>

// =============================================================================
// Protocol Constants
// =============================================================================

// Start of frame marker (little-endian: 0x55 first, then 0xAA)
#define PROTOCOL_SOF          0xAA55
#define PROTOCOL_SOF_BYTE0    0x55
#define PROTOCOL_SOF_BYTE1    0xAA

// Protocol version
#define PROTOCOL_VERSION      0x01

// Maximum packet sizes
#define PROTOCOL_HEADER_SIZE  6   // SOF(2) + VER(1) + TYPE(1) + SEQ(1) + LEN(1)
#define PROTOCOL_CRC_SIZE     2
#define PROTOCOL_MAX_PAYLOAD  16  // Maximum payload length
#define PROTOCOL_MAX_PACKET   (PROTOCOL_HEADER_SIZE + PROTOCOL_MAX_PAYLOAD + PROTOCOL_CRC_SIZE)

// Receive buffer size (should be >= max packet size)
#define PROTOCOL_RX_BUFFER_SIZE 32

// Serial timeout for reads (milliseconds)
#define PROTOCOL_SERIAL_TIMEOUT_MS 10

// =============================================================================
// Message Types
// =============================================================================

// Commands (RPi → Arduino): 0x00-0x7F
#define MSG_CMD_VELOCITY      0x01  // Velocity command (linear, angular floats)
#define MSG_CMD_ESTOP         0x02  // Emergency stop

// Telemetry (Arduino → RPi): 0x80-0xFF
#define MSG_TEL_STATUS        0x81  // Status telemetry response

// =============================================================================
// Payload Sizes
// =============================================================================

#define PAYLOAD_SIZE_VELOCITY 8   // linear(float) + angular(float)
#define PAYLOAD_SIZE_ESTOP    0   // No payload
#define PAYLOAD_SIZE_STATUS   11  // enc_left(int32) + enc_right(int32) + flags(uint8) + battery_mv(uint16)

// =============================================================================
// Status Flags (bit positions in status byte)
// =============================================================================

#define STATUS_FLAG_WATCHDOG_TRIGGERED  0x01
#define STATUS_FLAG_ESTOP_ACTIVE        0x02
#define STATUS_FLAG_MOTOR_FAULT         0x04
#define STATUS_FLAG_LOW_BATTERY         0x08
#define STATUS_FLAG_CRC_ERROR_SEEN      0x10

// =============================================================================
// Error Counters Structure
// =============================================================================

struct ProtocolErrors {
  uint16_t crcErrors;
  uint16_t lengthErrors;
  uint16_t timeoutErrors;
  uint16_t unknownMsgErrors;
};

// =============================================================================
// Velocity Command Structure
// =============================================================================

struct VelocityCommand {
  float linear;   // Linear velocity (m/s, positive = forward)
  float angular;  // Angular velocity (rad/s, positive = counter-clockwise)
};

// =============================================================================
// Telemetry Status Structure
// =============================================================================

struct TelemetryStatus {
  int32_t encoderLeft;
  int32_t encoderRight;
  uint8_t statusFlags;
  uint16_t batteryMv;
};

// =============================================================================
// Parser State Machine
// =============================================================================

enum ParserState {
  PARSER_WAIT_SOF0,
  PARSER_WAIT_SOF1,
  PARSER_READ_HEADER,
  PARSER_READ_PAYLOAD,
  PARSER_READ_CRC
};

// =============================================================================
// Protocol Handler Class
// =============================================================================

class Protocol {
public:
  Protocol();

  /**
   * @brief Initialize the protocol handler.
   * @param serial Pointer to the HardwareSerial instance (e.g., &Serial).
   */
  void begin(HardwareSerial* serial);

  /**
   * @brief Process incoming serial data and check for complete packets.
   * @return true if a valid packet was received and processed.
   */
  bool update();

  /**
   * @brief Check if a velocity command was received.
   * @return true if a new velocity command is available.
   */
  bool hasVelocityCommand() const;

  /**
   * @brief Get the last received velocity command.
   * @return VelocityCommand structure.
   */
  VelocityCommand getVelocityCommand();

  /**
   * @brief Check if an E-stop command was received.
   * @return true if E-stop was triggered.
   */
  bool hasEstopCommand() const;

  /**
   * @brief Clear the E-stop command flag.
   */
  void clearEstopCommand();

  /**
   * @brief Send telemetry status packet.
   * @param status TelemetryStatus structure to send.
   * @param seq Sequence number for this packet.
   */
  void sendTelemetry(const TelemetryStatus& status, uint8_t seq);

  /**
   * @brief Get error counters.
   * @return ProtocolErrors structure.
   */
  ProtocolErrors getErrors() const;

  /**
   * @brief Reset error counters.
   */
  void resetErrors();

  /**
   * @brief Get last received sequence number.
   * @return Sequence number of last valid command.
   */
  uint8_t getLastSequence() const;

private:
  HardwareSerial* _serial;
  ParserState _state;
  uint8_t _rxBuffer[PROTOCOL_RX_BUFFER_SIZE];
  uint8_t _rxIndex;
  uint8_t _expectedLength;

  // Parsed packet fields
  uint8_t _msgType;
  uint8_t _msgSeq;
  uint8_t _payloadLength;

  // Command state
  VelocityCommand _velocityCmd;
  bool _hasVelocityCmd;
  bool _hasEstopCmd;
  uint8_t _lastSeq;

  // Error tracking
  ProtocolErrors _errors;

  /**
   * @brief Process a single received byte through the state machine.
   * @param byte The received byte.
   * @return true if a complete valid packet was processed.
   */
  bool processByte(uint8_t byte);

  /**
   * @brief Validate and process a complete packet.
   * @return true if packet was valid and processed.
   */
  bool processPacket();

  /**
   * @brief Reset parser state machine to initial state.
   */
  void resetParser();
};

// =============================================================================
// CRC16 Functions
// =============================================================================

/**
 * @brief Calculate CRC-16-CCITT checksum.
 * @param data Pointer to data buffer.
 * @param length Number of bytes.
 * @return 16-bit CRC value.
 *
 * Uses polynomial 0x1021 with initial value 0xFFFF.
 * Calculate over: VER + TYPE + SEQ + LEN + PAYLOAD
 */
uint16_t crc16_ccitt(const uint8_t* data, uint8_t length);

#endif // PROTOCOL_H
