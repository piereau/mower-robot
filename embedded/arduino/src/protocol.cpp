/**
 * @file protocol.cpp
 * @brief Binary serial protocol implementation for mower-roboto.
 */

#include "protocol.h"

// =============================================================================
// CRC16 Implementation
// =============================================================================

uint16_t crc16_ccitt(const uint8_t* data, uint8_t length) {
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < length; i++) {
    crc ^= ((uint16_t)data[i]) << 8;
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }

  return crc;
}

// =============================================================================
// Protocol Class Implementation
// =============================================================================

Protocol::Protocol()
    : _serial(nullptr),
      _state(PARSER_WAIT_SOF0),
      _rxIndex(0),
      _expectedLength(0),
      _msgType(0),
      _msgSeq(0),
      _payloadLength(0),
      _hasVelocityCmd(false),
      _hasEstopCmd(false),
      _lastSeq(0) {
  memset(_rxBuffer, 0, sizeof(_rxBuffer));
  memset(&_velocityCmd, 0, sizeof(_velocityCmd));
  memset(&_errors, 0, sizeof(_errors));
}

void Protocol::begin(HardwareSerial* serial) {
  _serial = serial;
  _serial->setTimeout(PROTOCOL_SERIAL_TIMEOUT_MS);
  resetParser();
  resetErrors();
}

bool Protocol::update() {
  if (_serial == nullptr) {
    return false;
  }

  bool packetReceived = false;

  // Process all available bytes
  while (_serial->available() > 0) {
    uint8_t byte = _serial->read();
    if (processByte(byte)) {
      packetReceived = true;
    }
  }

  return packetReceived;
}

bool Protocol::processByte(uint8_t byte) {
  switch (_state) {
    case PARSER_WAIT_SOF0:
      if (byte == PROTOCOL_SOF_BYTE0) {
        _rxBuffer[0] = byte;
        _rxIndex = 1;
        _state = PARSER_WAIT_SOF1;
      }
      break;

    case PARSER_WAIT_SOF1:
      if (byte == PROTOCOL_SOF_BYTE1) {
        _rxBuffer[1] = byte;
        _rxIndex = 2;
        _state = PARSER_READ_HEADER;
        _expectedLength = 4; // VER + TYPE + SEQ + LEN
      } else if (byte == PROTOCOL_SOF_BYTE0) {
        // Could be start of new SOF, stay in same position
        _rxBuffer[0] = byte;
        _rxIndex = 1;
      } else {
        resetParser();
      }
      break;

    case PARSER_READ_HEADER:
      _rxBuffer[_rxIndex++] = byte;
      if (_rxIndex >= PROTOCOL_HEADER_SIZE) {
        // Parse header fields
        uint8_t version = _rxBuffer[2];
        _msgType = _rxBuffer[3];
        _msgSeq = _rxBuffer[4];
        _payloadLength = _rxBuffer[5];

        // Validate header
        if (version != PROTOCOL_VERSION) {
          _errors.unknownMsgErrors++;
          resetParser();
          break;
        }

        if (_payloadLength > PROTOCOL_MAX_PAYLOAD) {
          _errors.lengthErrors++;
          resetParser();
          break;
        }

        if (_payloadLength > 0) {
          _state = PARSER_READ_PAYLOAD;
          _expectedLength = _payloadLength;
        } else {
          _state = PARSER_READ_CRC;
          _expectedLength = PROTOCOL_CRC_SIZE;
        }
      }
      break;

    case PARSER_READ_PAYLOAD:
      _rxBuffer[_rxIndex++] = byte;
      if (_rxIndex >= PROTOCOL_HEADER_SIZE + _payloadLength) {
        _state = PARSER_READ_CRC;
        _expectedLength = PROTOCOL_CRC_SIZE;
      }
      break;

    case PARSER_READ_CRC:
      _rxBuffer[_rxIndex++] = byte;
      if (_rxIndex >= PROTOCOL_HEADER_SIZE + _payloadLength + PROTOCOL_CRC_SIZE) {
        bool valid = processPacket();
        resetParser();
        return valid;
      }
      break;
  }

  return false;
}

bool Protocol::processPacket() {
  // Extract CRC from packet (little-endian)
  uint16_t receivedCrc =
      (uint16_t)_rxBuffer[PROTOCOL_HEADER_SIZE + _payloadLength] |
      ((uint16_t)_rxBuffer[PROTOCOL_HEADER_SIZE + _payloadLength + 1] << 8);

  // Calculate CRC over VER + TYPE + SEQ + LEN + PAYLOAD
  uint8_t crcLength = 4 + _payloadLength; // VER, TYPE, SEQ, LEN + payload
  uint16_t calculatedCrc = crc16_ccitt(&_rxBuffer[2], crcLength);

  if (receivedCrc != calculatedCrc) {
    _errors.crcErrors++;
    return false;
  }

  // CRC valid - process message by type
  _lastSeq = _msgSeq;

  switch (_msgType) {
    case MSG_CMD_VELOCITY:
      if (_payloadLength == PAYLOAD_SIZE_VELOCITY) {
        // Extract floats (little-endian)
        memcpy(&_velocityCmd.linear, &_rxBuffer[PROTOCOL_HEADER_SIZE], sizeof(float));
        memcpy(&_velocityCmd.angular, &_rxBuffer[PROTOCOL_HEADER_SIZE + 4], sizeof(float));
        _hasVelocityCmd = true;
        return true;
      } else {
        _errors.lengthErrors++;
      }
      break;

    case MSG_CMD_ESTOP:
      _hasEstopCmd = true;
      return true;

    default:
      _errors.unknownMsgErrors++;
      break;
  }

  return false;
}

void Protocol::resetParser() {
  _state = PARSER_WAIT_SOF0;
  _rxIndex = 0;
  _expectedLength = 0;
}

bool Protocol::hasVelocityCommand() const {
  return _hasVelocityCmd;
}

VelocityCommand Protocol::getVelocityCommand() {
  _hasVelocityCmd = false;
  return _velocityCmd;
}

bool Protocol::hasEstopCommand() const {
  return _hasEstopCmd;
}

void Protocol::clearEstopCommand() {
  _hasEstopCmd = false;
}

void Protocol::sendTelemetry(const TelemetryStatus& status, uint8_t seq) {
  if (_serial == nullptr) {
    return;
  }

  uint8_t packet[PROTOCOL_HEADER_SIZE + PAYLOAD_SIZE_STATUS + PROTOCOL_CRC_SIZE];

  // SOF (little-endian)
  packet[0] = PROTOCOL_SOF_BYTE0;
  packet[1] = PROTOCOL_SOF_BYTE1;

  // Header
  packet[2] = PROTOCOL_VERSION;
  packet[3] = MSG_TEL_STATUS;
  packet[4] = seq;
  packet[5] = PAYLOAD_SIZE_STATUS;

  // Payload (little-endian)
  memcpy(&packet[6], &status.encoderLeft, sizeof(int32_t));
  memcpy(&packet[10], &status.encoderRight, sizeof(int32_t));
  packet[14] = status.statusFlags;
  memcpy(&packet[15], &status.batteryMv, sizeof(uint16_t));

  // CRC over VER + TYPE + SEQ + LEN + PAYLOAD
  uint16_t crc = crc16_ccitt(&packet[2], 4 + PAYLOAD_SIZE_STATUS);
  packet[17] = (uint8_t)(crc & 0xFF);
  packet[18] = (uint8_t)(crc >> 8);

  _serial->write(packet, sizeof(packet));
}

ProtocolErrors Protocol::getErrors() const {
  return _errors;
}

void Protocol::resetErrors() {
  memset(&_errors, 0, sizeof(_errors));
}

uint8_t Protocol::getLastSequence() const {
  return _lastSeq;
}
