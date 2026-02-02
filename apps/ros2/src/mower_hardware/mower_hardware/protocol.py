"""
Binary protocol implementation for Arduino communication.

Matches protocol.h from embedded/arduino/src/protocol.h exactly.
Protocol format: SOF(2) + VER(1) + TYPE(1) + SEQ(1) + LEN(1) + PAYLOAD(n) + CRC16(2)
"""

import struct
from typing import Optional, Tuple, NamedTuple

# Protocol constants (MUST match Arduino protocol.h)
PROTOCOL_SOF = 0xAA55
PROTOCOL_VERSION = 0x01

# Message types
MSG_CMD_VELOCITY = 0x01
MSG_CMD_ESTOP = 0x02
MSG_TEL_STATUS = 0x81

# Payload sizes
PAYLOAD_SIZE_VELOCITY = 8  # linear(f32) + angular(f32)
PAYLOAD_SIZE_ESTOP = 0
PAYLOAD_SIZE_STATUS = 11  # enc_l(i32) + enc_r(i32) + flags(u8) + battery(u16)

# Status flags (bit positions)
STATUS_FLAG_WATCHDOG_TRIGGERED = 0x01
STATUS_FLAG_ESTOP_ACTIVE = 0x02
STATUS_FLAG_CRC_ERROR_SEEN = 0x10

# Protocol header size: SOF(2) + VER(1) + TYPE(1) + SEQ(1) + LEN(1)
HEADER_SIZE = 6
CRC_SIZE = 2


class TelemetryPacket(NamedTuple):
    """Parsed telemetry packet from Arduino."""
    encoder_left: int
    encoder_right: int
    watchdog_triggered: bool
    estop_active: bool
    crc_error_seen: bool
    battery_mv: int
    sequence: int


def crc16_ccitt(data: bytes, initial: int = 0xFFFF) -> int:
    """
    Calculate CRC-16-CCITT checksum.
    
    Polynomial: 0x1021
    Initial value: 0xFFFF
    
    Matches Arduino implementation exactly.
    """
    crc = initial
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc &= 0xFFFF  # Keep 16-bit
    return crc


def build_packet(msg_type: int, seq: int, payload: bytes) -> bytes:
    """
    Build a complete protocol packet.
    
    Format: SOF(2, LE) + VER(1) + TYPE(1) + SEQ(1) + LEN(1) + PAYLOAD(n) + CRC16(2, LE)
    
    Args:
        msg_type: Message type byte (e.g., MSG_CMD_VELOCITY)
        seq: Sequence number (0-255)
        payload: Payload bytes
        
    Returns:
        Complete packet bytes ready for serial transmission
    """
    # Build header + payload for CRC calculation
    header = struct.pack('<HBBBB',
                         PROTOCOL_SOF,
                         PROTOCOL_VERSION,
                         msg_type,
                         seq & 0xFF,
                         len(payload))
    
    # Calculate CRC over header (excluding SOF) + payload
    crc_data = header[2:] + payload  # Skip SOF bytes for CRC
    crc = crc16_ccitt(crc_data)
    
    # Assemble complete packet
    packet = header + payload + struct.pack('<H', crc)
    return packet


def build_velocity_packet(linear: float, angular: float, seq: int) -> bytes:
    """
    Build velocity command packet.
    
    Args:
        linear: Linear velocity in m/s
        angular: Angular velocity in rad/s
        seq: Sequence number
        
    Returns:
        Complete packet bytes
    """
    payload = struct.pack('<ff', linear, angular)
    return build_packet(MSG_CMD_VELOCITY, seq, payload)


def build_estop_packet(seq: int) -> bytes:
    """
    Build emergency stop command packet.
    
    Args:
        seq: Sequence number
        
    Returns:
        Complete packet bytes
    """
    return build_packet(MSG_CMD_ESTOP, seq, b'')


class PacketParser:
    """
    Streaming packet parser with SOF detection and CRC validation.
    
    Handles partial packets and finds packet boundaries in byte stream.
    """
    
    def __init__(self):
        self.buffer = bytearray()
        self.crc_error_count = 0
    
    def feed(self, data: bytes) -> list[TelemetryPacket]:
        """
        Feed bytes to parser and return any complete valid packets.
        
        Args:
            data: Bytes received from serial port
            
        Returns:
            List of parsed telemetry packets (may be empty)
        """
        self.buffer.extend(data)
        packets = []
        
        while True:
            packet = self._try_parse_packet()
            if packet is None:
                break
            packets.append(packet)
        
        # Prevent buffer from growing unbounded
        if len(self.buffer) > 1024:
            # Find last potential SOF and keep from there
            sof_pos = self._find_last_sof()
            if sof_pos > 0:
                self.buffer = self.buffer[sof_pos:]
            elif sof_pos < 0:
                self.buffer.clear()
        
        return packets
    
    def _find_sof(self) -> int:
        """Find SOF (0x55 0xAA in memory = 0xAA55 LE) in buffer."""
        for i in range(len(self.buffer) - 1):
            if self.buffer[i] == 0x55 and self.buffer[i + 1] == 0xAA:
                return i
        return -1
    
    def _find_last_sof(self) -> int:
        """Find last SOF in buffer."""
        for i in range(len(self.buffer) - 2, -1, -1):
            if self.buffer[i] == 0x55 and self.buffer[i + 1] == 0xAA:
                return i
        return -1
    
    def _try_parse_packet(self) -> Optional[TelemetryPacket]:
        """
        Try to parse a complete packet from the buffer.
        
        Returns:
            TelemetryPacket if valid packet found, None otherwise
        """
        # Find SOF
        sof_pos = self._find_sof()
        if sof_pos < 0:
            return None
        
        # Discard bytes before SOF
        if sof_pos > 0:
            del self.buffer[:sof_pos]
        
        # Need at least header
        if len(self.buffer) < HEADER_SIZE:
            return None
        
        # Parse header
        # SOF is stored as 0x55 0xAA in little-endian
        _sof, version, msg_type, seq, payload_len = struct.unpack_from('<HBBBB', self.buffer)
        
        # Check version
        if version != PROTOCOL_VERSION:
            # Invalid version, skip this SOF and try next
            del self.buffer[:2]
            return None
        
        # Calculate total packet length
        packet_len = HEADER_SIZE + payload_len + CRC_SIZE
        
        # Wait for complete packet
        if len(self.buffer) < packet_len:
            return None
        
        # Extract packet bytes
        packet_bytes = bytes(self.buffer[:packet_len])
        
        # Verify CRC (over header excluding SOF + payload)
        crc_data = packet_bytes[2:HEADER_SIZE + payload_len]
        expected_crc = crc16_ccitt(crc_data)
        received_crc = struct.unpack_from('<H', packet_bytes, HEADER_SIZE + payload_len)[0]
        
        if expected_crc != received_crc:
            self.crc_error_count += 1
            # Skip this SOF and try next
            del self.buffer[:2]
            return None
        
        # Remove packet from buffer
        del self.buffer[:packet_len]
        
        # Parse payload based on message type
        if msg_type == MSG_TEL_STATUS and payload_len == PAYLOAD_SIZE_STATUS:
            return self._parse_telemetry_payload(
                packet_bytes[HEADER_SIZE:HEADER_SIZE + payload_len],
                seq
            )
        
        # Unknown or unhandled message type
        return None
    
    def _parse_telemetry_payload(self, payload: bytes, seq: int) -> TelemetryPacket:
        """Parse telemetry status payload."""
        enc_left, enc_right, flags, battery_mv = struct.unpack('<iiBH', payload)
        
        return TelemetryPacket(
            encoder_left=enc_left,
            encoder_right=enc_right,
            watchdog_triggered=bool(flags & STATUS_FLAG_WATCHDOG_TRIGGERED),
            estop_active=bool(flags & STATUS_FLAG_ESTOP_ACTIVE),
            crc_error_seen=bool(flags & STATUS_FLAG_CRC_ERROR_SEEN),
            battery_mv=battery_mv,
            sequence=seq
        )


def parse_telemetry_packet(data: bytes) -> Optional[TelemetryPacket]:
    """
    Parse a single complete telemetry packet.
    
    Convenience function for parsing a known-complete packet.
    For streaming data, use PacketParser class.
    
    Args:
        data: Complete packet bytes including SOF and CRC
        
    Returns:
        TelemetryPacket if valid, None otherwise
    """
    parser = PacketParser()
    packets = parser.feed(data)
    return packets[0] if packets else None
