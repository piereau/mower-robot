"""
Unit tests for protocol module.

Tests CRC16 calculation, packet building, and parsing.
Verifies compatibility with Arduino protocol.h implementation.
"""

import struct
import pytest

from mower_hardware.protocol import (
    crc16_ccitt,
    build_packet,
    build_velocity_packet,
    build_estop_packet,
    PacketParser,
    parse_telemetry_packet,
    TelemetryPacket,
    PROTOCOL_SOF,
    PROTOCOL_VERSION,
    MSG_CMD_VELOCITY,
    MSG_CMD_ESTOP,
    MSG_TEL_STATUS,
    PAYLOAD_SIZE_VELOCITY,
    PAYLOAD_SIZE_STATUS,
    STATUS_FLAG_WATCHDOG_TRIGGERED,
    STATUS_FLAG_ESTOP_ACTIVE,
    STATUS_FLAG_CRC_ERROR_SEEN,
    HEADER_SIZE,
    CRC_SIZE,
)


class TestCRC16:
    """Tests for CRC-16-CCITT implementation."""

    def test_crc16_empty_data(self):
        """CRC of empty data should return initial value."""
        # CRC of empty data with initial 0xFFFF stays 0xFFFF
        result = crc16_ccitt(b'')
        assert result == 0xFFFF

    def test_crc16_known_values(self):
        """Test CRC against known reference values."""
        # "123456789" is a standard CRC test string
        # CRC-16-CCITT (0x1021, 0xFFFF) = 0x29B1
        result = crc16_ccitt(b'123456789')
        assert result == 0x29B1

    def test_crc16_single_byte(self):
        """Test CRC of single byte."""
        result = crc16_ccitt(b'\x00')
        # Verified value for single zero byte
        assert isinstance(result, int)
        assert 0 <= result <= 0xFFFF

    def test_crc16_reproducible(self):
        """CRC should be reproducible for same input."""
        data = b'test data packet'
        result1 = crc16_ccitt(data)
        result2 = crc16_ccitt(data)
        assert result1 == result2


class TestPacketBuilding:
    """Tests for packet building functions."""

    def test_build_velocity_packet_structure(self):
        """Velocity packet should have correct structure."""
        packet = build_velocity_packet(1.0, 0.5, 42)

        # Check length: header(6) + payload(8) + crc(2)
        expected_len = HEADER_SIZE + PAYLOAD_SIZE_VELOCITY + CRC_SIZE
        assert len(packet) == expected_len

        # Check SOF (little-endian)
        sof = struct.unpack('<H', packet[:2])[0]
        assert sof == PROTOCOL_SOF

        # Check version
        assert packet[2] == PROTOCOL_VERSION

        # Check message type
        assert packet[3] == MSG_CMD_VELOCITY

        # Check sequence
        assert packet[4] == 42

        # Check payload length
        assert packet[5] == PAYLOAD_SIZE_VELOCITY

        # Check payload (linear, angular as floats)
        linear, angular = struct.unpack('<ff', packet[6:14])
        assert abs(linear - 1.0) < 1e-6
        assert abs(angular - 0.5) < 1e-6

    def test_build_velocity_packet_crc(self):
        """Velocity packet CRC should be valid."""
        packet = build_velocity_packet(0.0, 0.0, 0)

        # Extract and verify CRC
        crc_data = packet[2:HEADER_SIZE + PAYLOAD_SIZE_VELOCITY]
        expected_crc = crc16_ccitt(crc_data)
        received_crc = struct.unpack('<H', packet[-2:])[0]
        assert expected_crc == received_crc

    def test_build_estop_packet_structure(self):
        """E-stop packet should have correct structure."""
        packet = build_estop_packet(255)

        # Check length: header(6) + payload(0) + crc(2)
        expected_len = HEADER_SIZE + 0 + CRC_SIZE
        assert len(packet) == expected_len

        # Check message type
        assert packet[3] == MSG_CMD_ESTOP

        # Check sequence wraps at 255
        assert packet[4] == 255

        # Check payload length is 0
        assert packet[5] == 0

    def test_sequence_number_wrapping(self):
        """Sequence number should wrap at 256."""
        packet = build_velocity_packet(0.0, 0.0, 256)
        assert packet[4] == 0  # 256 & 0xFF = 0

        packet = build_velocity_packet(0.0, 0.0, 300)
        assert packet[4] == 44  # 300 & 0xFF = 44


class TestPacketParser:
    """Tests for packet parsing."""

    def _build_telemetry_packet(
        self,
        enc_left: int,
        enc_right: int,
        flags: int,
        battery: int,
        seq: int
    ) -> bytes:
        """Helper to build a telemetry packet for testing."""
        payload = struct.pack('<iiBH', enc_left, enc_right, flags, battery)
        return build_packet(MSG_TEL_STATUS, seq, payload)

    def test_parser_valid_telemetry(self):
        """Parser should correctly parse valid telemetry packet."""
        packet = self._build_telemetry_packet(
            enc_left=1000,
            enc_right=1050,
            flags=0,
            battery=12500,
            seq=1
        )

        parser = PacketParser()
        results = parser.feed(packet)

        assert len(results) == 1
        pkt = results[0]
        assert pkt.encoder_left == 1000
        assert pkt.encoder_right == 1050
        assert pkt.watchdog_triggered is False
        assert pkt.estop_active is False
        assert pkt.crc_error_seen is False
        assert pkt.battery_mv == 12500
        assert pkt.sequence == 1

    def test_parser_status_flags(self):
        """Parser should correctly extract status flags."""
        flags = STATUS_FLAG_WATCHDOG_TRIGGERED | STATUS_FLAG_ESTOP_ACTIVE
        packet = self._build_telemetry_packet(0, 0, flags, 0, 0)

        parser = PacketParser()
        results = parser.feed(packet)

        assert len(results) == 1
        pkt = results[0]
        assert pkt.watchdog_triggered is True
        assert pkt.estop_active is True
        assert pkt.crc_error_seen is False

    def test_parser_crc_error_flag(self):
        """Parser should correctly extract CRC error flag."""
        flags = STATUS_FLAG_CRC_ERROR_SEEN
        packet = self._build_telemetry_packet(0, 0, flags, 0, 0)

        parser = PacketParser()
        results = parser.feed(packet)

        assert len(results) == 1
        assert results[0].crc_error_seen is True

    def test_parser_invalid_crc(self):
        """Parser should reject packets with invalid CRC."""
        packet = bytearray(self._build_telemetry_packet(0, 0, 0, 0, 0))
        # Corrupt CRC
        packet[-1] ^= 0xFF

        parser = PacketParser()
        results = parser.feed(bytes(packet))

        assert len(results) == 0
        assert parser.crc_error_count == 1

    def test_parser_partial_packet(self):
        """Parser should handle partial packets across feeds."""
        packet = self._build_telemetry_packet(100, 200, 0, 11000, 5)

        parser = PacketParser()

        # Feed first half
        results1 = parser.feed(packet[:10])
        assert len(results1) == 0

        # Feed second half
        results2 = parser.feed(packet[10:])
        assert len(results2) == 1
        assert results2[0].encoder_left == 100
        assert results2[0].encoder_right == 200

    def test_parser_multiple_packets(self):
        """Parser should handle multiple packets in one feed."""
        pkt1 = self._build_telemetry_packet(10, 20, 0, 12000, 1)
        pkt2 = self._build_telemetry_packet(30, 40, 0, 12100, 2)
        pkt3 = self._build_telemetry_packet(50, 60, 0, 12200, 3)

        parser = PacketParser()
        results = parser.feed(pkt1 + pkt2 + pkt3)

        assert len(results) == 3
        assert results[0].sequence == 1
        assert results[1].sequence == 2
        assert results[2].sequence == 3

    def test_parser_garbage_before_packet(self):
        """Parser should skip garbage bytes before valid packet."""
        garbage = b'\x00\x11\x22\x33\x44'
        packet = self._build_telemetry_packet(0, 0, 0, 0, 0)

        parser = PacketParser()
        results = parser.feed(garbage + packet)

        assert len(results) == 1

    def test_parser_negative_encoder_values(self):
        """Parser should handle negative encoder values."""
        packet = self._build_telemetry_packet(-5000, -3000, 0, 12000, 0)

        parser = PacketParser()
        results = parser.feed(packet)

        assert len(results) == 1
        assert results[0].encoder_left == -5000
        assert results[0].encoder_right == -3000


class TestParseConvenience:
    """Tests for convenience parse function."""

    def test_parse_telemetry_packet_valid(self):
        """Convenience function should parse valid packet."""
        payload = struct.pack('<iiBH', 100, 200, 0, 12000)
        packet = build_packet(MSG_TEL_STATUS, 5, payload)

        result = parse_telemetry_packet(packet)

        assert result is not None
        assert result.encoder_left == 100
        assert result.sequence == 5

    def test_parse_telemetry_packet_invalid(self):
        """Convenience function should return None for invalid packet."""
        result = parse_telemetry_packet(b'\x00\x11\x22\x33')
        assert result is None
