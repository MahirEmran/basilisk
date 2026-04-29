"""
HIL Packet Codec for Basilisk-BBB Communication

This module provides serialization and deserialization of packets for
Hardware-in-the-Loop communication between Basilisk and the BeagleBone Black.
"""

import struct
import zlib
from typing import Tuple, Dict, Any, Optional
from dataclasses import dataclass
import numpy as np


# Packet type identifiers
PACKET_TYPE_TRUTH_NAV = 0x01
PACKET_TYPE_ENV = 0x02
PACKET_TYPE_POWER_THERMAL = 0x03
PACKET_TYPE_FC_COMMAND = 0x04
PACKET_TYPE_HEALTH = 0x05

# Protocol version
PROTOCOL_VERSION = 1

HEADER_FMT = "<IHHII"
HEADER_SIZE = struct.calcsize(HEADER_FMT)

# Magic numbers for packet validation
STATE_PACKET_MAGIC = 0x42424242  # "BBBB"
COMMAND_PACKET_MAGIC = 0x43434343  # "CCCC"


@dataclass
class TruthNavPacket:
    """Spacecraft navigation truth packet"""
    timestamp: int  # nanoseconds
    position: np.ndarray  # [x, y, z] in meters
    velocity: np.ndarray  # [vx, vy, vz] in m/s
    attitude: np.ndarray  # quaternion [q0, q1, q2, q3]
    rate: np.ndarray  # [wx, wy, wz] in rad/s
    sequence: int


@dataclass
class EnvPacket:
    """Environment packet"""
    timestamp: int  # nanoseconds
    sun_vector: np.ndarray  # Sun unit vector in inertial frame
    moon_vector: np.ndarray  # Moon unit vector in inertial frame
    eclipse_flag: bool
    station_visibility_summary: Dict[str, bool]
    sequence: int


@dataclass
class PowerThermalPacket:
    """Power and thermal packet"""
    timestamp: int  # nanoseconds
    state_of_charge: float  # Battery SOC (0.0 to 1.0)
    net_power: float  # Net power in watts
    payload_temp: float  # Payload temperature in Kelvin
    sequence: int


@dataclass
class FcCommandPacket:
    """Flight computer command packet"""
    timestamp: int  # nanoseconds
    command_type: str  # Command type (e.g., "SETMODE", "RESET")
    command_data: bytes  # Command data
    validity_window: int  # Validity window in nanoseconds
    sequence: int


@dataclass
class HealthPacket:
    """Health and status packet"""
    timestamp: int  # nanoseconds
    heartbeat: int  # Heartbeat counter
    process_uptime: int  # Process uptime in nanoseconds
    watchdog_flags: int  # Watchdog status flags
    sequence: int


class HilPacketCodec:
    """Codec for HIL packet serialization/deserialization"""

    def __init__(self):
        self.sequence_counters = {
            PACKET_TYPE_TRUTH_NAV: 0,
            PACKET_TYPE_ENV: 0,
            PACKET_TYPE_POWER_THERMAL: 0,
            PACKET_TYPE_FC_COMMAND: 0,
            PACKET_TYPE_HEALTH: 0
        }

    def get_next_sequence(self, packet_type: int) -> int:
        """Get next sequence number for packet type"""
        seq = self.sequence_counters[packet_type]
        self.sequence_counters[packet_type] = (seq + 1) & 0xFFFFFFFF  # Wrap around
        return seq

    def encode_truth_nav(self, packet: TruthNavPacket) -> bytes:
        """Encode truth navigation packet to bytes"""
        # Body: timestamp(8) + position(24) + velocity(24) + attitude(16) + rate(24)
        body = struct.pack('<Q',
                          packet.timestamp)
        body += packet.position.astype(np.float64).tobytes()
        body += packet.velocity.astype(np.float64).tobytes()
        body += packet.attitude.astype(np.float64).tobytes()
        body += packet.rate.astype(np.float64).tobytes()

        # Update length in header first
        length = len(body)

        # Header: magic(4) + version(2) + type(2) + length(4) + seq(4)
        header = struct.pack(HEADER_FMT,
                            STATE_PACKET_MAGIC,
                            PROTOCOL_VERSION,
                            PACKET_TYPE_TRUTH_NAV,
                            length,
                            packet.sequence)

        # CRC32 checksum (calculate with final header)
        crc = zlib.crc32(header + body) & 0xFFFFFFFF

        return header + body + struct.pack('<I', crc)

    def decode_truth_nav(self, data: bytes) -> Optional[TruthNavPacket]:
        """Decode truth navigation packet from bytes"""
        if len(data) < HEADER_SIZE:  # Minimum header size
            return None

        # Parse header
        magic, version, ptype, length, seq = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])

        if magic != STATE_PACKET_MAGIC:
            print(f"Invalid magic number: {magic:#x}")
            return None

        if version != PROTOCOL_VERSION:
            print(f"Unsupported protocol version: {version}")
            return None

        if ptype != PACKET_TYPE_TRUTH_NAV:
            print(f"Unexpected packet type: {ptype}")
            return None

        if len(data) < HEADER_SIZE + length + 4:  # header + body + crc
            return None

        # Extract body and CRC
        body = data[HEADER_SIZE:HEADER_SIZE+length]
        crc_received = struct.unpack('<I', data[HEADER_SIZE+length:HEADER_SIZE+length+4])[0]

        # Verify CRC
        crc_calculated = zlib.crc32(data[:HEADER_SIZE+length]) & 0xFFFFFFFF
        if crc_received != crc_calculated:
            print(f"CRC mismatch: received {crc_received:#x}, calculated {crc_calculated:#x}")
            return None

        # Parse body
        timestamp = struct.unpack('<Q', body[:8])[0]
        position = np.frombuffer(body[8:32], dtype=np.float64)
        velocity = np.frombuffer(body[32:56], dtype=np.float64)
        attitude = np.frombuffer(body[56:72], dtype=np.float64)
        rate = np.frombuffer(body[72:96], dtype=np.float64)

        return TruthNavPacket(
            timestamp=timestamp,
            position=position,
            velocity=velocity,
            attitude=attitude,
            rate=rate,
            sequence=seq
        )

    def encode_fc_command(self, packet: FcCommandPacket) -> bytes:
        """Encode flight computer command packet to bytes"""
        # Body: timestamp(8) + cmd_type_len(2) + cmd_type + cmd_data_len(2) + cmd_data + validity(4)
        cmd_type_bytes = packet.command_type.encode('utf-8')[:8]  # Max 8 bytes
        cmd_type_padded = cmd_type_bytes.ljust(8, b'\x00')

        body = struct.pack('<Q',
                          packet.timestamp)
        body += cmd_type_padded
        body += struct.pack('<H', len(packet.command_data))
        body += packet.command_data
        body += struct.pack('<Q', packet.validity_window)

        # Update length in header first
        length = len(body)

        # Header: magic(4) + version(2) + type(2) + length(4) + seq(4)
        header = struct.pack(HEADER_FMT,
                            COMMAND_PACKET_MAGIC,
                            PROTOCOL_VERSION,
                            PACKET_TYPE_FC_COMMAND,
                            length,
                            packet.sequence)

        # CRC32 checksum (calculate with final header)
        crc = zlib.crc32(header + body) & 0xFFFFFFFF

        return header + body + struct.pack('<I', crc)

    def decode_fc_command(self, data: bytes) -> Optional[FcCommandPacket]:
        """Decode flight computer command packet from bytes"""
        if len(data) < HEADER_SIZE:  # Minimum header size
            return None

        # Parse header
        magic, version, ptype, length, seq = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])

        if magic != COMMAND_PACKET_MAGIC:
            print(f"Invalid magic number: {magic:#x}")
            return None

        if version != PROTOCOL_VERSION:
            print(f"Unsupported protocol version: {version}")
            return None

        if ptype != PACKET_TYPE_FC_COMMAND:
            print(f"Unexpected packet type: {ptype}")
            return None

        if len(data) < HEADER_SIZE + length + 4:  # header + body + crc
            return None

        # Extract body and CRC
        body = data[HEADER_SIZE:HEADER_SIZE+length]
        crc_received = struct.unpack('<I', data[HEADER_SIZE+length:HEADER_SIZE+length+4])[0]

        # Verify CRC
        crc_calculated = zlib.crc32(data[:HEADER_SIZE+length]) & 0xFFFFFFFF
        if crc_received != crc_calculated:
            print(f"CRC mismatch: received {crc_received:#x}, calculated {crc_calculated:#x}")
            return None

        # Parse body
        timestamp = struct.unpack('<Q', body[:8])[0]
        cmd_type_bytes = body[8:16].rstrip(b'\x00')
        command_type = cmd_type_bytes.decode('utf-8', errors='ignore')
        cmd_data_len = struct.unpack('<H', body[16:18])[0]
        command_data = body[18:18+cmd_data_len]
        validity_window = struct.unpack('<Q', body[18+cmd_data_len:26+cmd_data_len])[0]

        return FcCommandPacket(
            timestamp=timestamp,
            command_type=command_type,
            command_data=command_data,
            validity_window=validity_window,
            sequence=seq
        )

    def encode_health(self, packet: HealthPacket) -> bytes:
        """Encode health packet to bytes"""
        # Body: timestamp(8) + heartbeat(4) + uptime(8) + watchdog_flags(4)
        body = struct.pack('<QIIQ',
                          packet.timestamp,
                          packet.heartbeat,
                          packet.process_uptime,
                          packet.watchdog_flags)

        # Update length in header first
        length = len(body)

        # Header: magic(4) + version(2) + type(2) + length(4) + seq(4)
        header = struct.pack(HEADER_FMT,
                            STATE_PACKET_MAGIC,
                            PROTOCOL_VERSION,
                            PACKET_TYPE_HEALTH,
                            length,
                            packet.sequence)

        # CRC32 checksum (calculate with final header)
        crc = zlib.crc32(header + body) & 0xFFFFFFFF

        return header + body + struct.pack('<I', crc)

    def decode_health(self, data: bytes) -> Optional[HealthPacket]:
        """Decode health packet from bytes"""
        if len(data) < HEADER_SIZE:  # Minimum header size
            return None

        # Parse header
        magic, version, ptype, length, seq = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])

        if magic != STATE_PACKET_MAGIC:
            print(f"Invalid magic number: {magic:#x}")
            return None

        if version != PROTOCOL_VERSION:
            print(f"Unsupported protocol version: {version}")
            return None

        if ptype != PACKET_TYPE_HEALTH:
            print(f"Unexpected packet type: {ptype}")
            return None

        if len(data) < HEADER_SIZE + length + 4:  # header + body + crc
            return None

        # Extract body and CRC
        body = data[HEADER_SIZE:HEADER_SIZE+length]
        crc_received = struct.unpack('<I', data[HEADER_SIZE+length:HEADER_SIZE+length+4])[0]

        # Verify CRC
        crc_calculated = zlib.crc32(data[:HEADER_SIZE+length]) & 0xFFFFFFFF
        if crc_received != crc_calculated:
            print(f"CRC mismatch: received {crc_received:#x}, calculated {crc_calculated:#x}")
            return None

        # Parse body
        timestamp, heartbeat, uptime, watchdog_flags = struct.unpack('<QIIQ', body)

        return HealthPacket(
            timestamp=timestamp,
            heartbeat=heartbeat,
            process_uptime=uptime,
            watchdog_flags=watchdog_flags,
            sequence=seq
        )


# Convenience functions for creating packets
def create_truth_nav_packet(position: np.ndarray, velocity: np.ndarray,
                           attitude: np.ndarray, rate: np.ndarray) -> TruthNavPacket:
    """Create a truth navigation packet with current timestamp"""
    import time
    timestamp = int(time.time() * 1e9)  # Convert to nanoseconds
    position = np.asarray(position, dtype=np.float64)
    velocity = np.asarray(velocity, dtype=np.float64)
    attitude = np.asarray(attitude, dtype=np.float64)
    rate = np.asarray(rate, dtype=np.float64)
    return TruthNavPacket(
        timestamp=timestamp,
        position=position,
        velocity=velocity,
        attitude=attitude,
        rate=rate,
        sequence=0  # Will be assigned by codec
    )


def create_fc_command_packet(command_type: str, command_data: bytes = b'',
                           validity_window: int = 1000000000) -> FcCommandPacket:
    """Create a flight computer command packet with current timestamp"""
    import time
    timestamp = int(time.time() * 1e9)  # Convert to nanoseconds
    return FcCommandPacket(
        timestamp=timestamp,
        command_type=command_type,
        command_data=command_data,
        validity_window=validity_window,
        sequence=0  # Will be assigned by codec
    )


def create_health_packet(heartbeat: int, process_uptime: int,
                        watchdog_flags: int = 0) -> HealthPacket:
    """Create a health packet with current timestamp"""
    import time
    timestamp = int(time.time() * 1e9)  # Convert to nanoseconds
    return HealthPacket(
        timestamp=timestamp,
        heartbeat=heartbeat,
        process_uptime=process_uptime,
        watchdog_flags=watchdog_flags,
        sequence=0  # Will be assigned by codec
    )