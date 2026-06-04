#! /usr/bin/env python3

import struct
import operator
from dataclasses import dataclass
from functools import reduce
from enum import IntEnum

START_BYTE = 0xAA

def calculate_lrc8(data: bytes) -> int:
    """Calculates the XOR Checksum (LRC8) of a byte sequence."""
    if not data:
        return 0
    return reduce(operator.xor, data, 0)

class MsgId(IntEnum):
    Config = 0x00
    Velocity = 0x01
    Deactivate = 0x02


@dataclass
class ConfigMsg:
    pid_rate: float
    r_wheel_kp: float
    r_wheel_ki: float
    r_wheel_kd: float
    r_wheel_pwm_deadband: int
    l_wheel_kp: float
    l_wheel_ki: float
    l_wheel_kd: float
    l_wheel_pwm_deadband: int
    
    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.Config

    # Header(4) + float(4) + 3*float(12) + uint8(1) + 3*float(12) + uint8(1) = 34 bytes total
    STRUCT_FORMAT = "<BBBBffffBfffB"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack(
            "<ffffBfffB", 
            self.pid_rate,
            self.r_wheel_kp, self.r_wheel_ki, self.r_wheel_kd, self.r_wheel_pwm_deadband,
            self.l_wheel_kp, self.l_wheel_ki, self.l_wheel_kd, self.l_wheel_pwm_deadband
        )
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_lrc8(header_no_crc) ^ calculate_lrc8(payload)
        
        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte, self.msg_id, len(payload), crc,
            self.pid_rate,
            self.r_wheel_kp, self.r_wheel_ki, self.r_wheel_kd, self.r_wheel_pwm_deadband,
            self.l_wheel_kp, self.l_wheel_ki, self.l_wheel_kd, self.l_wheel_pwm_deadband
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(f"ConfigMsg length mismatch. Expected {cls.EXPECTED_SIZE}, got {len(data)}")
        
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(
            pid_rate=unpacked[4],
            r_wheel_kp=unpacked[5], r_wheel_ki=unpacked[6], r_wheel_kd=unpacked[7], r_wheel_pwm_deadband=unpacked[8],
            l_wheel_kp=unpacked[9], l_wheel_ki=unpacked[10], l_wheel_kd=unpacked[11], l_wheel_pwm_deadband=unpacked[12]
        )

@dataclass
class VelocityMsg:
    right_wheel_velocity: float
    left_wheel_velocity: float
    
    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.Velocity

    # Header(4) + 2 floats(8) = 12 bytes total
    STRUCT_FORMAT = "<BBBBff"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack("<ff", self.right_wheel_velocity, self.left_wheel_velocity)
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_lrc8(header_no_crc) ^ calculate_lrc8(payload)
        
        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte, self.msg_id, len(payload), crc,
            self.right_wheel_velocity, self.left_wheel_velocity
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(f"VelocityMsg length mismatch. Expected {cls.EXPECTED_SIZE}, got {len(data)}")
        
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(right_wheel_velocity=unpacked[4], left_wheel_velocity=unpacked[5])

@dataclass
class DeactivateMsg:
    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.Deactivate

    # Header(4) + 0 payload bytes = 4 bytes total
    STRUCT_FORMAT = "<BBBB"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, 0)
        crc = calculate_lrc8(header_no_crc)
        return struct.pack(self.STRUCT_FORMAT, self.start_byte, self.msg_id, 0, crc)


# Sanity Checks matching C++ static_asserts
assert ConfigMsg.EXPECTED_SIZE == 34, f"ConfigMsg format size must be 34 bytes! (Got {ConfigMsg.EXPECTED_SIZE})"
assert VelocityMsg.EXPECTED_SIZE == 12, "VelocityMsg format size must be 12 bytes!"
assert DeactivateMsg.EXPECTED_SIZE == 4, "DeactivateMsg format size must be 4 bytes!"