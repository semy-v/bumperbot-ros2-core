#! /usr/bin/env python3

import struct
from dataclasses import dataclass
from enum import IntEnum, IntFlag

START_BYTE = 0xAA


def _generate_crc16_table() -> list[int]:
    """Generates the 256-element CRC16-CCITT lookup table (polynomial 0x1021),

    matching the C++ compile-time consteval generateCrcTable().
    """
    table = []
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table.append(crc)
    return table


CRC16_TABLE = _generate_crc16_table()


def calculate_crc16(data: bytes, initial_crc: int = 0xFFFF) -> int:
    """Calculates CRC16-CCITT over a byte sequence matching calculateCRC16()

    and crcByte() from message_traits.hpp.
    """
    crc = initial_crc
    for byte in data:
        index = ((crc >> 8) ^ byte) & 0xFF
        crc = ((crc << 8) ^ CRC16_TABLE[index]) & 0xFFFF
    return crc


class MsgId(IntEnum):
    DiffDriveConfig = 0x00
    ImuConfig = 0x01
    DiffDriveCommand = 0x02
    SystemState = 0x03
    Deactivate = 0x04
    End = 0x05


class SystemStateFlags(IntFlag):
    None_ = 0x00
    ImuUnavailable = 1 << 0


# --- Data Structures Matching C++ system_data.hpp ---


@dataclass
class ImuStateData:
    angular_velocity_x: float
    angular_velocity_y: float
    angular_velocity_z: float
    linear_acceleration_x: float
    linear_acceleration_y: float
    linear_acceleration_z: float


@dataclass
class DiffDriveVelocityData:
    right_wheel_velocity: float
    left_wheel_velocity: float


# --- Wire Protocol Message Definitions ---


@dataclass
class DiffDriveConfigMsg:
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
    msg_id: MsgId = MsgId.DiffDriveConfig

    # Header(5) + double(8) + 3*double(24) + uint8(1) + 3*double(24) + uint8(1) = 63 bytes total
    STRUCT_FORMAT = "<BBBHddddBdddB"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack(
            "<ddddBdddB",
            self.pid_rate,
            self.r_wheel_kp,
            self.r_wheel_ki,
            self.r_wheel_kd,
            self.r_wheel_pwm_deadband,
            self.l_wheel_kp,
            self.l_wheel_ki,
            self.l_wheel_kd,
            self.l_wheel_pwm_deadband,
        )
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_crc16(header_no_crc + payload)

        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte,
            self.msg_id,
            len(payload),
            crc,
            self.pid_rate,
            self.r_wheel_kp,
            self.r_wheel_ki,
            self.r_wheel_kd,
            self.r_wheel_pwm_deadband,
            self.l_wheel_kp,
            self.l_wheel_ki,
            self.l_wheel_kd,
            self.l_wheel_pwm_deadband,
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"DiffDriveConfigMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(
            pid_rate=unpacked[4],
            r_wheel_kp=unpacked[5],
            r_wheel_ki=unpacked[6],
            r_wheel_kd=unpacked[7],
            r_wheel_pwm_deadband=unpacked[8],
            l_wheel_kp=unpacked[9],
            l_wheel_ki=unpacked[10],
            l_wheel_kd=unpacked[11],
            l_wheel_pwm_deadband=unpacked[12],
        )


@dataclass
class ImuConfigMsg:
    calibrate_period_ms: int
    result: bool

    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.ImuConfig

    # Header(5) + uint16(2) + bool/uint8(1) = 8 bytes total
    STRUCT_FORMAT = "<BBBHH?"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack("<H?", self.calibrate_period_ms, self.result)
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_crc16(header_no_crc + payload)

        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte,
            self.msg_id,
            len(payload),
            crc,
            self.calibrate_period_ms,
            self.result,
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"ImuConfigMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(calibrate_period_ms=unpacked[4], result=unpacked[5])


@dataclass
class DiffDriveCommandMsg:
    right_wheel_velocity: float
    left_wheel_velocity: float
    response_delay_ms: int

    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.DiffDriveCommand

    # Header(5) + 2 float(8) + uint8(1) = 14 bytes total
    STRUCT_FORMAT = "<BBBHffB"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack(
            "<ffB",
            self.right_wheel_velocity,
            self.left_wheel_velocity,
            self.response_delay_ms,
        )
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_crc16(header_no_crc + payload)

        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte,
            self.msg_id,
            len(payload),
            crc,
            self.right_wheel_velocity,
            self.left_wheel_velocity,
            self.response_delay_ms,
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"DiffDriveCommandMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(
            right_wheel_velocity=unpacked[4],
            left_wheel_velocity=unpacked[5],
            response_delay_ms=unpacked[6],
        )


@dataclass
class SystemStateMsg:
    status: int
    imu: ImuStateData
    diff_drive: DiffDriveVelocityData

    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.SystemState

    # Header(5) + uint8(1) + 6 float(24) + 2 float(8) = 38 bytes total
    STRUCT_FORMAT = "<BBBHBffffffff"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload = struct.pack(
            "<Bffffffff",
            self.status,
            self.imu.angular_velocity_x,
            self.imu.angular_velocity_y,
            self.imu.angular_velocity_z,
            self.imu.linear_acceleration_x,
            self.imu.linear_acceleration_y,
            self.imu.linear_acceleration_z,
            self.diff_drive.right_wheel_velocity,
            self.diff_drive.left_wheel_velocity,
        )
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_crc16(header_no_crc + payload)

        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte,
            self.msg_id,
            len(payload),
            crc,
            self.status,
            self.imu.angular_velocity_x,
            self.imu.angular_velocity_y,
            self.imu.angular_velocity_z,
            self.imu.linear_acceleration_x,
            self.imu.linear_acceleration_y,
            self.imu.linear_acceleration_z,
            self.diff_drive.right_wheel_velocity,
            self.diff_drive.left_wheel_velocity,
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"SystemStateMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )
        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)

        imu_data = ImuStateData(
            angular_velocity_x=unpacked[5],
            angular_velocity_y=unpacked[6],
            angular_velocity_z=unpacked[7],
            linear_acceleration_x=unpacked[8],
            linear_acceleration_y=unpacked[9],
            linear_acceleration_z=unpacked[10],
        )
        diff_drive_data = DiffDriveVelocityData(
            right_wheel_velocity=unpacked[11],
            left_wheel_velocity=unpacked[12],
        )
        return cls(status=unpacked[4], imu=imu_data, diff_drive=diff_drive_data)


@dataclass
class DeactivateMsg:
    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.Deactivate

    # Header(5) + 0 payload bytes = 5 bytes total
    STRUCT_FORMAT = "<BBBH"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, 0)
        crc = calculate_crc16(header_no_crc)
        return struct.pack(self.STRUCT_FORMAT, self.start_byte, self.msg_id, 0, crc)

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"DeactivateMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )
        return cls()


# Registry mapping MsgId to Deserialization Methods
MESSAGE_DESERIALIZERS = {
    MsgId.DiffDriveConfig: DiffDriveConfigMsg.deserialize,
    MsgId.ImuConfig: ImuConfigMsg.deserialize,
    MsgId.DiffDriveCommand: DiffDriveCommandMsg.deserialize,
    MsgId.SystemState: SystemStateMsg.deserialize,
    MsgId.Deactivate: DeactivateMsg.deserialize,
}

# --- Sanity Checks matching C++ static_asserts + 5-byte Header ---
assert (
    DiffDriveConfigMsg.EXPECTED_SIZE == 63
), f"DiffDriveConfigMsg expected 63, got {DiffDriveConfigMsg.EXPECTED_SIZE}"
assert (
    ImuConfigMsg.EXPECTED_SIZE == 8
), f"ImuConfigMsg expected 8, got {ImuConfigMsg.EXPECTED_SIZE}"
assert (
    DiffDriveCommandMsg.EXPECTED_SIZE == 14
), f"DiffDriveCommandMsg expected 14, got {DiffDriveCommandMsg.EXPECTED_SIZE}"
assert (
    SystemStateMsg.EXPECTED_SIZE == 38
), f"SystemStateMsg expected 38, got {SystemStateMsg.EXPECTED_SIZE}"
assert (
    DeactivateMsg.EXPECTED_SIZE == 5
), f"DeactivateMsg expected 5, got {DeactivateMsg.EXPECTED_SIZE}"
