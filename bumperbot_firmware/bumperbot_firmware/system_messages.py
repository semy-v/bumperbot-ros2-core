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
    right_feedforward_ks: float
    right_feedforward_kv: float
    right_feedback_kp: float
    right_feedback_ki: float
    right_feedback_kd: float
    right_max_feedback_pwm: int

    left_feedforward_ks: float
    left_feedforward_kv: float
    left_feedback_kp: float
    left_feedback_ki: float
    left_feedback_kd: float
    left_max_feedback_pwm: int

    control_rate_hz: int

    start_byte: int = START_BYTE
    msg_id: MsgId = MsgId.DiffDriveConfig

    # Packed C++ payload:
    #   2 * WheelConfig(5 * float + uint16) + uint16 control_rate_hz
    #   = 2 * 22 + 2 = 46 payload bytes; header is 5 bytes.
    PAYLOAD_FORMAT = "<fffffHfffffHH"
    STRUCT_FORMAT = "<BBBHfffffHfffffHH"
    EXPECTED_SIZE = struct.calcsize(STRUCT_FORMAT)

    def serialize(self) -> bytes:
        payload_values = (
            self.right_feedforward_ks,
            self.right_feedforward_kv,
            self.right_feedback_kp,
            self.right_feedback_ki,
            self.right_feedback_kd,
            self.right_max_feedback_pwm,
            self.left_feedforward_ks,
            self.left_feedforward_kv,
            self.left_feedback_kp,
            self.left_feedback_ki,
            self.left_feedback_kd,
            self.left_max_feedback_pwm,
            self.control_rate_hz,
        )
        payload = struct.pack(self.PAYLOAD_FORMAT, *payload_values)
        header_no_crc = struct.pack("<BBB", self.start_byte, self.msg_id, len(payload))
        crc = calculate_crc16(header_no_crc + payload)

        return struct.pack(
            self.STRUCT_FORMAT,
            self.start_byte,
            self.msg_id,
            len(payload),
            crc,
            *payload_values,
        )

    @classmethod
    def deserialize(cls, data: bytes):
        if len(data) != cls.EXPECTED_SIZE:
            raise ValueError(
                f"DiffDriveConfigMsg length mismatch: expected {cls.EXPECTED_SIZE}, got {len(data)}"
            )

        unpacked = struct.unpack(cls.STRUCT_FORMAT, data)
        return cls(
            right_feedforward_ks=unpacked[4],
            right_feedforward_kv=unpacked[5],
            right_feedback_kp=unpacked[6],
            right_feedback_ki=unpacked[7],
            right_feedback_kd=unpacked[8],
            right_max_feedback_pwm=unpacked[9],
            left_feedforward_ks=unpacked[10],
            left_feedforward_kv=unpacked[11],
            left_feedback_kp=unpacked[12],
            left_feedback_ki=unpacked[13],
            left_feedback_kd=unpacked[14],
            left_max_feedback_pwm=unpacked[15],
            control_rate_hz=unpacked[16],
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
    DiffDriveConfigMsg.EXPECTED_SIZE == 51
), f"DiffDriveConfigMsg expected 51, got {DiffDriveConfigMsg.EXPECTED_SIZE}"
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