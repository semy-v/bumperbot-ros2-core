#!/usr/bin/env python3

import time
import smbus
import math
from dataclasses import dataclass

# ============================================================
# MPU6050 REGISTER MAP
# ============================================================

PWR_MGMT_1_REG = 0x6B
SMPLRT_DIV_REG = 0x19
CONFIG_REG = 0x1A
ACCEL_CONFIG_REG = 0x1C
GYRO_CONFIG_REG = 0x1B
ACCEL_XOUT_H_REG = 0x3B

DEVICE_ADDRESS = 0x68

# ============================================================
# MPU6050 PHYSICAL & CONVERSION CONSTANTS
# ============================================================
G_TO_MS2 = 9.80665
DEG_TO_RAD = math.pi / 180.0

# ============================================================
# SENSOR CONFIGURATION MAPS (LSB sensitivity per unit)
# ============================================================
# Maps bit-value to (LSB/g)
ACCEL_FS_MAP = {
    0: 16384.0, # ±2g
    1: 8192.0,  # ±4g
    2: 4096.0,  # ±8g
    3: 2048.0   # ±16g
}

# Maps bit-value to (LSB / deg/s)
GYRO_FS_MAP = {
    0: 131.0,   # ±250 deg/s
    1: 65.5,    # ±500 deg/s
    2: 32.8,    # ±1000 deg/s
    3: 16.4     # ±2000 deg/s
}

# ============================================================
# ACTIVE SETTINGS (Change values here)
# ============================================================
# Choose index 0-3 based on the maps above
ACCEL_RANGE_SEL = 3  # ±16g
GYRO_RANGE_SEL  = 3  # ±2000 deg/s

# Disable temperature sensor: 0=Enabled, 1=Disabled
TEMP_DIS = 1

# Clock Source: 0=Internal, 1=X-Gyro, 2=Y-Gyro, 3=Z-Gyro
CLK_SEL = 3 
DLPF_CFG = 4    # Low-pass filter ~20Hz bandwidth
SMPLRT_DIV = 9  # 100Hz Sample Rate

# ============================================================
# DERIVED REGISTER VALUES & COEFFICIENTS
# ============================================================
# Register Values
PWR_MGMT_1_VALUE   = (TEMP_DIS << 3) | CLK_SEL   # Bit 6 (Sleep) is 0 by default
ACCEL_CONFIG_VALUE = (ACCEL_RANGE_SEL << 3)
GYRO_CONFIG_VALUE  = (GYRO_RANGE_SEL << 3)

# Conversion Factors: Target_Unit = Raw_Data / COEF
# Resulting units: m/s^2
ACCEL_COEF = ACCEL_FS_MAP[ACCEL_RANGE_SEL] / G_TO_MS2
# Resulting units: rad/s
GYRO_COEF  = GYRO_FS_MAP[GYRO_RANGE_SEL] / DEG_TO_RAD


# ============================================================
# DATA TYPES
# ============================================================


@dataclass(slots=True)
class IMUData:
    accel_x: float
    accel_y: float
    accel_z: float

    gyro_x: float
    gyro_y: float
    gyro_z: float


@dataclass(slots=True)
class IMUCalibration:
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0

    gyro_x: float = 0.0
    gyro_y: float = 0.0
    gyro_z: float = 0.0


# ============================================================
# MPU6050 DRIVER
# ============================================================

class MPU6050:
    def __init__(self, bus_id: int = 1, device_address: int = DEVICE_ADDRESS):
        self._bus_id = bus_id
        self._device_address = device_address

        self._bus = None
        self._is_connected = False

        self._calibration = IMUCalibration()

    # ========================================================
    # PUBLIC API
    # ========================================================

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def connect(self) -> None:
        self._bus = smbus.SMBus(self._bus_id)

        # Wake up and set clock source
        self._write_register(PWR_MGMT_1_REG, PWR_MGMT_1_VALUE)
        
        # Set sample rate and filter
        self._write_register(SMPLRT_DIV_REG, SMPLRT_DIV)
        self._write_register(CONFIG_REG, DLPF_CFG)

        # Configure Full Scale Ranges
        self._write_register(ACCEL_CONFIG_REG, ACCEL_CONFIG_VALUE)
        self._write_register(GYRO_CONFIG_REG, GYRO_CONFIG_VALUE)

        self._is_connected = True

    def disconnect(self) -> None:
        if self._bus is not None:
            self._bus.close()
            self._bus = None

        self._is_connected = False

    def calibrate(
        self,
        sample_num: int = 200,
        sample_interval_ms: int = 5,
    ) -> IMUCalibration:

        sum_ax = 0.0
        sum_ay = 0.0
        sum_az = 0.0

        sum_gx = 0.0
        sum_gy = 0.0
        sum_gz = 0.0

        for _ in range(sample_num):
            data = self.read()

            sum_ax += data.accel_x
            sum_ay += data.accel_y
            sum_az += data.accel_z

            sum_gx += data.gyro_x
            sum_gy += data.gyro_y
            sum_gz += data.gyro_z

            time.sleep(sample_interval_ms/1000.0)

        self._calibration = IMUCalibration(
            accel_x=sum_ax / sample_num,
            accel_y=sum_ay / sample_num,
            accel_z=(sum_az / sample_num) - G_TO_MS2,
            gyro_x=sum_gx / sample_num,
            gyro_y=sum_gy / sample_num,
            gyro_z=sum_gz / sample_num,
        )

        return self._calibration

    def read(self) -> IMUData:
        raw_accel, raw_gyro = self._read_raw()

        return IMUData(
            accel_x=raw_accel[0] / ACCEL_COEF,
            accel_y=raw_accel[1] / ACCEL_COEF,
            accel_z=raw_accel[2] / ACCEL_COEF,
            gyro_x=raw_gyro[0] / GYRO_COEF,
            gyro_y=raw_gyro[1] / GYRO_COEF,
            gyro_z=raw_gyro[2] / GYRO_COEF,
        )

    def read_calibrated(self) -> IMUData:
        data = self.read()

        return IMUData(
            accel_x=data.accel_x - self._calibration.accel_x,
            accel_y=data.accel_y - self._calibration.accel_y,
            accel_z=data.accel_z - self._calibration.accel_z,
            gyro_x=data.gyro_x - self._calibration.gyro_x,
            gyro_y=data.gyro_y - self._calibration.gyro_y,
            gyro_z=data.gyro_z - self._calibration.gyro_z,
        )

    # ========================================================
    # INTERNALS
    # ========================================================

    def _write_register(self, reg: int, value: int) -> None:
        self._bus.write_byte_data(
            self._device_address,
            reg,
            value,
        )

    @staticmethod
    def _to_signed(high: int, low: int) -> int:
        value = (high << 8) | low
        return value - 65536 if value & 0x8000 else value

    def _read_raw(self):

        #
        # Single I2C transaction
        #
        data = self._bus.read_i2c_block_data(
            self._device_address,
            ACCEL_XOUT_H_REG,
            14,
        )

        raw_accel = (
            self._to_signed(data[0], data[1]),
            self._to_signed(data[2], data[3]),
            self._to_signed(data[4], data[5]),
        )

        raw_gyro = (
            self._to_signed(data[8], data[9]),
            self._to_signed(data[10], data[11]),
            self._to_signed(data[12], data[13]),
        )

        return raw_accel, raw_gyro
