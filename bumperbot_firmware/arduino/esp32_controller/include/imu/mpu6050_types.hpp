#ifndef MPU6050_TYPES_HPP
#define MPU6050_TYPES_HPP

#include <array>
#include <cstdint>
#include <numbers>

namespace mpu6050 {

// ============================================================
// DATA TYPES
// ============================================================

struct IMUData {
  float accelX{0.0f};
  float accelY{0.0f};
  float accelZ{0.0f};
  float gyroX{0.0f};
  float gyroY{0.0f};
  float gyroZ{0.0f};
};

struct IMUCalibration {
  float accelX{0.0f};
  float accelY{0.0f};
  float accelZ{0.0f};
  float gyroX{0.0f};
  float gyroY{0.0f};
  float gyroZ{0.0f};
};

struct RawIMUData {
  int16_t accelX{0};
  int16_t accelY{0};
  int16_t accelZ{0};
  int16_t gyroX{0};
  int16_t gyroY{0};
  int16_t gyroZ{0};
};

// ============================================================
// MPU6050 REGISTER MAP & CONSTANTS
// ============================================================

// Default I2C address for the MPU6050 (when AD0 pin is low).
constexpr uint8_t kDeviceAddress = 0x68;

// Who Am I Register: Used to verify device identity.
constexpr uint8_t kWhoAmIReg = 0x75;
constexpr uint8_t kWhoAmIValue = 0x68;  // Expected response from MPU6050

// Power Management Register 1: Controls sleep mode, reset, and clock source.
constexpr uint8_t kPwrMgmt1Reg = 0x6B;

// Sample Rate Divider: Determines the sample rate of the sensor output.
// Formula: Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
constexpr uint8_t kSmplrtDivReg = 0x19;

// Configuration Register: Configures the Digital Low Pass Filter (DLPF) and
// EXT_SYNC_SET.
constexpr uint8_t kConfigReg = 0x1A;

// Accelerometer Configuration: Sets the full-scale range (±2g, ±4g, ±8g, ±16g).
constexpr uint8_t kAccelConfigReg = 0x1C;

// Gyroscope Configuration: Sets the full-scale range (±250, 500, 1000, 2000
// deg/s).
constexpr uint8_t kGyroConfigReg = 0x1B;

// Starting register for accelerometer data (X-axis High byte).
// Reading 14 contiguous bytes from this register fetches Accel, Temp, and Gyro
// data.
constexpr uint8_t kAccelXoutHReg = 0x3B;

// Physics constants for conversions
constexpr float kGToMs2 = 9.80665f;
constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;

// ============================================================
// SENSOR CONFIGURATION MAPS (LSB sensitivity per unit)
// ============================================================
// Index mapping: 0=±2g, 1=±4g, 2=±8g, 3=±16g
constexpr std::array kAccelFsMap = {16384.0f, 8192.0f, 4096.0f,
                                              2048.0f};

// Index mapping: 0=±250dps, 1=±500dps, 2=±1000dps, 3=±2000dps
constexpr std::array kGyroFsMap = {131.0f, 65.5f, 32.8f, 16.4f};

// ============================================================
// ACTIVE SETTINGS
// ============================================================

constexpr uint8_t kAccelRangeSel = 3;  // Select ±16g (Index 3)
constexpr uint8_t kGyroRangeSel = 3;   // Select ±2000 deg/s (Index 3)

// Disables the temperature sensor (Bit 3 of PWR_MGMT_1) to save power.
constexpr uint8_t kTempDis = 1;

// Sets the clock source to the Z-axis gyro reference (Value 3),
// which is recommended by the datasheet for better stability than the internal
// clock.
constexpr uint8_t kClkSel = 3;

// Sets the Digital Low Pass Filter (DLPF) to ~20Hz bandwidth (Value 4).
// Smooths out high-frequency noise from vibrations.
constexpr uint8_t kDlpfCfg = 4;

// Divider of 9 with a 1kHz internal rate yields a 100Hz sample rate (1000 / (1
// + 9)).
constexpr uint8_t kSmplrtDiv = 9;

// ============================================================
// DERIVED REGISTER VALUES & COEFFICIENTS
// ============================================================

// Combines temperature disable bit and clock selection for a single register
// write. Note: Bit 6 (Sleep) defaults to 0, which wakes the device up.
constexpr uint8_t kPwrMgmt1Value = (kTempDis << 3) | kClkSel;
constexpr uint8_t kAccelConfigValue = (kAccelRangeSel << 3);
constexpr uint8_t kGyroConfigValue = (kGyroRangeSel << 3);

// Final scalar values used to convert raw 16-bit integers to physical units
// (m/s² and rad/s)
constexpr float kAccelCoef = kAccelFsMap[kAccelRangeSel] / kGToMs2;
constexpr float kGyroCoef = kGyroFsMap[kGyroRangeSel] / kDegToRad;

}  // namespace mpu6050

#endif  // MPU6050_TYPES_HPP