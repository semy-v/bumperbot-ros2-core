#ifndef MPU6050_TYPES_HPP
#define MPU6050_TYPES_HPP

#include <array>
#include <cstdint>
#include <numbers>

namespace mpu6050 {

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

constexpr uint8_t kDeviceAddress = 0x68;
constexpr uint8_t kWhoAmIReg = 0x75;
constexpr uint8_t kWhoAmIValue = 0x68;
constexpr uint8_t kPwrMgmt1Reg = 0x6B;
constexpr uint8_t kSmplrtDivReg = 0x19;
constexpr uint8_t kConfigReg = 0x1A;
constexpr uint8_t kGyroConfigReg = 0x1B;
constexpr uint8_t kAccelConfigReg = 0x1C;
constexpr uint8_t kIntPinCfgReg = 0x37;
constexpr uint8_t kIntEnableReg = 0x38;
constexpr uint8_t kAccelXoutHReg = 0x3B;

// Production DATA_RDY interrupt configuration:
//   INT_LEVEL    = 0 -> active HIGH
//   INT_OPEN     = 0 -> push-pull
//   LATCH_INT_EN = 0 -> short pulse for every DATA_RDY event
//
// Pulsed mode is intentional here. A transient I2C read failure cannot leave
// INT latched high and prevent future RISING edges; the next sensor sample will
// generate another interrupt. The SensorReadTask only needs the latest sample,
// so missing an intermediate sample does not corrupt state.
constexpr uint8_t kDataReadyIntPinCfg = 0U;
constexpr uint8_t kDataReadyIntEnable = 1U << 0;
constexpr uint8_t kInterruptsDisabled = 0U;

constexpr float kGToMs2 = 9.80665f;
constexpr float kDegToRad = std::numbers::pi_v<float> / 180.0f;

constexpr std::array kAccelFsMap = {16384.0f, 8192.0f, 4096.0f, 2048.0f};
constexpr std::array kGyroFsMap = {131.0f, 65.5f, 32.8f, 16.4f};

constexpr uint8_t kAccelRangeSel = 3;
constexpr uint8_t kGyroRangeSel = 3;
constexpr uint8_t kTempDis = 1;
constexpr uint8_t kClkSel = 3;
constexpr uint8_t kDlpfCfg = 4;

// DLPF enabled -> 1 kHz internal sample rate. Divider 9 -> 100 Hz DATA_RDY.
constexpr uint8_t kSmplrtDiv = 9;
constexpr uint16_t kSampleRateHz = 100;

constexpr uint8_t kPwrMgmt1Value = (kTempDis << 3) | kClkSel;
constexpr uint8_t kAccelConfigValue = (kAccelRangeSel << 3);
constexpr uint8_t kGyroConfigValue = (kGyroRangeSel << 3);

constexpr float kAccelCoef = kAccelFsMap[kAccelRangeSel] / kGToMs2;
constexpr float kGyroCoef = kGyroFsMap[kGyroRangeSel] / kDegToRad;

}  // namespace mpu6050

#endif  // MPU6050_TYPES_HPP