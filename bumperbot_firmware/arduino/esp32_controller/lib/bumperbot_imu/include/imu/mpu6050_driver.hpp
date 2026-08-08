#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

#include <array>
#include <cstdint>
#include <optional>
#include <utility>

#include "mpu6050_types.hpp"
#include "wire_i2c_bus.hpp"

typedef void (*mpu6050_delay_function)(unsigned long ms);

template <I2CBusConcept I2CBus>
class MPU6050 {
 public:
    explicit MPU6050(I2CBus bus,
                     mpu6050_delay_function delay_func,
                     uint8_t device_address = mpu6050::kDeviceAddress)
        : bus_(std::move(bus)), address_(device_address), delay_func_(delay_func) {}

    [[nodiscard]] bool isConnected() const { return connected_; }

    void disconnect() { connected_ = false; }

    bool connect() {
        if (isConnected()) {
            return true;
        }

        std::array<uint8_t, 1> identity_token{0};
        if (!bus_.readBlock(address_, mpu6050::kWhoAmIReg, identity_token) ||
            identity_token[0] != mpu6050::kWhoAmIValue) {
            connected_ = false;
            return false;
        }

        // Keep DATA_RDY disabled throughout initialization and calibration.
        connected_ = writeRegister(mpu6050::kIntEnableReg, mpu6050::kInterruptsDisabled) &&
                     writeRegister(mpu6050::kPwrMgmt1Reg, mpu6050::kPwrMgmt1Value) &&
                     writeRegister(mpu6050::kSmplrtDivReg, mpu6050::kSmplrtDiv) &&
                     writeRegister(mpu6050::kConfigReg, mpu6050::kDlpfCfg) &&
                     writeRegister(mpu6050::kAccelConfigReg, mpu6050::kAccelConfigValue) &&
                     writeRegister(mpu6050::kGyroConfigReg, mpu6050::kGyroConfigValue);

        return connected_;
    }

    [[nodiscard]] bool enableDataReadyInterrupt() {
        if (!isConnected()) {
            return false;
        }

        return writeRegister(mpu6050::kIntPinCfgReg, mpu6050::kDataReadyIntPinCfg) &&
               writeRegister(mpu6050::kIntEnableReg, mpu6050::kDataReadyIntEnable);
    }

    [[nodiscard]] bool disableInterrupts() {
        return isConnected() && writeRegister(mpu6050::kIntEnableReg, mpu6050::kInterruptsDisabled);
    }

    std::optional<mpu6050::IMUCalibration> calibrate(uint16_t sample_num = 200,
                                                     uint16_t sample_interval_ms = 10) {
        if (sample_num == 0U) {
            return std::nullopt;
        }

        float sum_ax = 0.0f;
        float sum_ay = 0.0f;
        float sum_az = 0.0f;
        float sum_gx = 0.0f;
        float sum_gy = 0.0f;
        float sum_gz = 0.0f;

        for (uint16_t i = 0; i < sample_num; ++i) {
            const auto data_opt = read();
            if (!data_opt) {
                return std::nullopt;
            }

            const auto& data = *data_opt;
            sum_ax += data.accelX;
            sum_ay += data.accelY;
            sum_az += data.accelZ;
            sum_gx += data.gyroX;
            sum_gy += data.gyroY;
            sum_gz += data.gyroZ;

            delay_func_(sample_interval_ms);
        }

        const float sample_count = static_cast<float>(sample_num);
        calibration_ = mpu6050::IMUCalibration{
            .accelX = sum_ax / sample_count,
            .accelY = sum_ay / sample_count,
            .accelZ = (sum_az / sample_count) - mpu6050::kGToMs2,
            .gyroX = sum_gx / sample_count,
            .gyroY = sum_gy / sample_count,
            .gyroZ = sum_gz / sample_count,
        };

        return calibration_;
    }

    std::optional<mpu6050::IMUData> read() {
        if (!isConnected()) {
            return std::nullopt;
        }

        const auto raw_opt = readRaw();
        if (!raw_opt) {
            return std::nullopt;
        }

        const auto& raw = *raw_opt;
        return mpu6050::IMUData{
            .accelX = static_cast<float>(raw.accelX) / mpu6050::kAccelCoef,
            .accelY = static_cast<float>(raw.accelY) / mpu6050::kAccelCoef,
            .accelZ = static_cast<float>(raw.accelZ) / mpu6050::kAccelCoef,
            .gyroX = static_cast<float>(raw.gyroX) / mpu6050::kGyroCoef,
            .gyroY = static_cast<float>(raw.gyroY) / mpu6050::kGyroCoef,
            .gyroZ = static_cast<float>(raw.gyroZ) / mpu6050::kGyroCoef,
        };
    }

    std::optional<mpu6050::IMUData> readCalibrated() {
        const auto data_opt = read();
        if (!data_opt) {
            return std::nullopt;
        }

        const auto& data = *data_opt;
        return mpu6050::IMUData{
            .accelX = data.accelX - calibration_.accelX,
            .accelY = data.accelY - calibration_.accelY,
            .accelZ = data.accelZ - calibration_.accelZ,
            .gyroX = data.gyroX - calibration_.gyroX,
            .gyroY = data.gyroY - calibration_.gyroY,
            .gyroZ = data.gyroZ - calibration_.gyroZ,
        };
    }

 private:
    I2CBus bus_;
    uint8_t address_;
    mpu6050_delay_function delay_func_;
    mpu6050::IMUCalibration calibration_{};
    bool connected_{false};

    bool writeRegister(uint8_t reg, uint8_t value) { return bus_.writeByte(address_, reg, value); }

    static int16_t toSigned(uint8_t high, uint8_t low) {
        return static_cast<int16_t>((static_cast<uint16_t>(high) << 8U) | low);
    }

    std::optional<mpu6050::RawIMUData> readRaw() {
        std::array<uint8_t, 14> buffer{};
        if (!bus_.readBlock(address_, mpu6050::kAccelXoutHReg, buffer)) {
            return std::nullopt;
        }

        return mpu6050::RawIMUData{
            .accelX = toSigned(buffer[0], buffer[1]),
            .accelY = toSigned(buffer[2], buffer[3]),
            .accelZ = toSigned(buffer[4], buffer[5]),
            .gyroX = toSigned(buffer[8], buffer[9]),
            .gyroY = toSigned(buffer[10], buffer[11]),
            .gyroZ = toSigned(buffer[12], buffer[13]),
        };
    }
};

#endif  // MPU6050_DRIVER_HPP