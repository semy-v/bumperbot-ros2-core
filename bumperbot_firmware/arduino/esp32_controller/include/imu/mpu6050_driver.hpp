#ifndef MPU6050_DRIVER_HPP
#define MPU6050_DRIVER_HPP

#include <optional>

#include "mpu6050_types.hpp"
#include "wire_i2c_bus.hpp"

// ============================================================
// MPU6050 DRIVER
// ============================================================

typedef void (*mpu6050_delay_function)(unsigned long ms);

template <I2CBusConcept I2CBus>
class MPU6050 {
 public:
  explicit MPU6050(I2CBus bus, mpu6050_delay_function delay_func,
                   uint8_t device_address = mpu6050::kDeviceAddress)
      : bus_(std::move(bus)),
        address_(device_address),
        delay_func_(delay_func) {}

  [[nodiscard]] bool isConnected() const { return connected_; }

  void disconnect() { connected_ = false; }

  bool connect() {
    if (isConnected()) {
      return true;  // Already connected
    }

    // Physical ping verification via WHO_AM_I register check
    std::array<uint8_t, 1> identityToken{0};
    if (!bus_.readBlock(address_, mpu6050::kWhoAmIReg, identityToken) ||
        identityToken[0] != mpu6050::kWhoAmIValue) {
      connected_ = false;
      return false;
    }

    connected_ =
        writeRegister(mpu6050::kPwrMgmt1Reg, mpu6050::kPwrMgmt1Value) &&
        writeRegister(mpu6050::kSmplrtDivReg, mpu6050::kSmplrtDiv) &&
        writeRegister(mpu6050::kConfigReg, mpu6050::kDlpfCfg) &&
        writeRegister(mpu6050::kAccelConfigReg, mpu6050::kAccelConfigValue) &&
        writeRegister(mpu6050::kGyroConfigReg, mpu6050::kGyroConfigValue);

    return connected_;
  }

  std::optional<mpu6050::IMUCalibration> calibrate(uint16_t sampleNum = 200,
                                                   uint16_t sampleIntervalMs = 10) {
    float sumAx = 0.0f, sumAy = 0.0f, sumAz = 0.0f;
    float sumGx = 0.0f, sumGy = 0.0f, sumGz = 0.0f;

    for (decltype(sampleNum) i = 0; i < sampleNum; ++i) {
      const auto dataOpt = read();
      if (!dataOpt.has_value()) {
        return std::nullopt;  // Return empty optional if reading fails
      }

      const auto& data = dataOpt.value();

      sumAx += data.accelX;
      sumAy += data.accelY;
      sumAz += data.accelZ;

      sumGx += data.gyroX;
      sumGy += data.gyroY;
      sumGz += data.gyroZ;

      delay_func_(sampleIntervalMs);
    }

    calibration_ = mpu6050::IMUCalibration{
        .accelX = sumAx / static_cast<float>(sampleNum),
        .accelY = sumAy / static_cast<float>(sampleNum),
        .accelZ = (sumAz / static_cast<float>(sampleNum)) - mpu6050::kGToMs2,
        .gyroX = sumGx / static_cast<float>(sampleNum),
        .gyroY = sumGy / static_cast<float>(sampleNum),
        .gyroZ = sumGz / static_cast<float>(sampleNum)};

    return calibration_;
  }

  std::optional<mpu6050::IMUData> read() {
    if (!isConnected()) {
      return std::nullopt;
    }

    const auto rawOpt = readRaw();
    if (!rawOpt.has_value()) {
      return std::nullopt;
    }

    const auto& raw = rawOpt.value();

    return mpu6050::IMUData{
        .accelX = static_cast<float>(raw.accelX) / mpu6050::kAccelCoef,
        .accelY = static_cast<float>(raw.accelY) / mpu6050::kAccelCoef,
        .accelZ = static_cast<float>(raw.accelZ) / mpu6050::kAccelCoef,
        .gyroX = static_cast<float>(raw.gyroX) / mpu6050::kGyroCoef,
        .gyroY = static_cast<float>(raw.gyroY) / mpu6050::kGyroCoef,
        .gyroZ = static_cast<float>(raw.gyroZ) / mpu6050::kGyroCoef};
  }

  std::optional<mpu6050::IMUData> readCalibrated() {
    const auto dataOpt = read();
    if (!dataOpt.has_value()) {
      return std::nullopt;
    }

    const auto& data = dataOpt.value();
    return mpu6050::IMUData{.accelX = data.accelX - calibration_.accelX,
                            .accelY = data.accelY - calibration_.accelY,
                            .accelZ = data.accelZ - calibration_.accelZ,
                            .gyroX = data.gyroX - calibration_.gyroX,
                            .gyroY = data.gyroY - calibration_.gyroY,
                            .gyroZ = data.gyroZ - calibration_.gyroZ};
  }

 private:
  I2CBus bus_;
  uint8_t address_;
  mpu6050_delay_function delay_func_;
  mpu6050::IMUCalibration calibration_{};
  bool connected_{false};

  bool writeRegister(uint8_t reg, uint8_t value) {
    return bus_.writeByte(address_, reg, value);
  }

  static int16_t toSigned(uint8_t high, uint8_t low) {
    return static_cast<int16_t>((high << 8) | low);
  }

  std::optional<mpu6050::RawIMUData> readRaw() {
    std::array<uint8_t, 14> buffer{};

    // If the read fails (e.g. loose wire),
    // return null optional to indicate failure
    if (!bus_.readBlock(address_, mpu6050::kAccelXoutHReg, buffer)) {
      return std::nullopt;
    }

    return mpu6050::RawIMUData{.accelX = toSigned(buffer[0], buffer[1]),
                               .accelY = toSigned(buffer[2], buffer[3]),
                               .accelZ = toSigned(buffer[4], buffer[5]),
                               // Data bytes 6 and 7 (buffer[6], buffer[7]) are
                               // Temperature data; skip them.
                               .gyroX = toSigned(buffer[8], buffer[9]),
                               .gyroY = toSigned(buffer[10], buffer[11]),
                               .gyroZ = toSigned(buffer[12], buffer[13])};
  }
};

#endif  // MPU6050_DRIVER_HPP