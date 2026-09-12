#include <Arduino.h>
#include <Wire.h>

#include <cmath>
#include <cstdint>
#include <limits>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "imu/mpu6050_driver.hpp"
#include "utils.hpp"
#include "wire_i2c_bus.hpp"
#include "wireless_console.hpp"

namespace {

constexpr uint32_t kSerialBaudRate{115200U};
constexpr uint32_t kI2cClockHz{200'000U};
constexpr uint8_t kImuInterruptPin{A6};
constexpr uint32_t kInterruptTimeoutMs{100U};
constexpr uint32_t kPrintEverySamples{10U};
constexpr uint8_t kMaxRetries{3U};

// The stationary diagnostic is intentionally long enough to estimate a useful
// variance for robot_localization. Increase to 60 s when doing final tuning.
constexpr uint32_t kStationaryDurationMs{30'000U};
constexpr uint32_t kStationaryExpectedSampleRateHz{100U};

// Startup calibration: the first second is discarded, followed by 5 s of
// stationary data at the configured 100 Hz sensor rate.
constexpr uint16_t kCalibrationSamples{3000U};
constexpr uint16_t kCalibrationIntervalMs{10U};

using Imu = MPU6050<WireI2cBus>;

Imu imu_sensor{WireI2cBus{Wire}};

#if __has_include("wifi_credentials.hpp")
#include "wifi_credentials.hpp"
constexpr const char* configured_ssid = wifi_credentials::kSsid;
constexpr const char* configured_password = wifi_credentials::kPassword;
#else
#pragma message("wifi_credentials.hpp not found. Using empty/fallback credentials.")
constexpr const char* configured_ssid = "";
constexpr const char* configured_password = "";
#endif

WirelessConsole console{WirelessConsole::Config{
    .ssid = configured_ssid,
    .password = configured_password,
    .port = 23,
    .log_buffer_size = 8192,
    .task_priority = 1,
    .cpu_core = 0,
}};

TaskHandle_t imu_data_task{nullptr};
uint32_t sample_count{0U};
uint32_t missed_notification_count{0U};
bool streaming_enabled{true};
bool diagnostic_active{false};

class RunningStatistics {
 public:
    void reset() noexcept {
        count_ = 0U;
        mean_ = 0.0;
        m2_ = 0.0;
        min_ = std::numeric_limits<double>::infinity();
        max_ = -std::numeric_limits<double>::infinity();
        sum_abs_ = 0.0;
    }

    void add(double value) noexcept {
        ++count_;

        const double delta = value - mean_;
        mean_ += delta / static_cast<double>(count_);
        const double delta2 = value - mean_;
        m2_ += delta * delta2;

        if (value < min_) {
            min_ = value;
        }
        if (value > max_) {
            max_ = value;
        }

        sum_abs_ += std::fabs(value);
    }

    [[nodiscard]] uint32_t count() const noexcept { return count_; }
    [[nodiscard]] double mean() const noexcept { return mean_; }

    [[nodiscard]] double variance() const noexcept {
        return count_ > 1U ? m2_ / static_cast<double>(count_ - 1U) : 0.0;
    }

    [[nodiscard]] double stddev() const noexcept { return std::sqrt(variance()); }

    [[nodiscard]] double rms() const noexcept {
        if (count_ == 0U) {
            return 0.0;
        }
        return std::sqrt((m2_ + static_cast<double>(count_) * mean_ * mean_) /
                         static_cast<double>(count_));
    }

    [[nodiscard]] double min() const noexcept { return count_ == 0U ? 0.0 : min_; }
    [[nodiscard]] double max() const noexcept { return count_ == 0U ? 0.0 : max_; }
    [[nodiscard]] double peakAbs() const noexcept {
        return std::fmax(std::fabs(min()), std::fabs(max()));
    }

    [[nodiscard]] double meanAbs() const noexcept {
        return count_ == 0U ? 0.0 : sum_abs_ / static_cast<double>(count_);
    }

 private:
    uint32_t count_{0U};
    double mean_{0.0};
    double m2_{0.0};
    double min_{0.0};
    double max_{0.0};
    double sum_abs_{0.0};
};

struct ImuStationaryStats {
    RunningStatistics accel_x{};
    RunningStatistics accel_y{};
    RunningStatistics accel_z{};
    RunningStatistics gyro_x{};
    RunningStatistics gyro_y{};
    RunningStatistics gyro_z{};

    void reset() noexcept {
        accel_x.reset();
        accel_y.reset();
        accel_z.reset();
        gyro_x.reset();
        gyro_y.reset();
        gyro_z.reset();
    }
};

[[noreturn]] void haltWithError(const char* message) {
    console.printf("[CRITICAL ERROR]: %s\n", message);
    for (;;) {
        delay(1000);
    }
}

void IRAM_ATTR handleImuDataReady() {
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(imu_data_task, &higher_priority_task_woken);

    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

void printSample(const mpu6050::IMUData& data) {
    console.printf(
        "Accel X: %.3f, Accel Y: %.3f, Accel Z: %.3f m/s^2 "
        "| Gyro X: %.5f, Gyro Y: %.5f, Gyro Z: %.5f rad/s\n",
        data.accelX, data.accelY, data.accelZ, data.gyroX, data.gyroY, data.gyroZ);
}

void printHelp() {
    console.println();
    console.println("========== MPU6050 Diagnostic Menu ==========");
    console.println("c : run stationary gyro/accelerometer calibration");
    console.println("m : run stationary statistics measurement");
    console.println("s : start continuous calibrated sample output");
    console.println("x : stop continuous sample output");
    console.println("i : print sensor/calibration/runtime information");
    console.println("h : print this menu");
    console.println();
    console.println("For calibration and measurement keep the robot completely stationary.");
    console.println();
}

void printInfo() {
    const auto& calibration = imu_sensor.calibration();

    console.println();
    console.println("=============== IMU INFO ===============");
    console.printf("Connected: %s\n", imu_sensor.isConnected() ? "yes" : "no");
    console.printf("Sensor sample rate: %u Hz\n", static_cast<unsigned>(mpu6050::kSampleRateHz));
    console.printf("I2C clock: %lu Hz\n", static_cast<unsigned long>(kI2cClockHz));
    console.printf("Interrupt pin: A6\n");
    console.printf("Calibration accel offsets: [%.6f, %.6f, %.6f] m/s^2\n", calibration.accelX,
                   calibration.accelY, calibration.accelZ);
    console.printf("Calibration gyro offsets:  [%.6f, %.6f, %.6f] rad/s\n", calibration.gyroX,
                   calibration.gyroY, calibration.gyroZ);
    console.printf("Samples read: %lu\n", static_cast<unsigned long>(sample_count));
    console.printf("Coalesced IRQ notifications: %lu\n",
                   static_cast<unsigned long>(missed_notification_count));
    console.printf("Streaming: %s\n", streaming_enabled ? "enabled" : "disabled");
    console.printf("Telnet: %s\n", console.telnetClientConnected() ? "connected" : "not connected");
    console.printf("Dropped wireless log bytes: %lu\n",
                   static_cast<unsigned long>(console.droppedBytes()));
    console.println();
}

void printAxisStatistics(const char* name, const RunningStatistics& stats, const char* unit) {
    const double mean = stats.mean();
    const double stddev = stats.stddev();
    const double variance = stats.variance();
    const double three_sigma = 3.0 * stddev;

    console.printf(
        "%s [%s] | N=%lu | mean=%+.8f | std=%+.8f | var=%.10e | "
        "min=%+.8f | max=%+.8f | peak_abs=%.8f | RMS=%.8f | 3sigma=%.8f\n",
        name, unit, static_cast<unsigned long>(stats.count()), mean, stddev, variance, stats.min(),
        stats.max(), stats.peakAbs(), stats.rms(), three_sigma);
}

void printCovarianceRecommendations(const ImuStationaryStats& stats) {
    console.println();
    console.println("----------- Covariance / EKF guidance -----------");
    console.println(
        "Use variance (std^2), not the maximum excursion, as the measurement covariance.");
    console.println(
        "For robot_localization vyaw, the stationary gyro Z variance is the key value.");
    console.printf("Suggested IMU angular.z covariance: %.10e rad^2/s^2\n",
                   stats.gyro_z.variance());
    console.printf("Equivalent standard deviation: %.8f rad/s\n", stats.gyro_z.stddev());
    console.printf("Residual stationary bias (mean): %+.8f rad/s\n", stats.gyro_z.mean());
    console.printf("3-sigma residual range: approximately +/- %.8f rad/s\n",
                   3.0 * stats.gyro_z.stddev());
    console.println();
    console.println("Important: covariance does NOT remove a constant gyro bias.");
    console.println(
        "If the stationary mean is materially non-zero, recalibrate or compensate that bias "
        "first.");
    console.println(
        "Do not copy the suggested value blindly; use the value from a sufficiently long "
        "stationary run.");
}

bool waitForImuSample(mpu6050::IMUData& calibrated_data,
                      uint32_t timeout_ms,
                      uint32_t& read_failure_count) {
    const uint32_t notification_count = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(timeout_ms));

    if (notification_count == 0U) {
        return false;
    }

    if (notification_count > 1U) {
        missed_notification_count += notification_count - 1U;
    }

    const auto raw = imu_sensor.read();
    if (!raw) {
        ++read_failure_count;
        return false;
    }

    ++sample_count;
    calibrated_data = imu_sensor.applyCalibration(*raw);
    return true;
}

bool runStationaryMeasurement() {
    if (diagnostic_active) {
        return false;
    }

    diagnostic_active = true;
    const bool was_streaming = streaming_enabled;
    streaming_enabled = false;

    console.println();
    console.println("Starting stationary measurement...");
    console.println("Keep the robot completely stationary and do not touch the IMU.");
    console.printf("Duration: %lu s at approximately %u Hz.\n",
                   static_cast<unsigned long>(kStationaryDurationMs / 1000U),
                   static_cast<unsigned>(kStationaryExpectedSampleRateHz));

    ImuStationaryStats stats{};
    stats.reset();

    const uint32_t start_ms = millis();
    uint32_t timeouts = 0U;
    uint32_t read_failures = 0U;

    while ((millis() - start_ms) < kStationaryDurationMs) {
        mpu6050::IMUData data{};
        if (!waitForImuSample(data, kInterruptTimeoutMs, read_failures)) {
            // A timeout and an I2C read failure are reported separately below.
            // The helper increments read_failures only for an actual read failure.
            continue;
        }

        stats.accel_x.add(data.accelX);
        stats.accel_y.add(data.accelY);
        stats.accel_z.add(data.accelZ);
        stats.gyro_x.add(data.gyroX);
        stats.gyro_y.add(data.gyroY);
        stats.gyro_z.add(data.gyroZ);

        // A failed read is indistinguishable from a timeout at this layer, so
        // the total number of accepted samples is the authoritative count.
        if (stats.gyro_z.count() % 500U == 0U) {
            console.printf("Progress: %lu accepted samples\n",
                           static_cast<unsigned long>(stats.gyro_z.count()));
        }
    }

    console.println();
    console.println("=============== STATIONARY STATISTICS ===============");
    printAxisStatistics("Accel X", stats.accel_x, "m/s^2");
    printAxisStatistics("Accel Y", stats.accel_y, "m/s^2");
    printAxisStatistics("Accel Z", stats.accel_z, "m/s^2");
    printAxisStatistics("Gyro X ", stats.gyro_x, "rad/s");
    printAxisStatistics("Gyro Y ", stats.gyro_y, "rad/s");
    printAxisStatistics("Gyro Z ", stats.gyro_z, "rad/s");

    const double measurement_seconds = static_cast<double>(millis() - start_ms) / 1000.0;
    const double measured_rate =
        measurement_seconds > 0.0 ? static_cast<double>(stats.gyro_z.count()) / measurement_seconds
                                  : 0.0;

    console.printf("Measured accepted sample rate: %.2f Hz\n", measured_rate);
    console.printf("DATA_RDY timeouts during measurement: %lu\n",
                   static_cast<unsigned long>(timeouts));
    console.printf("Total coalesced notifications: %lu\n",
                   static_cast<unsigned long>(missed_notification_count));
    console.printf("I2C/read failures during measurement: %lu\n",
                   static_cast<unsigned long>(read_failures));

    printCovarianceRecommendations(stats);

    streaming_enabled = was_streaming;
    diagnostic_active = false;
    return stats.gyro_z.count() > 1U;
}

void runCalibration() {
    if (diagnostic_active) {
        return;
    }

    diagnostic_active = true;
    const bool was_streaming = streaming_enabled;
    streaming_enabled = false;

    console.println();
    console.println("=============== CALIBRATION ===============");
    console.println("Keep the sensor stationary and flat for the entire calibration.");
    console.printf("Calibration samples: %u (~%.1f s)\n",
                   static_cast<unsigned>(kCalibrationSamples),
                   (static_cast<double>(kCalibrationSamples) * kCalibrationIntervalMs) / 1000.0);

    auto delay_function = [](unsigned long ms) { vTaskDelay(pdMS_TO_TICKS(ms)); };

    const auto calibration = utils::retry<kMaxRetries>([&delay_function] {
        return imu_sensor.calibrate(delay_function, kCalibrationSamples, kCalibrationIntervalMs);
    });

    if (!calibration) {
        console.println("[WARNING]: Calibration failed.");
    } else {
        console.printf(
            "Calibration offsets | Accel [%.6f, %.6f, %.6f] m/s^2 | "
            "Gyro [%.8f, %.8f, %.8f] rad/s\n",
            calibration->accelX, calibration->accelY, calibration->accelZ, calibration->gyroX,
            calibration->gyroY, calibration->gyroZ);
        console.println(
            "Calibration offsets are now applied by readCalibrated()/applyCalibration().");
    }

    streaming_enabled = was_streaming;
    diagnostic_active = false;
}

void processMenu() {
    char command{};
    while (console.readCommand(command)) {
        switch (command) {
            case 'c':
            case 'C':
                runCalibration();
                break;

            case 'm':
            case 'M':
                (void)runStationaryMeasurement();
                break;

            case 's':
            case 'S':
                streaming_enabled = true;
                console.println("Continuous calibrated sample output: STARTED");
                break;

            case 'x':
            case 'X':
                streaming_enabled = false;
                console.println("Continuous calibrated sample output: STOPPED");
                break;

            case 'i':
            case 'I':
                printInfo();
                break;

            case 'h':
            case 'H':
                printHelp();
                break;

            default:
                break;
        }
    }
}

}  // namespace

void setup() {
    Serial.begin(kSerialBaudRate);

    const uint32_t wait_start_ms = millis();
    constexpr uint32_t kSerialWaitMs{5'000U};
    while (!Serial && (millis() - wait_start_ms < kSerialWaitMs)) {
        delay(10);
    }

    console.setBootResetReason(esp_reset_reason());

    if (!console.begin()) {
        Serial.println("[CRITICAL ERROR]: Failed to start WirelessConsole task.");
        for (;;) {
            delay(1000);
        }
    }

    Wire.begin();
    Wire.setClock(kI2cClockHz);

    console.println();
    console.println("============================================");
    console.println("        BumperBot MPU6050 Diagnostic       ");
    console.println("============================================");
    console.printf("Reset reason: %d\n", static_cast<int>(esp_reset_reason()));
    console.printf("Wi-Fi/Telnet diagnostic console enabled on port %u.\n", 23U);

    console.println("Initializing MPU6050...");
    if (!utils::retry<kMaxRetries>(&Imu::connect, imu_sensor)) {
        haltWithError("Failed to connect to MPU6050.");
    }
    console.println("MPU6050 initialized.");

    // setup() and loop() execute in the same Arduino FreeRTOS task. Capture its
    // handle before the GPIO interrupt can ever be enabled.
    imu_data_task = xTaskGetCurrentTaskHandle();
    configASSERT(imu_data_task != nullptr);

    pinMode(kImuInterruptPin, INPUT);

    const int interrupt_number = digitalPinToInterrupt(kImuInterruptPin);
    if (interrupt_number < 0) {
        haltWithError("A6 is not available as an interrupt-capable pin.");
    }

    // Keep DATA_RDY disabled during the initial calibration. The calibration
    // implementation reads the sensor synchronously at the configured rate.
    runCalibration();

    attachInterrupt(interrupt_number, handleImuDataReady, RISING);

    if (!utils::retry<kMaxRetries>(&Imu::enableDataReadyInterrupt, imu_sensor)) {
        detachInterrupt(interrupt_number);
        haltWithError("Failed to enable MPU6050 DATA_RDY interrupt.");
    }

    console.printf("DATA_RDY interrupt enabled at %u Hz on A6.\n",
                   static_cast<unsigned>(mpu6050::kSampleRateHz));
    printInfo();
    printHelp();
}

void loop() {
    processMenu();

    // Diagnostic operations consume the same Arduino loop task. Do not attempt
    // to read the IMU here unless a DATA_RDY notification has arrived.
    if (diagnostic_active) {
        return;
    }

    const uint32_t notification_count =
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(kInterruptTimeoutMs));

    if (notification_count == 0U) {
        console.println("[WARNING]: MPU6050 DATA_RDY interrupt timeout.");
        return;
    }

    if (notification_count > 1U) {
        missed_notification_count += notification_count - 1U;
    }

    const auto raw = imu_sensor.read();
    if (!raw) {
        console.println("[WARNING]: Failed to read MPU6050 data.");
        return;
    }

    ++sample_count;
    const auto calibrated = imu_sensor.applyCalibration(*raw);

    if (streaming_enabled && (sample_count % kPrintEverySamples) == 0U) {
        printSample(calibrated);

        if (missed_notification_count != 0U) {
            console.printf("[WARNING]: %lu DATA_RDY notification(s) coalesced since startup.\n",
                           static_cast<unsigned long>(missed_notification_count));
        }
    }
}
