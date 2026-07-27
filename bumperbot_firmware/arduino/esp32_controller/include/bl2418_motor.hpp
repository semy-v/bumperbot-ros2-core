#ifndef BL2418_MOTOR_HPP
#define BL2418_MOTOR_HPP

#include <Arduino.h>

class BL2418Motor {
 public:
    /**
     * @brief Constructs a BL2418 motor controller.
     *
     * @param direction_pin Arduino pin connected to the BL2418 CW/CCW input.
     * @param speed_control_pin Arduino PWM-capable pin connected to the BL2418
     *        PWM input.
     * @param invert_direction When set to true, swaps the logical forward and
     *        reverse directions. This is useful for differential-drive robots
     *        where the left and right motors are mounted as mirror images.
     *
     * @note After applying the optional direction inversion, positive speed values
     * always correspond to the robot's logical forward wheel rotation, regardless
     * of the physical motor orientation.
     */
    BL2418Motor(uint8_t direction_pin, uint8_t speed_control_pin, bool invert_direction)
        : direction_pin_(direction_pin),
          speed_control_pin_(speed_control_pin),
          speed_channel_{next_channel_++},
          forward_direction_value_{invert_direction ? LOW : HIGH},
          reverse_direction_value_{invert_direction ? HIGH : LOW} {
        assert(speed_channel_ < SOC_LEDC_CHANNEL_NUM);
    }

    ~BL2418Motor() = default;

    BL2418Motor(const BL2418Motor&) = default;
    BL2418Motor(BL2418Motor&&) = delete;
    BL2418Motor& operator=(const BL2418Motor&) = delete;
    BL2418Motor& operator=(BL2418Motor&&) = delete;

    /**
     * @brief Initializes the motor controller.
     *
     * Configures the direction GPIO, allocates and configures an ESP32 LEDC PWM
     * channel, and places the motor into the stopped state.
     *
     * @note The BL2418 PWM input is active-low. A constant HIGH level disables the
     * motor output.
     */
    void begin() {
        pinMode(direction_pin_, OUTPUT);

        // Configure the default logical forward direction before enabling
        // the PWM output to avoid an unintended direction change during
        // initialization.
        digitalWrite(direction_pin_, forward_direction_value_);

        attachSpeedPwm();
        writeSpeedPwm(kMotorOffDuty);
    }

    /**
     * @brief Sets the motor speed and logical rotation direction.
     *
     * Positive values rotate the wheel in the configured forward direction,
     * negative values rotate it in the configured reverse direction, and zero
     * stops the motor.
     *
     * @param speed Requested motor speed in the range [-255, 255]. Values outside
     * the supported range are saturated to the nearest valid value.
     *
     * @note The BL2418 PWM input is active-low. Internally, the requested speed is
     * converted into an inverted PWM duty cycle before being written to the ESP32
     * LEDC peripheral:
     * - speed = 0   → duty = 255 → motor OFF
     * - speed = 255 → duty = 0   → maximum motor speed
     */
    void setPwmSpeed(int speed) {
        if (speed >= 0) {
            digitalWrite(direction_pin_, forward_direction_value_);
        } else {
            digitalWrite(direction_pin_, reverse_direction_value_);
            speed = -speed;
        }

        speed = std::clamp(speed, 0, 255);
        // Convert the requested speed into the active-low PWM duty
        const uint8_t duty = kMotorOffDuty - static_cast<uint8_t>(speed);
        writeSpeedPwm(duty);
    }

 private:
    // Recommended PWM frequency from the BL2418 datasheet:
    // typical operating range 15-25 kHz (60 kHz maximum). A frequency of 25 kHz is selected to
    // minimize audible noise while remaining within the recommended range.
    static constexpr uint32_t kFrequency{25000};  // 25 kHz

    // 8-bit PWM resolution (0-255 duty cycle).
    static constexpr uint8_t kResolution{8};

    // Allocate one unique LEDC channel for each motor instance.
    static constexpr uint8_t kMotorOffDuty{255};

    static inline uint8_t next_channel_{0};

    const uint8_t direction_pin_;
    const uint8_t speed_control_pin_;
    const uint8_t speed_channel_;

    // Logic levels corresponding to the robot's logical forward and reverse
    // wheel directions. These values may differ between the left and right
    // motors because they are mounted as mirror images.
    const uint8_t forward_direction_value_;
    const uint8_t reverse_direction_value_;

    void attachSpeedPwm() {
        ledcSetup(speed_channel_, kFrequency, kResolution);
        ledcAttachPin(speed_control_pin_, speed_channel_);
    }

    // Writes the already inverted PWM duty cycle to the LEDC peripheral.
    void writeSpeedPwm(const uint8_t duty) { ledcWrite(speed_channel_, duty); }
};

#endif  // BL2418_MOTOR_HPP
