#ifndef BL2418_ENCODER_HPP
#define BL2418_ENCODER_HPP

#include <functional>
#include <limits>
#include <optional>

#include <Arduino.h>
#include "driver/pcnt.h"

/**
 * @brief Reads the BL2418 motor feedback (FG) signal using the ESP32 PCNT
 *        peripheral.
 *
 * The ESP32 Pulse Counter (PCNT) is a dedicated hardware peripheral that
 * counts input pulses independently of the CPU. Once configured, it monitors
 * the FG signal and updates the pulse count in hardware without requiring a
 * GPIO interrupt or software polling for every pulse edge.
 *
 * Using the PCNT peripheral reduces CPU load and provides reliable pulse
 * measurement even when the processor is busy executing other tasks, such as
 * motor control, communication, or ROS 2 message handling.
 *
 * The BL2418 motor controller generates an FG (Frequency Generator) pulse
 * train whose frequency is proportional to the wheel speed. This class
 * configures one ESP32 PCNT unit to accumulate FG pulse edges while
 * automatically tracking the wheel rotation direction.
 *
 * The motor direction signal is connected to the PCNT control input. The PCNT
 * hardware automatically reverses the counter when the direction input
 * indicates reverse rotation, eliminating the need for software-side sign
 * correction.
 *
 * Both rising and falling edges of the FG signal are counted to maximize the
 * available encoder resolution.
 *
 * @note PCNT counts electrical signal transitions (edges), not logical pulses.
 *       With both rising and falling edges enabled, one FG pulse contributes
 *       two counts.
 */
class BL2418Encoder {
 public:
    /**
     * @brief Constructs an encoder using the ESP32 hardware pulse counter.
     *
     * A dedicated PCNT unit is allocated for this encoder instance. The PCNT
     * peripheral continuously counts FG signal edges in hardware after
     * initialization, allowing the CPU to read accumulated counts only when
     * required.
     *
     * @param direction_pin Arduino pin connected to the motor CW/CCW control
     *        signal.
     * @param speed_state_pin Arduino pin connected to the motor FG output.
     * @param counter_h_limit Positive PCNT limit value.
     * @param counter_l_limit Negative PCNT limit value.
     *
     * @note The limit values can be configured to generate PCNT hardware events
     *       after a predefined number of encoder counts, for example one complete
     *       wheel revolution.
     */
    BL2418Encoder(uint8_t direction_pin,
                  uint8_t speed_state_pin,
                  int16_t counter_h_limit,
                  int16_t counter_l_limit)
        : pcnt_unit_(allocatePcntUnit()), speed_state_pin_(speed_state_pin) {
        pcnt_config_t config{// FG output from the BL2418 motor controller.
                             .pulse_gpio_num = digitalPinToGPIONumber(speed_state_pin_),

                             // Motor direction command used by the PCNT hardware to determine
                             // whether the pulse count should be accumulated or reversed.
                             .ctrl_gpio_num = digitalPinToGPIONumber(direction_pin),

                             // Reverse the counting direction whenever the control signal is LOW.
                             // The BL2418Motor class drives LOW for reverse wheel rotation.
                             .lctrl_mode = PCNT_MODE_REVERSE,

                             // Preserve the configured counting direction while the control signal
                             // remains HIGH (logical forward).
                             .hctrl_mode = PCNT_MODE_KEEP,

                             // Count every rising edge of the FG signal.
                             .pos_mode = PCNT_COUNT_INC,

                             // Count every falling edge as well, doubling the effective encoder
                             // resolution compared to counting a single edge only.
                             .neg_mode = PCNT_COUNT_INC,

                             .counter_h_lim = counter_h_limit,
                             .counter_l_lim = counter_l_limit,

                             .unit = pcnt_unit_,
                             .channel = PCNT_CHANNEL_0};

        ESP_ERROR_CHECK(pcnt_unit_config(&config));

        // Enable the maximum hardware glitch filter.
        // The filter rejects any pulse shorter than 1023 APB clock cycles
        // (1023 / 80 MHz ≈ 12.8 µs). The BL2418 FG output has a measured
        // minimum high/low pulse width of approximately 1090 µs at maximum
        // motor speed, providing an ~85× safety margin while effectively
        // suppressing narrow noise spikes on the FG line.
        constexpr uint16_t kMaxFilterValue{1023};
        ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_unit_, kMaxFilterValue));
        ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_unit_));
    }

    /**
     * @brief Constructs an encoder with the maximum PCNT counter range.
     *
     * The counter is configured to use the full signed 16-bit range and will
     * not generate high- or low-limit events during normal operation.
     */
    BL2418Encoder(uint8_t direction_pin, uint8_t speed_state_pin)
        : BL2418Encoder(direction_pin,
                        speed_state_pin,
                        std::numeric_limits<int16_t>::max(),
                        std::numeric_limits<int16_t>::min()) {}

    ~BL2418Encoder() = default;

    BL2418Encoder(const BL2418Encoder&) = default;
    BL2418Encoder(BL2418Encoder&&) = delete;
    BL2418Encoder& operator=(const BL2418Encoder&) = delete;
    BL2418Encoder& operator=(BL2418Encoder&&) = delete;

    /**
     * @brief Initializes the PCNT counter.
     *
     * Configures the FG pin as an input, clears any previously accumulated
     * pulse count, and starts the hardware pulse counter.
     */
    void begin() {
        // Configure the FG signal as a digital input before enabling the PCNT
        // peripheral.
        pinMode(speed_state_pin_, INPUT);

        // Reset the hardware counter to a known state before beginning pulse
        // accumulation.
        ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_unit_));
        ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit_));
        ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_unit_));
    }

    /**
     * @brief Reads the accumulated hardware pulse count.
     *
     * The returned value is obtained directly from the ESP32 PCNT hardware counter.
     * No software pulse counting is performed.
     *
     * @param clear_counter If true, resets the hardware counter after reading.
     *
     * @return Signed number of FG signal edges accumulated by the PCNT peripheral.
     */
    int read(bool clear_counter = true) {
        int16_t count{};
        ESP_ERROR_CHECK(pcnt_get_counter_value(pcnt_unit_, &count));
        if (clear_counter) {
            ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit_));
        }
        return count;
    }

 private:
    const pcnt_unit_t pcnt_unit_;
    uint8_t speed_state_pin_;

    // Allocates a unique PCNT hardware unit for each encoder instance.
    //
    // The ESP32 contains a limited number of independent PCNT units. Each
    // BL2418Encoder exclusively owns one unit for its lifetime. Construction
    // fails with an assertion if more encoder instances are created than the
    // hardware supports.
    static pcnt_unit_t allocatePcntUnit() {
        static pcnt_unit_t next_pcnt_unit{PCNT_UNIT_0};
        assert(next_pcnt_unit < PCNT_UNIT_MAX);

        const auto unit = next_pcnt_unit;
        next_pcnt_unit = static_cast<pcnt_unit_t>(static_cast<int>(next_pcnt_unit) + 1);

        return unit;
    }

    friend class EncoderPcntTestHelper;
};

#endif  // BL2418_ENCODER_HPP