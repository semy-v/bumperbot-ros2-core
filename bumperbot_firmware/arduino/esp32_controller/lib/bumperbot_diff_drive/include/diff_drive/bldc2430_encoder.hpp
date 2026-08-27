#ifndef BLDC2430_ENCODER_HPP
#define BLDC2430_ENCODER_HPP

#include <atomic>

#include <Arduino.h>
#include "driver/pcnt.h"

/**
 * @brief Reads the BLDC2430 motor feedback (FG) signal using the ESP32 PCNT
 *        peripheral.
 *
 * The ESP32 Pulse Counter (PCNT) is a dedicated hardware peripheral that
 * counts input pulses independently of the CPU. Once configured, it monitors
 * the FG signal and updates the pulse count entirely in hardware without
 * requiring a GPIO interrupt or software polling for every pulse edge.
 *
 * Using the PCNT peripheral minimizes CPU utilization while providing reliable
 * pulse measurement even when the processor is busy executing other tasks,
 * such as motor control, communication, or ROS 2 message handling.
 *
 * The BLDC2430 motor controller generates an FG (Frequency Generator) pulse
 * train whose frequency is proportional to the wheel speed. This class
 * configures one ESP32 PCNT unit to accumulate FG signal transitions while
 * automatically tracking the wheel rotation direction.
 *
 * The motor direction command is connected to the PCNT control input. The
 * hardware uses this signal to determine whether encoder counts should be
 * accumulated in the positive or negative direction. The mapping between the
 * direction signal and the counting direction can optionally be inverted,
 * allowing identical software conventions to be used even when motors are
 * mounted as mirror images on opposite sides of a differential-drive robot.
 *
 * Both rising and falling edges of the FG signal are counted, effectively
 * doubling the available encoder resolution compared to counting a single
 * edge only.
 *
 * @note PCNT counts electrical signal transitions (edges), not logical pulses.
 *       With both rising and falling edges enabled, each FG pulse contributes
 *       two counter increments or decrements.
 */

struct EncoderEdgeData {
    uint32_t edge_time_us{0};
    uint32_t delta_us{0};
    uint32_t elapsed_us{0};
    int8_t direction{1};
};

class BLDC2430Encoder {
 public:
    using PulseEdgeCallback = void (*)(void* context);

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
     * @param invert_logic Inverts the relationship between the motor direction
     *        control signal and the PCNT counting direction. This allows both
     *        left and right wheel encoders to report positive counts for forward
     *        robot motion even when the motors are mounted in mirrored
     *        orientations.
     *
     * @note The limit values can be configured to generate PCNT hardware events
     *       after a predefined number of encoder counts, for example one complete
     *       wheel revolution.
     */
    BLDC2430Encoder(uint8_t direction_pin,
                  uint8_t speed_state_pin,
                  int16_t counter_h_limit,
                  int16_t counter_l_limit,
                  bool invert_logic);

    BLDC2430Encoder(uint8_t direction_pin, uint8_t speed_state_pin, bool invert_logic);

    ~BLDC2430Encoder();

    BLDC2430Encoder(const BLDC2430Encoder&) = delete;
    BLDC2430Encoder(BLDC2430Encoder&&) = delete;
    BLDC2430Encoder& operator=(const BLDC2430Encoder&) = delete;
    BLDC2430Encoder& operator=(BLDC2430Encoder&&) = delete;

    /**
     * @brief Initializes the PCNT counter.
     *
     * Configures the FG pin as an input, clears any previously accumulated
     * pulse count, and starts the hardware pulse counter.
     */
    void begin() { begin(&BLDC2430Encoder::handlePulseEdgeEvent, this); }

    void begin(PulseEdgeCallback callback, void* context);

    void reset();

    uint32_t getLastEdgeTimeUs() const {
        return last_edge_time_us_32_.load(std::memory_order_relaxed);
    }

    EncoderEdgeData getEdgeData() const;

 private:
    struct alignas(4) MotionState {
        uint32_t delta_us : 31 {0};  // Microsecond period between consecutive edges
        uint32_t direction : 1 {1};  // 1 = Forward (+1), 0 = Reverse (-1)
    };

    static_assert(std::atomic<MotionState>::is_always_lock_free,
                  "atomic MotionState has to be lock-free.");
    static_assert(std::atomic<uint32_t>::is_always_lock_free,
                  "atomic uint32_t has to be lock-free.");

    const pcnt_unit_t pcnt_unit_;
    uint8_t speed_state_pin_;

    std::atomic<uint32_t> seq_lock_{0};
    std::atomic<uint32_t> last_edge_time_us_32_{0};
    std::atomic<MotionState> motion_state_{};

    static void IRAM_ATTR handlePulseEdgeEvent(void* context);

    // Allocates a unique PCNT hardware unit for each encoder instance.
    //
    // The ESP32 contains a limited number of independent PCNT units. Each
    // BLDC2430Encoder exclusively owns one unit for its lifetime. Construction
    // fails with an assertion if more encoder instances are created than the
    // hardware supports.
    static pcnt_unit_t allocatePcntUnit() {
        static pcnt_unit_t next_pcnt_unit{PCNT_UNIT_0};
        assert(next_pcnt_unit < PCNT_UNIT_MAX);

        const auto unit = next_pcnt_unit;
        next_pcnt_unit = static_cast<pcnt_unit_t>(static_cast<int>(next_pcnt_unit) + 1);

        return unit;
    }

    static void installIsrServiceOnce() {
        static bool installed{false};
        if (!installed) {
            ESP_ERROR_CHECK(pcnt_isr_service_install(0));
            installed = true;
        }
    }
};

#endif  // BLDC2430_ENCODER_HPP