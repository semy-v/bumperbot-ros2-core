#ifndef BLDC2430_ENCODER_HPP
#define BLDC2430_ENCODER_HPP

#include <Arduino.h>

#include <atomic>
#include <cstdint>

#include "diff_drive/bldc2430_pulse_counter.hpp"

/**
 * @brief Latest reciprocal-period timing snapshot produced by BLDC2430Encoder.
 *
 * edge_time_us is the timestamp of the most recent counted FG falling edge.
 * delta_us is the interval between the two most recent falling edges and thus
 * represents one complete FG period.  elapsed_us is the age of the latest edge
 * at the time getEdgeData() is called.  direction follows the logical motor
 * direction convention: +1 forward, -1 reverse.
 */
struct EncoderEdgeData {
    uint32_t edge_time_us{0U};
    uint32_t delta_us{0U};
    uint32_t elapsed_us{0U};
    int8_t direction{1};
};

/**
 * @brief BLDC2430 falling-edge timing encoder for wheel-velocity estimation.
 *
 * BLDC2430Encoder is intentionally a narrow velocity-measurement abstraction.
 * It owns a BLDC2430PulseCounter permanently configured with PCNT limits +1/-1,
 * so every FG falling edge generates one callback.  Custom PCNT limits are not
 * exposed by this class; code that needs revolution-sized or other accumulated
 * pulse events must use BLDC2430PulseCounter directly.
 *
 * Measuring falling-edge-to-falling-edge periods rather than alternating
 * rising/falling half-periods removes sensitivity to FG duty-cycle asymmetry.
 * The resulting delta_us is therefore suitable for the reciprocal-period
 * WheelVelocityEstimator.
 *
 * The motor direction-command GPIO is also connected to PCNT.  PCNT uses that
 * command level to choose the positive or negative limit, and the limit event
 * is translated into direction +1 or -1.  This means direction is commanded
 * direction, not an independent measurement of physical shaft direction; the
 * BLDC2430 FG output is single-channel and cannot provide quadrature direction.
 *
 * The ISR publishes timestamp, period, and direction through lock-free 32-bit
 * atomics protected by a sequence counter so getEdgeData() receives a coherent
 * snapshot without disabling interrupts in the control task.
 */
class BLDC2430Encoder {
 public:
    /**
     * @param direction_pin Arduino pin connected to the BLDC2430 direction input.
     * @param speed_state_pin Arduino pin connected to the BLDC2430 FG output.
     * @param invert_logic Inverts the direction-command/count-direction mapping
     *        for a mirrored motor installation.
     */
    BLDC2430Encoder(uint8_t direction_pin, uint8_t speed_state_pin, bool invert_logic);

    ~BLDC2430Encoder() = default;

    BLDC2430Encoder(const BLDC2430Encoder&) = delete;
    BLDC2430Encoder(BLDC2430Encoder&&) = delete;
    BLDC2430Encoder& operator=(const BLDC2430Encoder&) = delete;
    BLDC2430Encoder& operator=(BLDC2430Encoder&&) = delete;

    /**
     * @brief Starts falling-edge timing acquisition.
     *
     * The owned pulse counter installs BLDC2430Encoder::handlePulseEdgeEvent as
     * its only callback.  No user callback is exposed by this class.
     */
    void begin();

    /**
     * @brief Clears PCNT state and the published timing snapshot.
     *
     * The first edge after reset spans reset->first-edge rather than a complete
     * FG period.  Consumers that restart reciprocal-period estimation should
     * therefore discard that first edge interval (WheelVelocityEstimator
     * supports this with reset(consumed_edge_time_us, true)).
     */
    void reset();

    [[nodiscard]] uint32_t getLastEdgeTimeUs() const {
        return last_edge_time_us_32_.load(std::memory_order_relaxed);
    }

    /** @brief Returns a coherent latest falling-edge timing snapshot. */
    [[nodiscard]] EncoderEdgeData getEdgeData() const;

 private:
    struct alignas(4) MotionState {
        // Full FG period in microseconds.  31 bits provide >35 minutes of range.
        uint32_t delta_us : 31 {0U};

        // Compact ISR representation: 1 = logical forward, 0 = logical reverse.
        uint32_t direction : 1 {1U};
    };

    static_assert(sizeof(MotionState) == sizeof(uint32_t),
                  "MotionState must remain exactly one 32-bit word");
    static_assert(std::atomic<MotionState>::is_always_lock_free,
                  "atomic MotionState has to be lock-free");
    static_assert(std::atomic<uint32_t>::is_always_lock_free,
                  "atomic uint32_t has to be lock-free");

    static void IRAM_ATTR handlePulseEdgeEvent(void* context);

    // Fixed +/-1 limits are fundamental to this abstraction: one callback per
    // FG falling edge gives one complete falling-edge-to-falling-edge period.
    BLDC2430PulseCounter pulse_counter_;

    std::atomic<uint32_t> seq_lock_{0U};
    std::atomic<uint32_t> last_edge_time_us_32_{0U};
    std::atomic<MotionState> motion_state_{};
};

#endif  // BLDC2430_ENCODER_HPP