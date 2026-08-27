#include "diff_drive/bldc2430_encoder.hpp"

#include <algorithm>

BLDC2430Encoder::BLDC2430Encoder(uint8_t direction_pin, uint8_t speed_state_pin, bool invert_logic)
    : pulse_counter_{direction_pin, speed_state_pin, 1, -1, invert_logic} {}

void BLDC2430Encoder::begin() {
    pulse_counter_.begin(&BLDC2430Encoder::handlePulseEdgeEvent, this);
    reset();
}

void BLDC2430Encoder::reset() {
    // Keep the PCNT ISR stopped while both the hardware count and software
    // timing snapshot are reset.  This avoids publishing an edge in the middle
    // of the reset transaction.
    pulse_counter_.pauseAndDisableInterrupt();
    pulse_counter_.clearCounter();

    seq_lock_.fetch_add(1U, std::memory_order_acquire);  // even -> odd: writer active

    last_edge_time_us_32_.store(static_cast<uint32_t>(esp_timer_get_time()),
                                std::memory_order_relaxed);
    motion_state_.store(MotionState{}, std::memory_order_relaxed);

    seq_lock_.fetch_add(1U, std::memory_order_release);  // odd -> even: snapshot published

    pulse_counter_.resumeAndEnableInterrupt();
}

EncoderEdgeData BLDC2430Encoder::getEdgeData() const {
    uint32_t edge_time_us{0U};
    MotionState state{};

    for (;;) {
        const uint32_t seq_before = seq_lock_.load(std::memory_order_acquire);

        if ((seq_before & 1U) != 0U) {
            // ISR/reset writer is updating the two-word snapshot.
            continue;
        }

        edge_time_us = last_edge_time_us_32_.load(std::memory_order_relaxed);
        state = motion_state_.load(std::memory_order_relaxed);

        // Keep payload reads before the final sequence validation.  If a writer
        // overlapped either read, the changed sequence value forces a retry.
        std::atomic_thread_fence(std::memory_order_acquire);
        const uint32_t seq_after = seq_lock_.load(std::memory_order_relaxed);

        if (seq_before == seq_after) {
            break;
        }
    }

    const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

    return EncoderEdgeData{
        .edge_time_us = edge_time_us,
        .delta_us = state.delta_us,
        .elapsed_us = now_us - edge_time_us,
        .direction = static_cast<int8_t>(state.direction != 0U ? 1 : -1),
    };
}

/*static*/ void IRAM_ATTR BLDC2430Encoder::handlePulseEdgeEvent(void* context) {
    auto* encoder = static_cast<BLDC2430Encoder*>(context);

    // check that limit event status available
    const auto opt_status = encoder->pulse_counter_.getLimitEventStatus();
    if (!opt_status) {
        return;
    }

    // With fixed +1/-1 limits and one counted falling edge, exactly one limit
    // should be reported. Reject an ambiguous status rather than publishing a
    // direction that cannot be determined reliably.
    const auto& status = opt_status.value();
    if (status.high_limit == status.low_limit) {
        return;
    }

    const uint32_t now_us = static_cast<uint32_t>(esp_timer_get_time());

    encoder->seq_lock_.fetch_add(1U, std::memory_order_acquire);  // even -> odd

    const uint32_t previous_us = encoder->last_edge_time_us_32_.load(std::memory_order_relaxed);
    encoder->last_edge_time_us_32_.store(now_us, std::memory_order_relaxed);

    const uint32_t delta_us = now_us - previous_us;
    constexpr uint32_t kMaximumStoredDeltaUs{0x7FFFFFFFU};

    const MotionState state{
        .delta_us = std::min(delta_us, kMaximumStoredDeltaUs),
        .direction = status.high_limit ? 1U : 0U,
    };

    encoder->motion_state_.store(state, std::memory_order_relaxed);

    encoder->seq_lock_.fetch_add(1U, std::memory_order_release);  // odd -> even
}