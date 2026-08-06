#include <algorithm>

#include "diff_drive/bl2418_encoder.hpp"

BL2418Encoder::BL2418Encoder(uint8_t direction_pin,
                             uint8_t speed_state_pin,
                             int16_t counter_h_limit,
                             int16_t counter_l_limit,
                             bool invert_logic)
    : pcnt_unit_(allocatePcntUnit()), speed_state_pin_(speed_state_pin) {
    pcnt_config_t config{// FG output from the BL2418 motor controller.
                         .pulse_gpio_num = digitalPinToGPIONumber(speed_state_pin_),

                         // Motor direction command used by the PCNT hardware to determine
                         // whether the pulse count should be accumulated or reversed.
                         .ctrl_gpio_num = digitalPinToGPIONumber(direction_pin),

                         // Direction-dependent counting while the control signal is LOW.
                         .lctrl_mode = invert_logic ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE,

                         // Direction-dependent counting while the control signal is HIGH.
                         .hctrl_mode = invert_logic ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP,

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

BL2418Encoder::BL2418Encoder(uint8_t direction_pin, uint8_t speed_state_pin, bool invert_logic)
    : BL2418Encoder(direction_pin, speed_state_pin, 1, -1, invert_logic) {}

void BL2418Encoder::begin(PulseEdgeCallback callback, void* context) {
    // Configure the FG signal as a digital input before enabling the PCNT
    // peripheral.
    pinMode(speed_state_pin_, INPUT);

    installIsrServiceOnce();

    // Enable interrupt generation when the PCNT counter reaches the
    // configured positive and negative limits.
    ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit_, PCNT_EVT_L_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit_, PCNT_EVT_H_LIM));

    // Associate the user callback with this PCNT unit.
    ESP_ERROR_CHECK(pcnt_isr_handler_add(pcnt_unit_, callback, context));

    // Reset and start counter
    reset();
}

void BL2418Encoder::reset() {
    ESP_ERROR_CHECK(pcnt_intr_disable(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit_));
    last_edge_time_us_32_.store(static_cast<uint32_t>(esp_timer_get_time()),
                                std::memory_order_relaxed);
    motion_state_.store(MotionState{}, std::memory_order_relaxed);
    ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_unit_));
}

EncoderEdgeData BL2418Encoder::getEdgeData() const {
    uint32_t last_edge;
    MotionState state;
    uint32_t seq;

    for (;;) {
        const uint32_t seq_before = seq_lock_.load(std::memory_order_acquire);

        if (seq_before & 1) {
            // A write is in progress. Spin and wait for the ISR to finish.
            continue;
        }

        last_edge = last_edge_time_us_32_.load(std::memory_order_acquire);
        state = motion_state_.load(std::memory_order_acquire);

        const uint32_t seq_after = seq_lock_.load(std::memory_order_acquire);

        if (seq_before == seq_after) {
            break;
        }
    }

    const uint32_t now_32 = static_cast<uint32_t>(esp_timer_get_time());

    return EncoderEdgeData{.edge_time_us = last_edge,
                           .delta_us = state.delta_us,
                           .elapsed_us = now_32 - last_edge,
                           .direction = static_cast<int8_t>(state.direction ? 1 : -1)};
}

/*static*/ void IRAM_ATTR BL2418Encoder::handlePulseEdgeEvent(void* context) {
    auto* encoder = static_cast<BL2418Encoder*>(context);
    const uint32_t now_32 = static_cast<uint32_t>(esp_timer_get_time());
    uint32_t status;
    pcnt_get_event_status(encoder->pcnt_unit_, &status);

    encoder->seq_lock_.fetch_add(1, std::memory_order_acquire);

    const uint32_t prev_32 =
        encoder->last_edge_time_us_32_.exchange(now_32, std::memory_order_relaxed);
    const uint32_t delta = now_32 - prev_32;
    const uint32_t clamped_delta = std::min(delta, uint32_t{0x7FFFFFFF});

    const MotionState state{
        .delta_us = clamped_delta,
        .direction =
            (status & PCNT_EVT_H_LIM ? 1u /*forward direction*/ : 0u /* reverse direction */)};

    encoder->motion_state_.store(state, std::memory_order_relaxed);

    encoder->seq_lock_.fetch_add(1, std::memory_order_release);
}