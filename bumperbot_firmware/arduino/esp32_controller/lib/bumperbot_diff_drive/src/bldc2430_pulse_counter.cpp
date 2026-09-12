#include "diff_drive/bldc2430_pulse_counter.hpp"

#include <cassert>

BLDC2430PulseCounter::BLDC2430PulseCounter(uint8_t direction_pin,
                                           uint8_t speed_state_pin,
                                           int16_t counter_h_limit,
                                           int16_t counter_l_limit,
                                           bool invert_logic)
    : pcnt_unit_{allocatePcntUnit()}, speed_state_pin_{speed_state_pin} {
    assert(counter_h_limit > 0);
    assert(counter_l_limit < 0);

    const pcnt_config_t config{
        // BLDC2430 open-collector FG output.
        .pulse_gpio_num = digitalPinToGPIONumber(speed_state_pin_),

        // Motor direction command.  PCNT uses this level to decide whether a
        // falling FG edge moves the counter toward the positive or negative
        // limit.
        .ctrl_gpio_num = digitalPinToGPIONumber(direction_pin),

        // Preserve the already validated logical direction convention used by
        // the left/right mirrored motor installation.
        .lctrl_mode = invert_logic ? PCNT_MODE_REVERSE : PCNT_MODE_KEEP,
        .hctrl_mode = invert_logic ? PCNT_MODE_KEEP : PCNT_MODE_REVERSE,

        // Reciprocal-period timing must use one complete FG period.  Ignore the
        // rising transition and count only the falling transition so FG duty
        // cycle does not create alternating short/long half-period samples.
        .pos_mode = PCNT_COUNT_DIS,
        .neg_mode = PCNT_COUNT_INC,

        .counter_h_lim = counter_h_limit,
        .counter_l_lim = counter_l_limit,
        .unit = pcnt_unit_,
        .channel = PCNT_CHANNEL_0,
    };

    ESP_ERROR_CHECK(pcnt_unit_config(&config));

    // Reject narrow electrical glitches before they can affect the hardware
    // counter.  With the 80 MHz APB clock, the maximum legacy-PCNT filter value
    // of 1023 corresponds to about 12.8 us, far shorter than a legitimate
    // BLDC2430 FG period in the expected wheel-speed range.
    constexpr uint16_t kMaxFilterValue{1023U};
    ESP_ERROR_CHECK(pcnt_set_filter_value(pcnt_unit_, kMaxFilterValue));
    ESP_ERROR_CHECK(pcnt_filter_enable(pcnt_unit_));
}

BLDC2430PulseCounter::~BLDC2430PulseCounter() {
    // Stop event delivery before disconnecting the PCNT channel.
    std::ignore = pcnt_intr_disable(pcnt_unit_);
    std::ignore = pcnt_counter_pause(pcnt_unit_);

    std::ignore = pcnt_event_disable(pcnt_unit_, PCNT_EVT_L_LIM);
    std::ignore = pcnt_event_disable(pcnt_unit_, PCNT_EVT_H_LIM);

    if (begun_) {
        std::ignore = pcnt_isr_handler_remove(pcnt_unit_);
    }

    std::ignore = pcnt_filter_disable(pcnt_unit_);
    std::ignore = pcnt_set_pin(pcnt_unit_, PCNT_CHANNEL_0, PCNT_PIN_NOT_USED, PCNT_PIN_NOT_USED);
}

void BLDC2430PulseCounter::begin(PulseCountCallback callback, void* context) {
    if (begun_) {
        return;
    }

    assert(callback != nullptr);

    // FG requires an external 3.3 V pull-up; INPUT intentionally does not
    // enable an internal pull-up here.
    pinMode(speed_state_pin_, INPUT);

    installIsrServiceOnce();

    ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit_, PCNT_EVT_L_LIM));
    ESP_ERROR_CHECK(pcnt_event_enable(pcnt_unit_, PCNT_EVT_H_LIM));

    // The per-unit ESP-IDF ISR calls this class-owned dispatcher.  All raw PCNT
    // status handling remains inside BLDC2430PulseCounter.
    ESP_ERROR_CHECK(pcnt_isr_handler_add(pcnt_unit_, callback, context));

    begun_ = true;
    reset();
}

void BLDC2430PulseCounter::reset() {
    assert(begun_);

    pauseAndDisableInterrupt();
    clearCounter();
    resumeAndEnableInterrupt();
}

void BLDC2430PulseCounter::pauseAndDisableInterrupt() {
    ESP_ERROR_CHECK(pcnt_intr_disable(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_counter_pause(pcnt_unit_));
}

void BLDC2430PulseCounter::clearCounter() {
    ESP_ERROR_CHECK(pcnt_counter_clear(pcnt_unit_));
}

void BLDC2430PulseCounter::resumeAndEnableInterrupt() {
    ESP_ERROR_CHECK(pcnt_counter_resume(pcnt_unit_));
    ESP_ERROR_CHECK(pcnt_intr_enable(pcnt_unit_));
}

/*static*/ pcnt_unit_t BLDC2430PulseCounter::allocatePcntUnit() {
    static pcnt_unit_t next_pcnt_unit{PCNT_UNIT_0};
    assert(next_pcnt_unit < PCNT_UNIT_MAX);

    const pcnt_unit_t unit = next_pcnt_unit;
    next_pcnt_unit = static_cast<pcnt_unit_t>(static_cast<int>(next_pcnt_unit) + 1);
    return unit;
}

/*static*/ void BLDC2430PulseCounter::installIsrServiceOnce() {
    static bool installed{false};
    if (!installed) {
        ESP_ERROR_CHECK(pcnt_isr_service_install(0));
        installed = true;
    }
}
