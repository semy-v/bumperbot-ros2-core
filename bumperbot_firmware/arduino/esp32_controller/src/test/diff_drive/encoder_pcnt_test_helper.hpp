#ifndef ENCODER_PCNT_TEST_HELPER_HPP
#define ENCODER_PCNT_TEST_HELPER_HPP

#include <Arduino.h>
#include "driver/pcnt.h"

#include "bl2418_encoder.hpp"


/**
 * @brief Test utility for registering ESP32 PCNT interrupt callbacks.
 *
 * The BL2418Encoder class intentionally exposes only the minimal encoder
 * interface required by the application:
 *
 *   - initialize the pulse counter hardware.
 *   - read accumulated encoder pulses.
 *
 * Direct PCNT interrupt registration is not part of the production encoder
 * interface because application-level interrupt handling is not required for
 * normal wheel odometry operation.
 *
 * This helper provides additional functionality used only by hardware
 * validation tests. It allows tests to receive notifications when the PCNT
 * hardware counter reaches configured high or low limits.
 *
 * The ESP32 Pulse Counter (PCNT) peripheral is a dedicated hardware component
 * that counts external pulses independently of the CPU. Once configured,
 * encoder FG pulses are accumulated in hardware without requiring an interrupt
 * or software processing for every pulse.
 *
 * Interrupts are generated only for selected PCNT events, such as:
 *
 *   - PCNT_EVT_H_LIM:
 *       Counter reached the configured positive limit.
 *
 *   - PCNT_EVT_L_LIM:
 *       Counter reached the configured negative limit.
 *
 * These events are used by wheel rotation tests to detect completion of a
 * predefined number of revolutions without continuously polling the encoder.
 *
 * This helper is intentionally separated from BL2418Encoder to keep production
 * code independent from test-specific interrupt handling.
 */
class EncoderPcntTestHelper {
 public:
    /**
     * @brief Callback type invoked from the PCNT interrupt handler.
     *
     * The callback executes in interrupt context. It must therefore be short,
     * non-blocking, and should not perform operations that depend on FreeRTOS
     * scheduling, dynamic memory allocation, or serial communication.
     *
     * @param arg User-provided context pointer.
     */
    using Callback = void (*)(void* arg);


    /**
     * @brief Registers a callback for PCNT counter limit events.
     *
     * Enables PCNT high and low limit events and attaches an interrupt handler
     * for the specified encoder instance.
     *
     * The callback is triggered when the encoder pulse counter reaches either:
     *
     *   - the positive configured limit (PCNT_EVT_H_LIM)
     *   - the negative configured limit (PCNT_EVT_L_LIM)
     *
     * The counter limits are configured when the BL2418Encoder object is
     * created. For wheel rotation tests these limits typically correspond to
     * one complete wheel revolution worth of FG pulses.
     *
     * @param encoder Encoder instance whose PCNT unit will generate events.
     * @param callback Function executed when the PCNT event occurs.
     * @param context User-defined pointer passed to the callback.
     *
     * @note This function is intended for hardware validation tests only.
     */
    static void registerLowHighLimitEventCallback(BL2418Encoder& encoder,
                                                  Callback callback,
                                                  void* context) {
        installServiceOnce();

        // Enable interrupt generation when the PCNT counter reaches the
        // configured positive and negative limits.
        ESP_ERROR_CHECK(pcnt_event_enable(encoder.pcnt_unit_, PCNT_EVT_L_LIM));
        ESP_ERROR_CHECK(pcnt_event_enable(encoder.pcnt_unit_, PCNT_EVT_H_LIM));

        // Associate the user callback with this PCNT unit.
        ESP_ERROR_CHECK(
            pcnt_isr_handler_add(encoder.pcnt_unit_, callback, context));

        // Enable PCNT interrupt generation.
        ESP_ERROR_CHECK(pcnt_intr_enable(encoder.pcnt_unit_));
    }


 private:
    /*
     * The PCNT interrupt service must be installed only once globally.
     *
     * All PCNT units share the same ESP32 PCNT ISR service. Individual encoder
     * instances register their own handlers through pcnt_isr_handler_add().
     */
    static inline bool installed_{false};


    /**
     * @brief Installs the ESP32 PCNT ISR service once.
     *
     * Multiple encoder instances can exist simultaneously (for example, left
     * and right differential drive wheels), but the underlying ISR service is
     * shared by all PCNT units.
     */
    static void installServiceOnce() {
        if (!installed_) {
            ESP_ERROR_CHECK(pcnt_isr_service_install(0));
            installed_ = true;
        }
    }
};


#endif // ENCODER_PCNT_TEST_HELPER_HPP