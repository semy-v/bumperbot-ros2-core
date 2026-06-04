#ifndef QUADRATURE_ENCODER_HPP
#define QUADRATURE_ENCODER_HPP

#include <Arduino.h>

/**
 * 2x Quadrature Encoder class.
 */
class QuadratureEncoder {
public:
    QuadratureEncoder(uint8_t pinA, uint8_t pinB)
        : pinA_(pinA)
        , pinB_(pinB)
        , pinA_register_(portInputRegister(digitalPinToPort(pinA_)))
        , pinB_register_(portInputRegister(digitalPinToPort(pinB_)))
        , pinA_mask_(digitalPinToBitMask(pinA_))
        , pinB_mask_(digitalPinToBitMask(pinB_))
    {}

    ~QuadratureEncoder() = default;

    QuadratureEncoder(const QuadratureEncoder&) = default;
    QuadratureEncoder(QuadratureEncoder&&) = delete;
    QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;
    QuadratureEncoder& operator=(QuadratureEncoder&&) = delete;

    // Initialize pins. Interrupts must be attached manually outside the class.
    void begin() {
        pinMode(pinA_, INPUT_PULLUP);
        pinMode(pinB_, INPUT_PULLUP);
    }

    // Safely read the current position
    long read() const {
        // On 8-bit AVRs, reading a 32-bit volatile requires disabling interrupts
        // to prevent "torn reads" (where an interrupt changes the value mid-read).
        noInterrupts();
        long current_ticks{ticks_};
        interrupts();
        return current_ticks;
    }

    // ISR handler for 2x decoding. Triggered on CHANGE of Phase A.
    void update() {
        // Logic: On a change of Phase A, if A and B are the same, 
        // we are moving in one direction. If they are different, the other.
        const bool pinA_state = (*pinA_register_ & pinA_mask_);
        const bool pinB_state = (*pinB_register_ & pinB_mask_);

        if (pinA_state == pinB_state) {
          ticks_-=1;
        } else {
          ticks_+=1;
        }
    }

private:
    const uint8_t pinA_, pinB_;

    // Pointers to the volatile HW input registers (e.g. &PIND, &PINB)
    volatile uint8_t* pinA_register_; 
    volatile uint8_t* pinB_register_;

    // HW bitmasks corresponding to the port mappings
    const uint8_t pinA_mask_, pinB_mask_;

    volatile long ticks_{0};
};

#endif // QUADRATURE_ENCODER_HPP