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
        noInterrupts();
        long current_ticks{ticks_};
        interrupts();
        return current_ticks;
    }

    // ISR handler for 2x decoding. Triggered on CHANGE of Phase A.
    void ARDUINO_ISR_ATTR update() {
        // Logic: On a change of Phase A, if A and B are the same, 
        // we are moving in one direction. If they are different, the other.
        const bool pinA_state = digitalRead(pinA_);
        const bool pinB_state = digitalRead(pinB_);

        if (pinA_state == pinB_state) {
          ticks_-=1;
        } else {
          ticks_+=1;
        }
    }

private:
    const uint8_t pinA_, pinB_;
    volatile long ticks_{0};
};

#endif // QUADRATURE_ENCODER_HPP