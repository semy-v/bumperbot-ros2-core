#ifndef L298N_MOTOR_HPP
#define L298N_MOTOR_HPP

/**
 * L298N DC motor driver class.
 */
class L298NMotor {
public:
    L298NMotor(uint8_t en_pin, uint8_t in1_pin, uint8_t in2_pin)
        : en_pin_(en_pin)
        , in1_pin_(in1_pin)
        , in2_pin_(in2_pin)
    {}

    ~L298NMotor() = default;

    L298NMotor(const L298NMotor&) = default;
    L298NMotor(L298NMotor&&) = delete;
    L298NMotor& operator=(const L298NMotor&) = delete;
    L298NMotor& operator=(L298NMotor&&) = delete;

    void begin() const {
        pinMode(en_pin_, OUTPUT);
        pinMode(in1_pin_, OUTPUT);
        pinMode(in2_pin_, OUTPUT);
        setPWM(0);
    }

    // Accepts pwm from -255 (reverse) to 255 (forward)
    void setPWM(int16_t pwm) const {
        if (pwm >= 0) {
            digitalWrite(in1_pin_, LOW);
            digitalWrite(in2_pin_, HIGH);
        } else {
            digitalWrite(in1_pin_, HIGH);
            digitalWrite(in2_pin_, LOW);
            pwm = -pwm;
        }
        analogWrite(en_pin_, (pwm > 255) ? 255 : pwm);
    }

private:
    const uint8_t en_pin_, in1_pin_, in2_pin_;
};

#endif // L298N_MOTOR_HPP