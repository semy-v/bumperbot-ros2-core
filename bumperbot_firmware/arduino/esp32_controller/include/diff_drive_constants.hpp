#ifndef DIFF_DRIVE_CONSTANTS_HPP
#define DIFF_DRIVE_CONSTANTS_HPP

#include <cstdint>

/// Number of FG pulses generated during one complete wheel revolution of the
/// left motor. This value is determined experimentally and is used by the
/// encoder driver to detect full wheel revolutions.
inline constexpr int16_t kLeftMotorPulsePerRevolution{245};

/// Number of FG pulses generated during one complete wheel revolution of the
/// right motor. This value is determined experimentally and compensates for
/// manufacturing tolerances between the two motors.
inline constexpr int16_t kRightMotorPulsePerRevolution{256};

/// Arduino pin connected to the left motor FG (speed feedback) output.
inline constexpr uint8_t kLeftMotorSpeedStatePin{5};

/// Arduino pin connected to the left motor CW/CCW direction input.
inline constexpr uint8_t kLeftMotorDirectionPin{7};

/// Arduino PWM output pin connected to the left motor PWM input.
inline constexpr uint8_t kLeftMotorSpeedCommandPin{9};

/// Arduino pin connected to the right motor FG (speed feedback) output.
inline constexpr uint8_t kRightMotorSpeedStatePin{6};

/// Arduino pin connected to the right motor CW/CCW direction input.
inline constexpr uint8_t kRightMotorDirectionPin{8};

/// Arduino PWM output pin connected to the right motor PWM input.
inline constexpr uint8_t kRightMotorSpeedCommandPin{10};

#endif  // DIFF_DRIVE_CONSTANTS_HPP