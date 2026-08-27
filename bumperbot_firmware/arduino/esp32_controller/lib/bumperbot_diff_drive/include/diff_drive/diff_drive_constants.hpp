#ifndef DIFF_DRIVE_CONSTANTS_HPP
#define DIFF_DRIVE_CONSTANTS_HPP

#include <cstdint>

/// Number of BLDC2430 FG FALLING edges measured during one complete
/// left and right wheel revolution. Falling-edge-only counting is used by both
/// reciprocal-period velocity measurement and the full-revolution diagnostic counter.
inline constexpr int16_t kLeftMotorPulsePerRevolution{306};
inline constexpr int16_t kRightMotorPulsePerRevolution{306};

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

inline constexpr uint8_t kMotorsPowerEnablePin{2};

inline constexpr uint16_t kEmergencyStopTimeoutMs{2000};

#endif  // DIFF_DRIVE_CONSTANTS_HPP