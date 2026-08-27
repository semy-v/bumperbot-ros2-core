#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include <cmath>
#include <cstdint>

#include <QuickPID.h>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"
#include "protocol/system_data.hpp"

/**
 * @brief Closed-loop angular-velocity controller for one BLDC2430-driven wheel.
 *
 * WheelController combines three layers:
 *
 *   1. BLDC2430Encoder + WheelVelocityEstimator
 *      The encoder timestamps consecutive FG falling edges and the estimator
 *      converts the full FG period into filtered wheel angular velocity [rad/s].
 *      ticks_per_rev therefore means FG falling-edge events per output-wheel
 *      revolution, not the historical rising+falling transition count.
 *
 *   2. Directional steady-state feed-forward
 *
 *        forward:  PWM_ff = +(Ks_forward + Kv * |target_velocity|)
 *        reverse:  PWM_ff = -(Ks_reverse + Kv * |target_velocity|)
 *
 *      Ks is direction-specific because the calibrated BLDC2430 wheel assemblies
 *      exhibit repeatable forward/reverse friction asymmetry. Kv is shared
 *      between directions. There is deliberately no startup/breakaway PWM term.
 *
 *   3. Bounded PID feedback
 *      QuickPID corrects residual tracking error around the feed-forward command.
 *      The PID correction is limited both by max_feedback_pwm and by the
 *      remaining actuator range, so PWM_ff + PWM_pid always stays in [-255,255].
 *      The final command is additionally prevented from changing sign relative
 *      to the requested target; direction reversal is handled only by Brake.
 *
 * Control states:
 *
 *   Inactive
 *      Controller disabled. Motor command is zero and PID/velocity state is reset.
 *
 *   Stopped
 *      Controller active with zero target. Motor command and reported controller
 *      velocity are forced to zero.
 *
 *   Normal
 *      Read the latest falling-edge period, update the reciprocal-period velocity
 *      estimate, run PID feedback, combine feed-forward + feedback, and command
 *      the motor.
 *
 *   Brake
 *      Used for stop requests and direction reversals. PID is disabled/reset and
 *      a small PWM command in the current motion direction is applied so the
 *      BLDC2430 driver continues producing FG pulses while wheel speed decays.
 *      Once braking completes, the controller either enters Stopped or starts
 *      the latest nonzero target.
 *
 * @important BLDC2430 FG is a single-channel speed signal. Encoder direction is
 *            inferred from the motor direction command, not independently
 *            measured from quadrature feedback. A physical direction reversal
 *            therefore must not be commanded until the wheel has actually slowed
 *            sufficiently in Brake.
 */
class WheelController {
 public:
    enum class ControlState : uint8_t { Inactive, Stopped, Normal, Brake };

    /**
     * @param direction_pin Motor CW/CCW command pin. The same signal is used by
     *        BLDC2430Encoder/PCNT to assign the logical velocity direction.
     * @param speed_control_pin Active-low BLDC2430 PWM speed-control pin.
     * @param speed_state_pin BLDC2430 FG feedback pin.
     * @param ticks_per_rev Empirically measured FG falling-edge events per
     *        output-wheel revolution.
     * @param invert_logic Mirrors logical forward/reverse for the opposite side
     *        of the differential drive while preserving a common robot convention.
     */
    WheelController(uint8_t direction_pin,
                    uint8_t speed_control_pin,
                    uint8_t speed_state_pin,
                    float ticks_per_rev,
                    bool invert_logic = false);

    ~WheelController() = default;

    WheelController(const WheelController&) = delete;
    WheelController(WheelController&&) = delete;
    WheelController& operator=(const WheelController&) = delete;
    WheelController& operator=(WheelController&&) = delete;

    /**
     * @brief Applies velocity-estimator, feed-forward, and PID configuration.
     *
     * max_feedback_pwm limits only the PID correction. The feed-forward term is
     * allowed to use the full actuator range; updatePidOutputLimits() further
     * clips PID headroom against the remaining +/-255 PWM range.
     */
    void configure(uint32_t control_period_ms, const WheelConfig& wheel_config);

    /**
     * @brief Initializes motor and encoder hardware.
     *
     * The motor is initialized first so the shared direction signal has a
     * deterministic output level before the encoder enables PCNT direction
     * tracking and falling-edge interrupts.
     */
    void begin();

    /**
     * @brief Enables or disables closed-loop control.
     *
     * Disabling clears the target, stops the motor, and discards controller
     * history. Re-enabling chooses Stopped for a zero target or prepares a fresh
     * motion start for a nonzero target.
     */
    void setActive(bool active = true);

    /**
     * @brief Executes one controller update.
     *
     * @param dt_ms Elapsed scheduler period in milliseconds. QuickPID uses its
     *        configured sample period internally; dt_ms is currently used for
     *        the bounded Brake-state duration.
     */
    void update(uint32_t dt_ms);

    /**
     * @brief Sets the desired wheel angular velocity [rad/s].
     *
     * Small commands inside the target-zero epsilon are normalized to zero.
     * A sign change while running enters Brake instead of immediately changing
     * the motor direction pin.
     */
    void setTargetVelocity(float target);

    /** Recomputes PID correction limits around the current feed-forward PWM. */
    void updatePidOutputLimits();

    /** Clears PID integral/output history and the stored correction term. */
    void resetPid();

    /**
     * @return Controller's current filtered velocity estimate [rad/s].
     *
     * In Inactive and Stopped this intentionally reports zero rather than
     * continuously observing externally forced/coasting wheel motion.
     */
    float getCurrentVelocity() const { return current_velocity_; }

 private:
    static constexpr float kMaxPwm{255.0f};
    static constexpr float kMinPwm{-255.0f};

    BLDC2430Motor motor_;
    BLDC2430Encoder encoder_;
    WheelVelocityEstimator velocity_estimator_;

    // QuickPID reads current_velocity_ and target_velocity_ by pointer and writes
    // only the feedback correction term. Feed-forward is combined separately.
    QuickPID pid_;

    ControlState state_{ControlState::Inactive};

    // Signed wheel angular velocities [rad/s].
    float target_velocity_{};
    float current_velocity_{};

    // Maximum configured feedback authority and latest QuickPID correction [PWM].
    float max_pid_correction_pwm_{};
    float pid_correction_pwm_{};

    // Directional steady-state feed-forward model.
    float feedforward_ks_forward_{};
    float feedforward_ks_reverse_{};
    float feedforward_kv_{};
    float feedforward_pwm_{};

    // Brake-state bookkeeping.
    uint32_t brake_total_period_ms_{};
    int brake_pwm_{};

    void updateNormal();
    bool updateBrake(uint32_t dt_ms);

    void prepareMotionStart();
    void enterStopped();
    void completeBraking();
    void beginBraking();

    // PWM zero disables motor drive. For the BLDC2430 driver this also prevents
    // relying on FG feedback during the fully-off interval.
    void stopMotor() { motor_.setPwmSpeed(0); }

    /** Calculates directional feed-forward PWM from the latest target velocity. */
    float calculateFeedForwardPwm() const;

    /**
     * @brief Combines feed-forward and PID correction into the actuator command.
     *
     * The result is saturated to [-255,255] and constrained to the target
     * direction so feedback cannot perform an uncoordinated direction reversal.
     */
    int calculateMotorSpeedPwm() const;

    /**
     * @brief Returns the low-speed command used while decelerating.
     *
     * A small command in the currently reported motion direction asks the motor
     * to decelerate toward a speed below its sustainable range while keeping the
     * driver active enough to continue generating FG timing information.
     *
     * This is not a reverse-torque command: the sign intentionally follows the
     * current motion direction. Direction reversal occurs only after Brake is
     * declared complete.
     */
    int calculateBrakePwm() const {
        constexpr float kBrakeDeadBandPwm{5.0f};
        return static_cast<int>(std::copysign(kBrakeDeadBandPwm, current_velocity_));
    }
};

#endif  // WHEEL_CONTROLLER_HPP