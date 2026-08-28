# BLDC2430 Directional Feed-Forward Calibration

This diagnostic identifies the steady-state feed-forward model for the left and
right BLDC2430 wheel assemblies with the fully assembled robot moving on its
normal floor surface.

The production model is

```math
\mathrm{PWM}_{ff}(\omega)=
\begin{cases}
K_{s,f}+K_v\lvert\omega\rvert, & \omega>0 \\[4pt]
0, & \omega=0 \\[4pt]
-\left(K_{s,r}+K_v\lvert\omega\rvert\right), & \omega<0
\end{cases}
```

where:

- $\omega$ — wheel angular velocity, $\mathrm{rad/s}$;
- $K_{s,f}$ — forward steady-state intercept;
- $K_{s,r}$ — reverse steady-state intercept;
- $K_v$ — shared velocity slope,
  $\mathrm{PWM}/(\mathrm{rad/s})$.

The calibration fits **no startup/breakaway PWM term**.

A calibration-only **initial-movement prephase** is executed before each signed
test point to reduce the launch impulse that can unload the rear caster during
high-speed backward starts. Prephase data is never used by the regression.

---

# 1. Calibration model and PID relationship

For every accepted test point:

```math
x_i=\lvert\omega_i\rvert,
\qquad
y_i=\lvert\mathrm{PWM}_i\rvert
```

Forward and reverse groups use separate intercepts and one common slope:

```math
y_{f,i}=K_{s,f}+K_vx_{f,i}
```

```math
y_{r,i}=K_{s,r}+K_vx_{r,i}
```

The calibration identifies only

```math
K_{s,f},\qquad K_{s,r},\qquad K_v
```

It does **not** calculate or tune

```math
K_p,\qquad K_i,\qquad K_d
```

In the production `WheelController`:

```math
\mathrm{PWM}_{cmd}
=
\mathrm{PWM}_{ff}
+
\mathrm{PWM}_{pid}
```

where `max_feedback_pwm` limits the PID correction. Feed-forward supplies the
nominal motor command; PID corrects residual error caused by load, battery
voltage, disturbances, temperature, and remaining model error.

Recommended order:

```text
1. Calibrate Ks_forward, Ks_reverse, and Kv.
2. Install the feed-forward coefficients.
3. Verify tracking with PID gains set to zero.
4. Tune Kp, Ki, and Kd for the remaining error.
```

---

# 2. Encoder and timing assumptions

The diagnostic uses the production `BLDC2430Encoder` and
`WheelVelocityEstimator`.

The encoder measures **FG falling edges only**:

```text
falling edge N
      |
      | one complete FG period
      |
falling edge N+1
```

`ticks_per_rev` therefore means the number of FG falling-edge events per
output-wheel revolution. Do not use the historical rising+falling transition
count.

Wheel-specific values come from:

```cpp
kLeftMotorPulsePerRevolution
kRightMotorPulsePerRevolution
```

The calibration update rate is:

```cpp
kCalibrationUpdateRateHz = 100;
```

or a nominal:

```text
10 ms
```

update period.

---

# 3. Default configuration

```cpp
constexpr FeedForwardCalibrationRunner::Config kCalibrationConfig{
    .velocity_pwm_start = 40,
    .velocity_pwm_end = 200,
    .velocity_pwm_step = 10,

    .calibrate_forward = true,
    .calibrate_reverse = true,

    .initial_movement_pwm = 50,
    .initial_movement_time_ms = 1000,

    .settle_time_ms = 1000,

    .capture_time_ms = 2000,
    .capture_sample_count = 100,
    .max_capture_wait_ms = 2500,

    .brake_time_ms = 250,
    .stop_time_ms = 500,

    .max_capture_relative_stddev = 0.10F,
    .max_capture_relative_mean_drift = 0.05F,

    .minimum_regression_velocity = 1.0F,
    .minimum_regression_points_per_direction = 5,
};
```

The signed sweep is:

```text
+40, -40, +50, -50, ... , +200, -200
```

This gives 17 PWM magnitudes and up to 34 signed samples per wheel.

---

# 4. Initial-movement prephase

The drive wheels are toward the front of the chassis and the caster is at the
rear. Applying a large backward PWM directly from rest can pitch the chassis and
momentarily lift/unload the caster.

The prephase establishes motion in the requested direction before the exact test
PWM is applied.

It is **calibration-only**:

- it is not fitted;
- it is not captured;
- it is not a production `WheelConfig` parameter.

For each test point:

```math
\lvert\mathrm{PWM}_{init}\rvert
=
\min\left(
\mathrm{PWM}_{init,max},
\lvert\mathrm{PWM}_{test}\rvert
\right)
```

and

```math
\mathrm{PWM}_{init}
=
\operatorname{sgn}(\mathrm{PWM}_{test})
\lvert\mathrm{PWM}_{init}\rvert
```

With `initial_movement_pwm = 50`:

| Test PWM | Initial PWM |
|---:|---:|
| +40 | +40 |
| -40 | -40 |
| +100 | +50 |
| -100 | -50 |
| +200 | +50 |
| -200 | -50 |

The prephase is time-based. Velocity estimation continues, but the measurements
are ignored by the calibration statistics.

The following `Settling` phase is still required because it removes the transient
created by the step from `PWM_init` to the exact `PWM_test`.

---

# 5. Application sequence

Starting with command `s`:

```text
initial motor-driver reset
        |
        v
start both wheel runners
        |
        v
execute one signed point on both wheels
        |
        v
wait for both runners
        |
        v
shared driver power cycle
        |
        v
advance both runners together
        |
        v
...
        |
        v
fit left/right models and print results
```

Both wheels always execute the same signed PWM point concurrently.

## Shared motor-driver reset

Between signed points:

```text
motor supply OFF --300 ms--> motor supply ON --150 ms--> ready
```

The same power cycle is also performed once before the complete calibration.

---

# 6. Per-point state machine

```text
ApplyInitialMovement
        |
        v
InitialMovement
        |
        v
ApplyTestPWM
        |
        v
Settling
        |
        v
CaptureSample
        |
        v
Brake
        |
        v
Stop
        |
        v
WaitForPeer
```

| State | Purpose |
|---|---|
| `ApplyInitialMovement` | Synchronize estimator history and apply the bounded same-direction prephase PWM. |
| `InitialMovement` | Move for `initial_movement_time_ms`; estimator stays active, data is not captured. |
| `ApplyTestPWM` | Switch to the exact PWM point; do not reset the estimator. |
| `Settling` | Hold exact test PWM for `settle_time_ms`; data is not captured. |
| `CaptureSample` | Capture exact-test-PWM velocity and evaluate the point. |
| `Brake` | Command PWM zero for `brake_time_ms`. |
| `Stop` | Keep PWM zero, reset estimator, wait `stop_time_ms`. |
| `WaitForPeer` | Wait for the other wheel before the shared power cycle and next point. |

Before `InitialMovement`, the runner executes:

```cpp
const EncoderEdgeData edge = encoder_.getEdgeData();
estimator_.reset(edge.edge_time_us, true);
```

This marks the previous edge as consumed and discards the first new interval,
which can include part of the preceding stopped period.

The estimator is intentionally **not reset** when changing from the initial PWM
to the exact test PWM. The subsequent settling interval excludes that transition
from the captured data.

After both runners reach `WaitForPeer`, the shared motor supply is reset and both
runners advance together.

---

# 7. Capture statistics

A normal capture completes when:

```text
elapsed capture time >= capture_time_ms
AND
finite samples >= capture_sample_count
```

Defaults:

```text
capture_time_ms      = 2000 ms
capture_sample_count = 100
max_capture_wait_ms  = 2500 ms
```

At 100 Hz, a normal two-second capture usually contains about 200 observations.

Zero, low-speed, and wrong-direction measurements are **not discarded during
capture**. They remain part of the candidate sample and are evaluated afterward.

## Mean velocity

For $N$ observations $\omega_i$:

```math
\bar{\omega}
=
\frac{1}{N}
\sum_{i=1}^{N}\omega_i
```

The regression uses $\lvert\bar{\omega}\rvert$.

## Standard deviation

```math
\sigma_\omega^2
=
\frac{1}{N}
\sum_{i=1}^{N}\omega_i^2
-
\bar{\omega}^2
```

```math
\sigma_\omega
=
\sqrt{\max\left(0,\sigma_\omega^2\right)}
```

The normalized variation is:

```math
\sigma_{\mathrm{rel}}
=
\frac{\sigma_\omega}{\lvert\bar{\omega}\rvert}
```

Current limit:

```math
\sigma_{\mathrm{rel}}\le0.10
```

## Mean drift

The capture is split into first and second halves:

```math
\bar{\omega}_1
=
\frac{1}{N_1}
\sum_{i\in H_1}\omega_i
```

```math
\bar{\omega}_2
=
\frac{1}{N_2}
\sum_{i\in H_2}\omega_i
```

Relative drift is:

```math
d_{\mathrm{rel}}
=
\frac{
\lvert\bar{\omega}_2-\bar{\omega}_1\rvert
}{
\lvert\bar{\omega}\rvert
}
```

Current limit:

```math
d_{\mathrm{rel}}\le0.05
```

Minimum and maximum captured velocity are also logged but are not direct
acceptance criteria.

---

# 8. Sample acceptance

A point is accepted only if:

1. all statistics are finite;
2. test PWM is nonzero;
3. sample count is at least `capture_sample_count`;
4. mean speed satisfies $\lvert\bar{\omega}\rvert \ge \omega_{min}$, where
   `minimum_regression_velocity` supplies $\omega_{min}$;
5. mean velocity sign matches test PWM sign;
6. $\sigma_{\mathrm{rel}}$ does not exceed its configured limit;
7. $d_{\mathrm{rel}}$ does not exceed its configured limit.

Current thresholds:

```text
|mean velocity| >= 1.0 rad/s
relative sigma  <= 10 %
relative drift  <= 5 %
```

Rejected points are logged but do not abort the sweep.

---

# 9. Directional regression

Accepted samples are grouped by direction:

```math
g\in\{f,r\}
```

```math
x_{g,i}=\lvert\omega_{g,i}\rvert,
\qquad
y_{g,i}=\lvert\mathrm{PWM}_{g,i}\rvert
```

Group means:

```math
\bar{x}_g
=
\frac{1}{N_g}\sum_i x_{g,i}
```

```math
\bar{y}_g
=
\frac{1}{N_g}\sum_i y_{g,i}
```

The common slope is fitted from within-group centered data:

```math
K_v
=
\frac{
\displaystyle
\sum_{g\in\{f,r\}}
\sum_i
(x_{g,i}-\bar{x}_g)
(y_{g,i}-\bar{y}_g)
}{
\displaystyle
\sum_{g\in\{f,r\}}
\sum_i
(x_{g,i}-\bar{x}_g)^2
}
```

The direction-specific intercepts are:

```math
K_{s,f}
=
\bar{y}_f-K_v\bar{x}_f
```

```math
K_{s,r}
=
\bar{y}_r-K_v\bar{x}_r
```

At least `minimum_regression_points_per_direction` accepted samples are required
for both directions. The current minimum is 5.

Regression fails for insufficient points/spread or invalid fitted coefficients.

---

# 10. Fit-quality metrics

## Coefficient of determination

```math
R^2
=
1-
\frac{
\displaystyle\sum_i(y_i-\hat{y}_i)^2
}{
\displaystyle\sum_i(y_i-\bar{y})^2
}
```

Warning threshold:

```text
R² < 0.98
```

## PWM RMSE

```math
\mathrm{RMSE}_{PWM}
=
\sqrt{
\frac{1}{N}
\sum_{i=1}^{N}
(y_i-\hat{y}_i)^2
}
```

Warning threshold:

```text
RMSE > 5 PWM
```

## Prediction at 2 rad/s

At

```math
\lvert\omega\rvert=2\;\mathrm{rad/s}
```

the predicted magnitudes are:

```math
\lvert\mathrm{PWM}_{2,f}\rvert
=
K_{s,f}+2K_v
```

```math
\lvert\mathrm{PWM}_{2,r}\rvert
=
K_{s,r}+2K_v
```

---

# 11. Output and production use

Successful calibration reports, for each wheel:

```text
Ks_forward
Ks_reverse
Kv
R²
RMSE
accepted forward-point count
accepted reverse-point count
```

and prints the production parameter names:

```text
left_feedforward_ks_forward
left_feedforward_ks_reverse
left_feedforward_kv

right_feedforward_ks_forward
right_feedforward_ks_reverse
right_feedforward_kv
```

Do not copy calibration-only timing/acceptance parameters into the production
feed-forward model.

---

# 12. Commands and setup

Wireless-console commands:

| Command | Action |
|---|---|
| `s` | Start calibration |
| `x` | Abort immediately |
| `h` | Print help/configuration |

Before running:

1. Place the assembled robot on its normal floor surface.
2. Provide enough space for the full forward/reverse sweep.
3. Verify that the rear caster moves freely.
4. Verify motor power, common ground, and FG wiring.
5. Verify falling-edge counts per wheel revolution.
6. Verify left/right `invert_logic`.
7. Use a representative battery state when comparing runs.

Do not lift or hold the robot for production calibration; that removes the normal
drivetrain/chassis load.

On abort or regression failure, both motor PWM commands are set to zero.

---

# 13. Tuning the initial-movement prephase

Increase `initial_movement_pwm` if the robot does not begin moving reliably.
Decrease it if the prephase itself causes excessive pitch.

The effective value always remains bounded by:

```math
\lvert\mathrm{PWM}_{init}\rvert
=
\min\left(
\mathrm{PWM}_{init,max},
\lvert\mathrm{PWM}_{test}\rvert
\right)
```

Increase `initial_movement_time_ms` if the caster or chassis needs more time to
settle into normal rolling motion before the test PWM is applied.

## Current limitation

The current implementation still makes one direct step:

```text
initial PWM -> exact test PWM
```

For example:

```text
-50 -> -200
```

If the caster stays on the ground at `-50` but lifts during that transition, the
remaining issue is the acceleration step itself. Increasing the prephase duration
will not remove it.

The next refinement would be a configurable PWM ramp before `Settling`, for
example:

```text
-50 -> -70 -> -90 -> ... -> -200
```

Such a ramp is not part of the current implementation.

---

# 14. Sequence summary

```text
shared driver reset
        |
        v
initial movement at bounded same-direction PWM
        |   not captured
        v
exact test PWM
        |
        v
settling
        |   not captured
        v
capture exact-test-PWM velocity
        |
        v
mean / sigma / drift
        |
        +--> accept -> regression sample
        |
        +--> reject -> log reason
        |
        v
PWM zero -> stationary pause -> WaitForPeer
        |
        v
next shared driver reset
```

Only an **accepted capture at the exact test PWM** contributes to
$K_{s,f}$, $K_{s,r}$, and $K_v$.
