# IMU-gated upright auto zero v2

## Purpose

Remove the small boot-to-boot mechanical zero offset without changing the
measured pixel-to-angle slopes.

The camera still measures marker X. The BMI270 is used only to decide when the
body is sufficiently upright and stable to define the current boot's 0 deg
reference.

## Upright definition

Current hardware mounting:

    upright gravity direction = IMU -X

A 200 Hz IMU sample is an upright candidate only when all conditions are true:

    gravity direction error <= 5.0 deg
    abs(accel_norm_g - 1.0) <= 0.03 g
    gyro_norm_dps <= 1.5 deg/s

The direction error is the 3-D angle between normalized acceleration and IMU
-X. This avoids deciding upright from a single Euler angle.

## Stability

The conditions must remain continuously true for at least 2000 ms.

The IMU task measures this continuity at 200 Hz. If motion occurs between two
10 Hz camera frames, upright_stable_ms restarts, so the camera-side averaging
window is discarded as well.

## Marker averaging

While the IMU upright condition is continuously valid and both white markers
are detected, the firmware accumulates:

    x_zero,A = mean(cx_A)
    x_zero,B = mean(cx_B)

At least 15 camera samples and 2000 ms of continuous IMU stability are required.

If either marker is lost before lock, the partial average is discarded.

After lock, zero values are immutable until reboot.

## Angle output

The measured calibration slopes are unchanged:

    theta_A = 0.167779119 * (x_zero,A - x_A)
    theta_B = 0.162645305 * (x_zero,B - x_B)

Before the zero is locked, the firmware keeps raw cx_px and a provisional angle
for diagnostics, but reports:

    angle_valid = false

After lock:

    zeroing.ready = true
    angle_valid = true

## Important assumption

The IMU can determine that the body is upright; it cannot independently know
that each foot is in its intended neutral pose. Therefore the robot must be
placed with the feet in the desired 0 deg pose while the startup auto-zero is
performed.
