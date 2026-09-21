# Foot-angle calibration v1

Date: 2026-09-21

## Definition

The output angle is the rigid foot/leg link angle **relative to the body**.

- Initial upright posture: 0 deg
- Positive direction: white-marker X moves left
- Marker A: upper image lane
- Marker B: lower image lane

The calibration experiment fixed the feet in the world and rotated the body.
Because the camera is rigidly attached to the body and the markers are rigidly
attached to the feet, the camera observes the same body-foot relative angle that
will occur in normal operation when the body is approximately fixed and the
feet rotate.

## Upright reference

Initial static interval: frames 57..97.

- A zero X: 169.615317 px
- B zero X: 174.843512 px
- mean complementary-filter body tilt: 91.043366 deg

The IMU's ~91 deg absolute number is a mount-coordinate convention. It is not
used as the foot-angle zero; the initial upright posture is explicitly defined
as 0 deg.

## Runtime equations

Marker A:

    theta_A = 0.167779119 * (169.615317 - x_A)

Marker B:

    theta_B = 0.162645305 * (174.843512 - x_B)

The firmware reports these values as foot_angle_deg.

## Calibration support

Observed quasi-static X support:

- A: 42.877 .. 172.245 px
- B: 44.315 .. 176.971 px

The firmware keeps a small guard margin and reports angle_in_range. It still
computes the angle outside the measured range, but that value is extrapolation.

## Notes

This is calibration v1. The raw cx_px value is always preserved in telemetry so
future calibration models can be compared without repeating every experiment.
