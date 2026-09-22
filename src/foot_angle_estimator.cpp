#include "foot_angle_estimator.h"

#include "app_config.h"

FootAngleEstimate estimateFootAngle(
    const WhiteMarkerObservation& marker,
    float zero_x_px,
    bool zero_ready) {
    FootAngleEstimate out;

    const bool is_a = marker.id == appcfg::kMarkerAId;
    out.zero_x_px = zero_x_px;
    out.deg_per_px =
        is_a ? appcfg::kFootAngleADegPerPx
             : appcfg::kFootAngleBDegPerPx;

    if (!marker.valid) return out;

    const float min_x =
        is_a ? appcfg::kFootAngleAMinCalXPx
             : appcfg::kFootAngleBMinCalXPx;
    const float max_x =
        is_a ? appcfg::kFootAngleAMaxCalXPx
             : appcfg::kFootAngleBMaxCalXPx;

    out.in_calibration_range =
        marker.center_x_px >= min_x &&
        marker.center_x_px <= max_x;

    // Before automatic zeroing is locked, still calculate the provisional
    // value from the nominal calibration zero for diagnostics, but keep
    // angle_valid=false so control/UI cannot mistake it for a tared angle.
    out.angle_deg =
        out.deg_per_px * (out.zero_x_px - marker.center_x_px);
    out.valid = zero_ready;
    return out;
}
