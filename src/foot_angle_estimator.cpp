#include "foot_angle_estimator.h"

#include "app_config.h"

FootAngleEstimate estimateFootAngle(
    const WhiteMarkerObservation& marker) {
    FootAngleEstimate out;

    const bool is_a = marker.id == appcfg::kMarkerAId;
    out.zero_x_px =
        is_a ? appcfg::kFootAngleAZeroXPx
             : appcfg::kFootAngleBZeroXPx;
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

    out.valid = true;
    out.in_calibration_range =
        marker.center_x_px >= min_x &&
        marker.center_x_px <= max_x;

    // Upright calibration position is 0 deg.
    // Positive angle is the direction where marker X decreases.
    out.angle_deg =
        out.deg_per_px * (out.zero_x_px - marker.center_x_px);
    return out;
}
