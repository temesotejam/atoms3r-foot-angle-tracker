#pragma once

#include <stdint.h>

namespace appcfg {

static constexpr int kFrameWidth = 320;
static constexpr int kFrameHeight = 240;
static constexpr int kCameraFpsTarget = 15;

static constexpr int kMarkerAId = 0;
static constexpr int kMarkerBId = 1;

// Plain-white-marker sparse-line detector.
// Upper lane = A, lower lane = B.
static constexpr int kWhiteMarkerRowCount = 5;
static constexpr int kWhiteReferenceRowCount = 3;

static constexpr int kWhiteMarkerARows[kWhiteMarkerRowCount] =
    {58, 62, 66, 70, 74};
static constexpr int kWhiteReferenceARows[kWhiteReferenceRowCount] =
    {38, 42, 46};
static constexpr int kWhiteMarkerACenterY = 66;

static constexpr int kWhiteMarkerBRows[kWhiteMarkerRowCount] =
    {152, 156, 160, 164, 168};
static constexpr int kWhiteReferenceBRows[kWhiteReferenceRowCount] =
    {182, 186, 190};
static constexpr int kWhiteMarkerBCenterY = 160;

static constexpr float kWhitePeakMinContrast = 55.0f;
static constexpr float kWhiteCentroidBaseline = 35.0f;
static constexpr float kWhiteMinWeightSum = 150.0f;
static constexpr int kWhiteCentroidHalfWindowPx = 35;

// Foot-angle calibration v1, measured 2026-09-21.
// Definition:
//   foot_angle_deg = angle of the rigid foot/leg link relative to the body.
//   Upright initial posture = 0 deg.
//   Positive direction = the direction observed when marker X moves left.
//
// Baseline was the initial static interval (frames 57..97):
//   A x0 = 169.615317 px
//   B x0 = 174.843512 px
//
// Slopes were fitted through the upright-zero point using quasi-static
// calibration samples:
//   A: theta = 0.167779119 * (169.615317 - x)
//   B: theta = 0.162645305 * (174.843512 - x)
static constexpr float kFootAngleAZeroXPx = 169.615317f;
static constexpr float kFootAngleBZeroXPx = 174.843512f;
static constexpr float kFootAngleADegPerPx = 0.167779119f;
static constexpr float kFootAngleBDegPerPx = 0.162645305f;

// Observed quasi-static calibration support, with a small guard margin.
// The angle is still computed outside this range, but angle_in_range=false.
static constexpr float kFootAngleAMinCalXPx = 42.0f;
static constexpr float kFootAngleAMaxCalXPx = 173.0f;
static constexpr float kFootAngleBMinCalXPx = 43.5f;
static constexpr float kFootAngleBMaxCalXPx = 177.5f;

// IMU diagnostics retained to validate body motion independently.
static constexpr float kBodyTiltComplementaryTauS = 0.50f;
static constexpr float kTiltStaticMaxGyroDps = 3.0f;
static constexpr float kTiltStaticAccelNormToleranceG = 0.05f;

// Automatic upright zeroing.
//
// On the current AtomS3R-CAM mount, gravity points approximately along IMU -Z
// when the body is upright. Upright recognition is intentionally based only
// on IMU gravity direction + stability; marker X is never used to decide
// whether the body is upright.
//
// The condition must remain continuously true before marker A/B X positions
// are averaged and locked as the 0 deg reference for the rest of the boot.
static constexpr float kAutoZeroMaxUprightErrorDeg = 5.0f;
static constexpr float kAutoZeroMaxGyroDps = 1.5f;
static constexpr float kAutoZeroAccelNormToleranceG = 0.03f;
static constexpr uint32_t kAutoZeroStableMs = 2000;
static constexpr uint32_t kAutoZeroMinVisionSamples = 15;

static constexpr uint32_t kImuControlPeriodUs = 5000; // 200 Hz
static constexpr uint32_t kTelemetryPeriodMs = 100;   // 10 Hz JSON

} // namespace appcfg
