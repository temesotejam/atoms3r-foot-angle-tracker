#pragma once

#include <stdint.h>

struct ImuTelemetry {
    bool enabled = false;
    uint64_t sample_timestamp_us = 0;

    float ax = 0.0f;
    float ay = 0.0f;
    float az = 0.0f;
    float gx = 0.0f;
    float gy = 0.0f;
    float gz = 0.0f;

    float accel_norm_g = 0.0f;
    float gyro_norm_dps = 0.0f;
    float body_tilt_acc_deg = 0.0f;
    float body_tilt_cf_deg = 0.0f;
    bool tilt_static = false;

    uint32_t loop_count = 0;
    uint32_t deadline_misses = 0;
    uint32_t max_step_us = 0;
};
