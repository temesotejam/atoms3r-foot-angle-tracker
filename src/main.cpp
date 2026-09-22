#include <Arduino.h>
#include <M5Unified.h>
#include <math.h>
#include "esp_timer.h"

#include "app_config.h"
#include "camera_driver.h"
#include "foot_angle_estimator.h"
#include "vision_types.h"
#include "white_marker_tracker.h"

namespace {

CameraDriver g_camera;
WhiteMarker1DTracker g_tracker_a(appcfg::kMarkerAId);
WhiteMarker1DTracker g_tracker_b(appcfg::kMarkerBId);

portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
ImuTelemetry g_imu;
TaskHandle_t g_imu_task = nullptr;

uint32_t g_last_telemetry_ms = 0;
uint32_t g_frame_count = 0;
uint32_t g_camera_failures = 0;
uint32_t g_max_vision_us = 0;
uint32_t g_frame_dt_us = 0;
uint64_t g_last_frame_timestamp_us = 0;

struct AutoZeroState {
    bool ready = false;
    bool collecting = false;
    uint32_t samples = 0;
    uint32_t last_upright_stable_ms = 0;
    float sum_a_x = 0.0f;
    float sum_b_x = 0.0f;
    float zero_a_x =
        appcfg::kFootAngleAZeroXPx;
    float zero_b_x =
        appcfg::kFootAngleBZeroXPx;
};

AutoZeroState g_auto_zero;

float wrapAngleDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

ImuTelemetry getImuSnapshot() {
    ImuTelemetry imu;
    portENTER_CRITICAL(&g_imu_mux);
    imu = g_imu;
    portEXIT_CRITICAL(&g_imu_mux);
    return imu;
}

void resetAutoZeroCollection() {
    if (g_auto_zero.ready) return;
    g_auto_zero.collecting = false;
    g_auto_zero.samples = 0;
    g_auto_zero.last_upright_stable_ms = 0;
    g_auto_zero.sum_a_x = 0.0f;
    g_auto_zero.sum_b_x = 0.0f;
}

void updateAutoZero(const ImuTelemetry& imu,
                    const WhiteMarkerObservation& a,
                    const WhiteMarkerObservation& b) {
    if (g_auto_zero.ready) return;

    const bool markers_valid = a.valid && b.valid;
    if (!imu.enabled || !imu.upright_candidate || !markers_valid) {
        resetAutoZeroCollection();
        return;
    }

    // If the 200 Hz IMU task observed even a brief instability between two
    // camera frames, upright_stable_ms will restart from zero. Detect that
    // restart here and discard the partially accumulated marker average.
    if (g_auto_zero.collecting &&
        imu.upright_stable_ms <
            g_auto_zero.last_upright_stable_ms) {
        resetAutoZeroCollection();
    }

    g_auto_zero.collecting = true;
    g_auto_zero.sum_a_x += a.center_x_px;
    g_auto_zero.sum_b_x += b.center_x_px;
    ++g_auto_zero.samples;
    g_auto_zero.last_upright_stable_ms =
        imu.upright_stable_ms;

    if (imu.upright_stable_ms >= appcfg::kAutoZeroStableMs &&
        g_auto_zero.samples >= appcfg::kAutoZeroMinVisionSamples) {
        const float n =
            static_cast<float>(g_auto_zero.samples);
        g_auto_zero.zero_a_x =
            g_auto_zero.sum_a_x / n;
        g_auto_zero.zero_b_x =
            g_auto_zero.sum_b_x / n;
        g_auto_zero.ready = true;
        g_auto_zero.collecting = false;

        Serial.printf(
            "AUTO_ZERO_LOCKED: A=%.3f px, B=%.3f px, samples=%u, "
            "upright_stable_ms=%u\n",
            g_auto_zero.zero_a_x,
            g_auto_zero.zero_b_x,
            g_auto_zero.samples,
            imu.upright_stable_ms);
    }
}

void controlStep(const ImuTelemetry&) {
    // Reserved for future control. Vision/angle estimation never blocks this task.
}

void imuControlTask(void*) {
    const TickType_t period_ticks =
        pdMS_TO_TICKS(appcfg::kImuControlPeriodUs / 1000);
    TickType_t last_wake = xTaskGetTickCount();

    uint32_t max_step_us = 0;
    uint32_t deadline_misses = 0;
    uint32_t loop_count = 0;

    bool tilt_initialized = false;
    float tilt_cf_deg = 0.0f;
    uint64_t last_tilt_timestamp_us = 0;
    uint64_t upright_start_us = 0;

    for (;;) {
        const uint32_t t0 = micros();

        ImuTelemetry sample;
        sample.enabled = M5.Imu.isEnabled();

        if (sample.enabled) {
            M5.Imu.update();
            const auto data = M5.Imu.getImuData();

            sample.sample_timestamp_us =
                static_cast<uint64_t>(esp_timer_get_time());

            sample.ax = data.accel.x;
            sample.ay = data.accel.y;
            sample.az = data.accel.z;
            sample.gx = data.gyro.x;
            sample.gy = data.gyro.y;
            sample.gz = data.gyro.z;

            sample.accel_norm_g = sqrtf(
                sample.ax * sample.ax +
                sample.ay * sample.ay +
                sample.az * sample.az);

            sample.gyro_norm_dps = sqrtf(
                sample.gx * sample.gx +
                sample.gy * sample.gy +
                sample.gz * sample.gz);

            sample.body_tilt_acc_deg =
                atan2f(-sample.az, -sample.ax) *
                57.2957795131f;

            if (!tilt_initialized) {
                tilt_cf_deg = sample.body_tilt_acc_deg;
                tilt_initialized = true;
            } else if (sample.sample_timestamp_us >
                       last_tilt_timestamp_us) {
                const float dt =
                    static_cast<float>(
                        sample.sample_timestamp_us -
                        last_tilt_timestamp_us) * 1e-6f;

                if (dt > 0.0f && dt < 0.05f) {
                    const float predicted =
                        tilt_cf_deg + sample.gy * dt;
                    const float error =
                        wrapAngleDeg(
                            sample.body_tilt_acc_deg - predicted);
                    const float beta =
                        dt /
                        (appcfg::kBodyTiltComplementaryTauS + dt);
                    tilt_cf_deg =
                        wrapAngleDeg(predicted + beta * error);
                } else {
                    tilt_cf_deg = sample.body_tilt_acc_deg;
                }
            }

            last_tilt_timestamp_us = sample.sample_timestamp_us;
            sample.body_tilt_cf_deg = tilt_cf_deg;
            sample.tilt_static =
                sample.gyro_norm_dps <=
                    appcfg::kTiltStaticMaxGyroDps &&
                fabsf(sample.accel_norm_g - 1.0f) <=
                    appcfg::kTiltStaticAccelNormToleranceG;

            // Upright recognition for automatic zeroing.
            // Current hardware: body upright => gravity approximately IMU -X.
            if (sample.accel_norm_g > 0.1f) {
                float gravity_alignment =
                    -sample.ax / sample.accel_norm_g;
                if (gravity_alignment > 1.0f) gravity_alignment = 1.0f;
                if (gravity_alignment < -1.0f) gravity_alignment = -1.0f;

                sample.upright_error_deg =
                    acosf(gravity_alignment) *
                    57.2957795131f;

                sample.upright_candidate =
                    sample.upright_error_deg <=
                        appcfg::kAutoZeroMaxUprightErrorDeg &&
                    sample.gyro_norm_dps <=
                        appcfg::kAutoZeroMaxGyroDps &&
                    fabsf(sample.accel_norm_g - 1.0f) <=
                        appcfg::kAutoZeroAccelNormToleranceG;
            }

            if (sample.upright_candidate) {
                if (upright_start_us == 0) {
                    upright_start_us =
                        sample.sample_timestamp_us;
                }

                const uint64_t stable_us =
                    sample.sample_timestamp_us -
                    upright_start_us;
                sample.upright_stable_ms =
                    stable_us > 0xffffffffULL * 1000ULL
                        ? 0xffffffffU
                        : static_cast<uint32_t>(
                            stable_us / 1000ULL);
            } else {
                upright_start_us = 0;
                sample.upright_stable_ms = 0;
            }
        } else {
            upright_start_us = 0;
        }

        const uint32_t step_us = micros() - t0;
        if (step_us > max_step_us) max_step_us = step_us;
        if (step_us > appcfg::kImuControlPeriodUs) ++deadline_misses;
        ++loop_count;

        sample.loop_count = loop_count;
        sample.deadline_misses = deadline_misses;
        sample.max_step_us = max_step_us;

        portENTER_CRITICAL(&g_imu_mux);
        g_imu = sample;
        portEXIT_CRITICAL(&g_imu_mux);

        controlStep(sample);
        vTaskDelayUntil(&last_wake, period_ticks);
    }
}

void printMarkerJson(const char* name,
                     const WhiteMarkerObservation& marker,
                     float zero_x_px,
                     bool zero_ready) {
    const FootAngleEstimate angle =
        estimateFootAngle(marker, zero_x_px, zero_ready);

    Serial.printf(
        "\"%s\":{\"valid\":%s,\"source\":\"%s\",\"id\":%d,"
        "\"cx_px\":%.3f,\"cy_px\":%.1f,"
        "\"foot_angle_deg\":%.3f,\"angle_valid\":%s,"
        "\"angle_in_range\":%s,\"zero_x_px\":%.3f,"
        "\"peak_x_px\":%d,\"peak_contrast\":%.2f,"
        "\"weight_sum\":%.2f,\"bright_width_px\":%d,"
        "\"detect_ok\":%u,\"detect_fail\":%u,\"vision_us\":%u}",
        name,
        marker.valid ? "true" : "false",
        marker.valid ? "white1d" : "none",
        marker.id,
        marker.center_x_px,
        marker.center_y_px,
        angle.angle_deg,
        angle.valid ? "true" : "false",
        angle.in_calibration_range ? "true" : "false",
        angle.zero_x_px,
        marker.peak_x_px,
        marker.peak_contrast,
        marker.weight_sum,
        marker.bright_width_px,
        marker.success_count,
        marker.fail_count,
        marker.processing_us);
}

const char* autoZeroStateName(const ImuTelemetry& imu,
                              const WhiteMarkerObservation& a,
                              const WhiteMarkerObservation& b) {
    if (g_auto_zero.ready) return "locked";
    if (!imu.enabled) return "imu_unavailable";
    if (!imu.upright_candidate) return "waiting_upright";
    if (!a.valid || !b.valid) return "waiting_markers";
    return "collecting";
}

void printTelemetry(const CameraFrame& frame,
                    const WhiteMarkerObservation& a,
                    const WhiteMarkerObservation& b,
                    uint32_t total_vision_us,
                    const ImuTelemetry& imu) {
    Serial.printf(
        "{\"t_us\":%llu,\"frame\":%u,\"frame_t_us\":%llu,"
        "\"frame_dt_us\":%u,\"camera_failures\":%u,"
        "\"camera\":{\"width\":%d,\"height\":%d,\"bytes\":%u},"
        "\"vision_mode\":\"white_sparse_1d\","
        "\"angle_mode\":\"body_relative_foot_auto_zero_v2\","
        "\"vision_total_us\":%u,\"vision_max_us\":%u,"
        "\"zeroing\":{\"ready\":%s,\"state\":\"%s\","
        "\"samples\":%u,\"stable_ms\":%u,"
        "\"zero_a_px\":%.3f,\"zero_b_px\":%.3f},"
        "\"imu\":{\"enabled\":%s,\"sample_t_us\":%llu,"
        "\"loops\":%u,\"misses\":%u,\"max_step_us\":%u,"
        "\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
        "\"accel_norm_g\":%.5f,\"gyro_norm_dps\":%.5f,"
        "\"body_tilt_acc_deg\":%.3f,\"body_tilt_cf_deg\":%.3f,"
        "\"tilt_static\":%s,\"upright_error_deg\":%.3f,"
        "\"upright_candidate\":%s,\"upright_stable_ms\":%u},",
        static_cast<unsigned long long>(esp_timer_get_time()),
        g_frame_count,
        static_cast<unsigned long long>(frame.timestamp_us),
        g_frame_dt_us,
        g_camera_failures,
        frame.width,
        frame.height,
        static_cast<unsigned>(frame.length),
        total_vision_us,
        g_max_vision_us,
        g_auto_zero.ready ? "true" : "false",
        autoZeroStateName(imu, a, b),
        g_auto_zero.samples,
        imu.upright_stable_ms,
        g_auto_zero.zero_a_x,
        g_auto_zero.zero_b_x,
        imu.enabled ? "true" : "false",
        static_cast<unsigned long long>(imu.sample_timestamp_us),
        imu.loop_count,
        imu.deadline_misses,
        imu.max_step_us,
        imu.ax, imu.ay, imu.az,
        imu.gx, imu.gy, imu.gz,
        imu.accel_norm_g,
        imu.gyro_norm_dps,
        imu.body_tilt_acc_deg,
        imu.body_tilt_cf_deg,
        imu.tilt_static ? "true" : "false",
        imu.upright_error_deg,
        imu.upright_candidate ? "true" : "false",
        imu.upright_stable_ms);

    printMarkerJson(
        "marker_a", a,
        g_auto_zero.zero_a_x,
        g_auto_zero.ready);
    Serial.print(",");
    printMarkerJson(
        "marker_b", b,
        g_auto_zero.zero_b_x,
        g_auto_zero.ready);
    Serial.println("}");
}

} // namespace

void setup() {
    Serial.begin(921600);
    delay(300);

    Serial.println();
    Serial.println("AtomS3R Foot Angle Tracker boot");
    Serial.println("Upper white marker=A / lower white marker=B");
    Serial.println(
        "Angle definition: foot relative to body; auto-tared upright=0 deg");
    Serial.println("Positive direction: marker X moves left");
    Serial.printf(
        "Auto zero: gravity -X within %.1f deg, |a|-1g <= %.3f g, "
        "gyro <= %.1f dps, stable >= %u ms\n",
        appcfg::kAutoZeroMaxUprightErrorDeg,
        appcfg::kAutoZeroAccelNormToleranceG,
        appcfg::kAutoZeroMaxGyroDps,
        appcfg::kAutoZeroStableMs);
    Serial.printf(
        "Calibration slopes: A=%.9f deg/px, B=%.9f deg/px\n",
        appcfg::kFootAngleADegPerPx,
        appcfg::kFootAngleBDegPerPx);

    if (!psramFound()) {
        Serial.println("FATAL: PSRAM not detected");
        while (true) delay(1000);
    }

    if (!g_camera.begin()) {
        Serial.printf("FATAL: camera init: %s\n", g_camera.lastError());
        while (true) delay(1000);
    }

    M5.In_I2C.setPort(I2C_NUM_1, GPIO_NUM_45, GPIO_NUM_0);
    const bool imu_ok =
        M5.Imu.begin(&M5.In_I2C, m5::board_t::board_M5AtomS3RCam);

    Serial.printf(
        "IMU init=%s, type=%d\n",
        imu_ok ? "ok" : "failed",
        static_cast<int>(M5.Imu.getType()));

    if (!imu_ok || !M5.Imu.isEnabled()) {
        Serial.println(
            "WARNING: BMI270 unavailable; cx_px continues but "
            "auto-zero cannot lock and angle_valid stays false");
    }

    const BaseType_t created = xTaskCreatePinnedToCore(
        imuControlTask,
        "imu_control_200hz",
        4096,
        nullptr,
        configMAX_PRIORITIES - 3,
        &g_imu_task,
        0);

    if (created != pdPASS) {
        Serial.println("FATAL: could not create IMU/control task");
        while (true) delay(1000);
    }

    Serial.println(
        "READY: hold body upright and still until AUTO_ZERO_LOCKED");
}

void loop() {
    CameraFrame frame;
    if (!g_camera.capture(frame)) {
        ++g_camera_failures;
        delay(2);
        return;
    }

    ++g_frame_count;

    if (g_last_frame_timestamp_us &&
        frame.timestamp_us > g_last_frame_timestamp_us) {
        const uint64_t dt =
            frame.timestamp_us - g_last_frame_timestamp_us;
        g_frame_dt_us =
            dt > 0xffffffffULL
                ? 0xffffffffU
                : static_cast<uint32_t>(dt);
    }

    g_last_frame_timestamp_us = frame.timestamp_us;

    const uint32_t t0 = micros();

    const WhiteMarkerObservation a =
        g_tracker_a.process(frame.data);
    const WhiteMarkerObservation b =
        g_tracker_b.process(frame.data);

    const uint32_t vision_us = micros() - t0;
    if (vision_us > g_max_vision_us) {
        g_max_vision_us = vision_us;
    }

    const ImuTelemetry imu = getImuSnapshot();
    updateAutoZero(imu, a, b);

    g_camera.release();

    const uint32_t now_ms = millis();
    if (now_ms - g_last_telemetry_ms >=
        appcfg::kTelemetryPeriodMs) {
        g_last_telemetry_ms = now_ms;
        printTelemetry(frame, a, b, vision_us, imu);
    }

    delay(1);
}
