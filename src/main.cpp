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

float wrapAngleDeg(float deg) {
    while (deg > 180.0f) deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
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
                     const WhiteMarkerObservation& marker) {
    const FootAngleEstimate angle =
        estimateFootAngle(marker);

    Serial.printf(
        "\"%s\":{\"valid\":%s,\"source\":\"%s\",\"id\":%d,"
        "\"cx_px\":%.3f,\"cy_px\":%.1f,"
        "\"foot_angle_deg\":%.3f,\"angle_valid\":%s,"
        "\"angle_in_range\":%s,"
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
        marker.peak_x_px,
        marker.peak_contrast,
        marker.weight_sum,
        marker.bright_width_px,
        marker.success_count,
        marker.fail_count,
        marker.processing_us);
}

void printTelemetry(const CameraFrame& frame,
                    const WhiteMarkerObservation& a,
                    const WhiteMarkerObservation& b,
                    uint32_t total_vision_us) {
    ImuTelemetry imu;
    portENTER_CRITICAL(&g_imu_mux);
    imu = g_imu;
    portEXIT_CRITICAL(&g_imu_mux);

    Serial.printf(
        "{\"t_us\":%llu,\"frame\":%u,\"frame_t_us\":%llu,"
        "\"frame_dt_us\":%u,\"camera_failures\":%u,"
        "\"camera\":{\"width\":%d,\"height\":%d,\"bytes\":%u},"
        "\"vision_mode\":\"white_sparse_1d\","
        "\"angle_mode\":\"body_relative_foot_upright_zero_v1\","
        "\"vision_total_us\":%u,\"vision_max_us\":%u,"
        "\"imu\":{\"enabled\":%s,\"sample_t_us\":%llu,"
        "\"loops\":%u,\"misses\":%u,\"max_step_us\":%u,"
        "\"ax\":%.5f,\"ay\":%.5f,\"az\":%.5f,"
        "\"gx\":%.5f,\"gy\":%.5f,\"gz\":%.5f,"
        "\"accel_norm_g\":%.5f,\"gyro_norm_dps\":%.5f,"
        "\"body_tilt_acc_deg\":%.3f,\"body_tilt_cf_deg\":%.3f,"
        "\"tilt_static\":%s},",
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
        imu.tilt_static ? "true" : "false");

    printMarkerJson("marker_a", a);
    Serial.print(",");
    printMarkerJson("marker_b", b);
    Serial.println("}");
}

} // namespace

void setup() {
    Serial.begin(921600);
    delay(300);

    Serial.println();
    Serial.println("AtomS3R Foot Angle Tracker boot");
    Serial.println("Upper white marker=A / lower white marker=B");
    Serial.println("Angle definition: foot relative to body; upright initial posture=0 deg");
    Serial.println("Positive direction: marker X moves left");
    Serial.printf(
        "Calibration A: theta=%.9f*(%.6f-x) deg\n",
        appcfg::kFootAngleADegPerPx,
        appcfg::kFootAngleAZeroXPx);
    Serial.printf(
        "Calibration B: theta=%.9f*(%.6f-x) deg\n",
        appcfg::kFootAngleBDegPerPx,
        appcfg::kFootAngleBZeroXPx);

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
            "WARNING: BMI270 unavailable; camera foot angle still works");
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
        "READY: QVGA grayscale / white1d / foot_angle_deg / 921600 baud");
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

    g_camera.release();

    const uint32_t now_ms = millis();
    if (now_ms - g_last_telemetry_ms >=
        appcfg::kTelemetryPeriodMs) {
        g_last_telemetry_ms = now_ms;
        printTelemetry(frame, a, b, vision_us);
    }

    delay(1);
}
