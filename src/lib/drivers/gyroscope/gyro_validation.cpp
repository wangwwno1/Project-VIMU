//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Gyroscope.hpp"

using namespace matrix;

void PX4Gyroscope::updateReference(const hrt_abstime &timestamp_sample) {
    while (_next_ref_gyro.timestamp_sample <= timestamp_sample) {
        _curr_ref_gyro = _next_ref_gyro;
        if (!_reference_gyro_sub.update(&_next_ref_gyro)) {
            break;
        }
    }
}

void PX4Gyroscope::validateGyro(sensor_gyro_s &gyro) {
    // Update until the reference is catch up accel
    if (_curr_ref_gyro.timestamp_sample == 0 || hrt_elapsed_time(&_curr_ref_gyro.timestamp_sample) >= 20000) {
        return;
    }

    // residuals = measurement - reference
    Vector3f error_residuals{-_curr_ref_gyro.x, -_curr_ref_gyro.y, -_curr_ref_gyro.z};
    if ((gyro.timestamp_sample > _curr_ref_gyro.timestamp_sample) &&
        (_next_ref_gyro.timestamp_sample > _curr_ref_gyro.timestamp_sample)) {
        // linear interpolate the reference
        const float interval = 1.e-6f * (_next_ref_gyro.timestamp_sample - _curr_ref_gyro.timestamp_sample);
        const float dt = 1.e-6f * (gyro.timestamp_sample - _curr_ref_gyro.timestamp_sample);
        const float weight = dt / interval;
        const Vector3f next_ref{-_next_ref_gyro.x, -_next_ref_gyro.y, -_next_ref_gyro.z};
        // error_residuals = (1.f - weight) * error_residuals + weight * next_ref;
        error_residuals += weight * (next_ref - error_residuals);
    }

    error_residuals(0) += gyro.x;
    error_residuals(1) += gyro.y;
    error_residuals(2) += gyro.z;

    const Vector3f error_ratio = error_residuals * _inv_gyro_noise;
    _gyro_validator.validate(error_ratio);

    if (_gyro_validator.test_ratio() >= 1.f) {
        // Declare gyro failure immediately by add error count
        gyro.error_count = fmaxf(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
    }

    // Record error ratio and test ratio for debug and post-mortem analysis
    sensor_gyro_errors_s gyro_error{};

    gyro_error.device_id = gyro.device_id;
    gyro_error.samples = gyro.samples;
    gyro_error.x = error_residuals(0);
    gyro_error.y = error_residuals(1);
    gyro_error.z = error_residuals(2);
    gyro_error.test_ratio = _gyro_validator.test_ratio();
    gyro_error.timestamp_reference = _curr_ref_gyro.timestamp_sample;
    gyro_error.timestamp = hrt_absolute_time();
    _sensor_gyro_errors_pub.publish(gyro_error);
}
