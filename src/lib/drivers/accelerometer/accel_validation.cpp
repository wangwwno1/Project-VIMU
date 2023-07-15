//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Accelerometer.hpp"

using namespace matrix;

void PX4Accelerometer::updateReference(const hrt_abstime &timestamp_sample) {
    while (_next_ref_accel.timestamp_sample <= timestamp_sample) {
        _curr_ref_accel = _next_ref_accel;
        if (!_reference_accel_sub.update(&_next_ref_accel)) {
            break;
        }
    }
}

void PX4Accelerometer::validateAccel(sensor_accel_s &accel) {
    // Update until the reference is catch up accel
    if (_curr_ref_accel.timestamp_sample == 0 || hrt_elapsed_time(&_curr_ref_accel.timestamp_sample) >= 20000) {
        return;
    }

    // residuals = measurement - reference
    Vector3f error_residuals{-_curr_ref_accel.x, -_curr_ref_accel.y, -_curr_ref_accel.z};
    if ((accel.timestamp_sample > _curr_ref_accel.timestamp_sample) &&
        (_next_ref_accel.timestamp_sample > _curr_ref_accel.timestamp_sample)) {
        // linear interpolate the reference
        const float interval = 1.e-6f * (_next_ref_accel.timestamp_sample - _curr_ref_accel.timestamp_sample);
        const float dt = 1.e-6f * (accel.timestamp_sample - _curr_ref_accel.timestamp_sample);
        const float weight = dt / interval;
        const Vector3f next_ref{-_next_ref_accel.x, -_next_ref_accel.y, -_next_ref_accel.z};
        // error_residuals = (1.f - weight) * error_residuals + weight * next_ref;
        error_residuals += weight * (next_ref - error_residuals);
    }

    error_residuals(0) += accel.x;
    error_residuals(1) += accel.y;
    error_residuals(2) += accel.z;

    const Vector3f error_ratio = error_residuals * _inv_acc_noise;
    _accel_validator.validate(error_ratio);

    if (_accel_validator.test_ratio() >= 1.f) {
        // Declare accel failure immediately by add error count
        accel.error_count = fmaxf(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
    }

    // Record error ratio and test ratio for debug and post-mortem analysis
    sensor_accel_errors_s accel_error{};

    accel_error.device_id = accel.device_id;
    accel_error.samples = accel.samples;
    accel_error.x = error_residuals(0);
    accel_error.y = error_residuals(1);
    accel_error.z = error_residuals(2);
    accel_error.test_ratio = _accel_validator.test_ratio();
    accel_error.timestamp_reference = _curr_ref_accel.timestamp_sample;
    accel_error.timestamp = hrt_absolute_time();
    _sensor_accel_errors_pub.publish(accel_error);
}
