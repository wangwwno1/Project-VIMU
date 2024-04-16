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
    if (_curr_ref_accel.timestamp_sample == 0 || hrt_elapsed_time(&_curr_ref_accel.timestamp_sample) >= 20_ms) {
        _accel_filter.reset(Vector3f(accel.x, accel.y, accel.z));
        return;
    } else {
        _accel_filter.update(Vector3f(accel.x, accel.y, accel.z));
    }

    Vector3f error_residuals{0.f, 0.f, 0.f};
    const Vector3f filter_state = _accel_filter.getState();
    error_residuals(0) = filter_state(0) - _curr_ref_accel.x;
    error_residuals(1) = filter_state(1) - _curr_ref_accel.y;
    error_residuals(2) = filter_state(2) - _curr_ref_accel.z;

    const float inv_acc_noise = 1.f / fmaxf(_param_iv_acc_noise.get(), 0.01f);
    const Vector3f error_ratio = error_residuals * inv_acc_noise;
    _accel_validator.validate(error_ratio);

    if (attack_enabled(sensor_attack::ATK_MASK_ACCEL, accel.timestamp_sample)
        && _param_iv_delay_mask.get() & sensor_attack::ATK_MASK_ACCEL
        && _param_iv_ttd_delay_ms.get() > 0) {
        // Use delay for precise Time to Detection
        if (hrt_elapsed_time(&_attack_timestamp) >= (hrt_abstime) (_param_iv_ttd_delay_ms.get() * 1000)) {
            // Declare accel failure immediately by add error count
            accel.error_count = fmaxf(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }
    } else if (_accel_validator.test_ratio() >= 1.f) {
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
