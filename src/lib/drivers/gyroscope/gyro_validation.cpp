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

    // fixme remove
    if (gyro.timestamp_sample != _curr_ref_gyro.timestamp_sample) {
        PX4_WARN("%" PRIu64 ": %d - reference async: GYRO %" PRIu64 " vs. REF %" PRIu64 ")" ,
                 hrt_absolute_time(), get_instance(), gyro.timestamp_sample, _curr_ref_gyro.timestamp_sample);
    }

    Vector3f error_residuals{0.f, 0.f, 0.f};
    error_residuals(0) = gyro.x - _curr_ref_gyro.x;
    error_residuals(1) = gyro.y - _curr_ref_gyro.y;
    error_residuals(2) = gyro.z - _curr_ref_gyro.z;

    const Vector3f error_ratio = error_residuals * _inv_gyro_noise;
    _gyro_validator.validate(error_ratio);

    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro.timestamp_sample)
        && _param_iv_delay_mask.get() & sensor_attack::ATK_MASK_GYRO
        && _param_iv_ttd_delay_ms.get() > 0) {
        // Use delay for precise Time to Detection
        if (hrt_elapsed_time(&_attack_timestamp) >= (hrt_abstime) (_param_iv_ttd_delay_ms.get() * 1000)) {
            // Declare gyro failure immediately by add error count
            gyro.error_count = math::max(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }
    } else if (_gyro_validator.test_ratio() >= 1.f) {
        // Declare gyro failure immediately by add error count
        gyro.error_count = math::max(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
    }

    if (_enable_debug_log != 0) {
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
}
