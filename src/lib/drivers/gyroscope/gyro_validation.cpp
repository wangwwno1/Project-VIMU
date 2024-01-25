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

    if ((_last_angular_rates.timestamp != 0) && (timestamp_sample - _last_angular_rates.timestamp_sample <= 20_ms)) {
        const float dt = 1.e-6f * (timestamp_sample - _last_angular_rates.timestamp_sample);
        vehicle_angular_acceleration_s ref_ang_accel{};
        _reference_angular_acceleration_sub.copy(&ref_ang_accel);

        _last_angular_rates.xyz[0] += ref_ang_accel.xyz[0] * dt;
        _last_angular_rates.xyz[1] += ref_ang_accel.xyz[1] * dt;
        _last_angular_rates.xyz[2] += ref_ang_accel.xyz[2] * dt;
        _last_angular_rates.timestamp_sample = timestamp_sample;
    }
}

void PX4Gyroscope::validateGyro(sensor_gyro_s &gyro) {
    // Update until the reference is catch up accel
    if (_curr_ref_gyro.timestamp_sample == 0 || hrt_elapsed_time(&_curr_ref_gyro.timestamp_sample) >= 20_ms) {
        return;
    }

    Vector3f error_residuals{0.f, 0.f, 0.f};
    error_residuals(0) = gyro.x - _curr_ref_gyro.x;
    error_residuals(1) = gyro.y - _curr_ref_gyro.y;
    error_residuals(2) = gyro.z - _curr_ref_gyro.z;

    const float inv_gyr_noise = 1.f / fmaxf(_param_iv_gyr_noise.get(), 0.01f);
    const Vector3f error_ratio = error_residuals * inv_gyr_noise;

    // Simulate Stealthy Attack for CUSUM
    const float max_deviation = getMaxDeviation();
    Vector3f cusum_error_ratios{0.f, 0.f, 0.f};
    if (PX4_ISFINITE(max_deviation) && attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro.timestamp_sample)) {
       cusum_error_ratios.setAll(max_deviation * inv_gyr_noise);
    } else if (gyro.timestamp_sample - _last_angular_rates.timestamp_sample <= 20_ms) {
        const Vector3f ref_ang_vel{_last_angular_rates.xyz};
        const Vector3f gyro_meas{gyro.x, gyro.y, gyro.z};
        cusum_error_ratios = (gyro_meas - ref_ang_vel) * inv_gyr_noise;
    }
    _vehicle_angular_velocity_sub.update(&_last_angular_rates);  // Update reference angular rate after validate
    _gyro_validator.validate(cusum_error_ratios, error_ratio);

    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro.timestamp_sample)
        && _param_iv_delay_mask.get() & sensor_attack::ATK_MASK_GYRO
        && _param_iv_ttd_delay_ms.get() > 0) {
        // Use delay for precise Time to Detection
        if (hrt_elapsed_time(&_attack_timestamp) >= (hrt_abstime) (_param_iv_ttd_delay_ms.get() * 1000)) {
            // Declare gyro failure immediately by add error count
            gyro.error_count = fmaxf(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }
    } else if (_gyro_validator.test_ratio() >= 1.f) {
        // Declare gyro failure immediately by add error count
        gyro.error_count = fmaxf(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
    }

    if (_param_iv_debug_log.get()) {
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
