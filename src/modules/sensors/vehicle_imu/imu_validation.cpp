//
// Created by Robert Wang on 2022/8/22.
//

#include "VehicleIMU.hpp"

using namespace matrix;

namespace sensors {
    void VehicleIMU::ValidateGyroData(sensor_gyro_s &gyro) {
        const hrt_abstime interval_us = static_cast<hrt_abstime>(_imu_integration_interval_us * 1.5f);
        const hrt_abstime validate_start = hrt_absolute_time();

        sensor_gyro_s ref_gyro{};
        if (_reference_gyro_sub.update(&ref_gyro)) {
            if ((_last_ref_gyro.timestamp_sample == 0) ||
                (validate_start > (ref_gyro.timestamp_sample + _last_ref_gyro.timestamp_sample) / 2)) {
                // Use the newest reference as it closer to our current timestamp.
                _last_ref_gyro = ref_gyro;
            } else {
                // Use last reference, then update it
                sensor_gyro_s tmp{ref_gyro};
                ref_gyro = _last_ref_gyro;
                _last_ref_gyro = tmp;
            }
        }

        Vector3f error_residuals{0.f, 0.f, 0.f};
        if (math::isInRange(validate_start, ref_gyro.timestamp_sample - interval_us, ref_gyro.timestamp_sample + interval_us)) {

            ApplyGyroAttack(gyro, ref_gyro);

            error_residuals(0) = gyro.x - ref_gyro.x;
            error_residuals(1) = gyro.y - ref_gyro.y;
            error_residuals(2) = gyro.z - ref_gyro.z;

        } else {
            ApplyGyroAttack(gyro);
            return;
        }

        const float inv_gyr_noise = 1.f / fmaxf(_param_iv_gyr_noise.get(), 0.01f);
        const Vector3f error_ratio = error_residuals * inv_gyr_noise;
        _gyro_validator.validate(error_ratio);

        if (attack_enabled(sensor_attack::ATK_MASK_GYRO)
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


        if (_last_gyro_errors.samples < MAX_GYRO_SAMPLES && _param_iv_debug_log.get()) {
            // Record error ratio and test ratio for debug and post-mortem analysis
            const uint8_t idx = _last_gyro_errors.samples;
            if (idx == 0) {
                _last_gyro_errors.timestamp_start = hrt_absolute_time();
            }
            _last_gyro_errors.x[idx] = error_residuals(0);
            _last_gyro_errors.y[idx] = error_residuals(1);
            _last_gyro_errors.z[idx] = error_residuals(2);
            _last_gyro_errors.test_ratio[idx] = _gyro_validator.test_ratio();
            _last_gyro_errors.samples++;
        }
    }

    void VehicleIMU::ValidateAccelData(sensor_accel_s &accel) {
        const hrt_abstime interval_us = static_cast<hrt_abstime>(_imu_integration_interval_us * 1.5f);
        const hrt_abstime validate_start = hrt_absolute_time();

        sensor_accel_s ref_accel{};
        if (_reference_accel_sub.update(&ref_accel)) {
            if ((_last_ref_accel.timestamp_sample == 0) ||
                (validate_start > (ref_accel.timestamp_sample + _last_ref_accel.timestamp_sample) / 2)) {
                // Use the newest reference as it closer to our current timestamp.
                _last_ref_accel = ref_accel;
            } else {
                // Use last reference, then update it
                sensor_accel_s tmp{ref_accel};
                ref_accel = _last_ref_accel;
                _last_ref_accel = tmp;
            }
        }

        Vector3f error_residuals{0.f, 0.f, 0.f};
        if (math::isInRange(validate_start, ref_accel.timestamp_sample - interval_us, ref_accel.timestamp_sample + interval_us)) {

            ApplyAccelAttack(accel, ref_accel);

            error_residuals(0) = accel.x - ref_accel.x;
            error_residuals(1) = accel.y - ref_accel.y;
            error_residuals(2) = accel.z - ref_accel.z;

        } else {
            ApplyAccelAttack(accel);
            return;
        }

        const float inv_acc_noise = 1.f / fmaxf(_param_iv_acc_noise.get(), 0.01f);
        const Vector3f error_ratio = error_residuals * inv_acc_noise;
        _accel_validator.validate(error_ratio);

        if (attack_enabled(sensor_attack::ATK_MASK_ACCEL)
            && _param_iv_delay_mask.get() & sensor_attack::ATK_MASK_ACCEL
            && _param_iv_ttd_delay_ms.get() > 0) {
            // Use delay for precise Time to Detection
            if (hrt_elapsed_time(&_attack_timestamp) >= (hrt_abstime) (_param_iv_ttd_delay_ms.get() * 1000)) {
                // Declare gyro failure immediately by add error count
                accel.error_count = math::max(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
            }
        } else if (_accel_validator.test_ratio() >= 1.f) {
            // Declare accel failure immediately by add error count
            accel.error_count = math::max(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }

        if (_last_accel_errors.samples < MAX_ACCEL_SAMPLES && _param_iv_debug_log.get()) {
            // Record error ratio and test ratio for debug and post-mortem analysis
            const uint8_t idx = _last_accel_errors.samples;
            if (idx == 0) {
                _last_accel_errors.timestamp_start = hrt_absolute_time();
            }
            _last_accel_errors.x[idx] = error_residuals(0);
            _last_accel_errors.y[idx] = error_residuals(1);
            _last_accel_errors.z[idx] = error_residuals(2);
            _last_accel_errors.test_ratio[idx] = _accel_validator.test_ratio();
            _last_accel_errors.samples++;
        }
    }
}  // namespace sensors