//
// Created by Robert Wang on 2022/8/22.
//

#include "VehicleIMU.hpp"

using namespace matrix;

namespace sensors {
    void VehicleIMU::ValidateGyroData(sensor_gyro_s &gyro) {
        const hrt_abstime interval_us = _imu_integration_interval_us;
        const hrt_abstime validate_start = hrt_absolute_time();

        sensor_gyro_s ref_gyro{};
        if (_reference_gyro_sub.update(&ref_gyro)) {
            if ((_last_ref_gyro.timestamp_sample == 0) ||
                (validate_start > (ref_gyro.timestamp_sample + _last_ref_gyro.timestamp_sample) / 2)) {
                // Use the newest reference as it closer to our current timestamp.
                _last_ref_gyro = ref_gyro;
            } else {
                // Use last reference, then update it
                sensor_gyro_s tmp;
                tmp = ref_gyro;
                ref_gyro = _last_ref_gyro;
                _last_ref_gyro = tmp;
            }
        }

        if (math::isInRange(validate_start, ref_gyro.timestamp_sample - interval_us, ref_gyro.timestamp_sample + interval_us)) {

            ApplyGyroAttack(gyro, ref_gyro);

            _last_gyro_residual(0) = gyro.x - ref_gyro.x;
            _last_gyro_residual(1) = gyro.y - ref_gyro.y;
            _last_gyro_residual(2) = gyro.z - ref_gyro.z;

        } else {
            _last_gyro_residual.zero();
            ApplyGyroAttack(gyro);
        }

        const float inv_gyr_noise = 1.f / fmaxf(_param_iv_gyr_noise.get(), 0.01f);
        _gyro_validator.validate(_last_gyro_residual * inv_gyr_noise);
        if (_gyro_validator.test_ratio() > 1.f) {
            // Declare gyro failure immediately by add error count
            gyro.error_count = math::max(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }
    }

    void VehicleIMU::ValidateAccelData(sensor_accel_s &accel) {
        const hrt_abstime interval_us = _imu_integration_interval_us;
        const hrt_abstime validate_start = hrt_absolute_time();

        sensor_accel_s ref_accel{};
        if (_reference_accel_sub.update(&ref_accel)) {
            if ((_last_ref_accel.timestamp_sample == 0) ||
                (validate_start > (ref_accel.timestamp_sample + _last_ref_accel.timestamp_sample) / 2)) {
                // Use the newest reference as it closer to our current timestamp.
                _last_ref_accel = ref_accel;
            } else {
                // Use last reference, then update it
                sensor_accel_s tmp;
                tmp = ref_accel;
                ref_accel = _last_ref_accel;
                _last_ref_accel = tmp;
            }
        }

        if (math::isInRange(validate_start, ref_accel.timestamp_sample - interval_us, ref_accel.timestamp_sample + interval_us)) {

            ApplyAccelAttack(accel, ref_accel);
            
            _last_accel_residual(0) = accel.x - ref_accel.x;
            _last_accel_residual(1) = accel.y - ref_accel.y;
            _last_accel_residual(2) = accel.z - ref_accel.z;

        } else {
            _last_accel_residual.zero();
            ApplyAccelAttack(accel);
        }

        const float inv_acc_noise = 1.f / fmaxf(_param_iv_acc_noise.get(), 0.01f);
        _accel_validator.validate(_last_accel_residual * inv_acc_noise);
        if (_accel_validator.test_ratio() > 1.f) {
            // Declare accel failure immediately by add error count
            accel.error_count = math::max(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
        }
    }
}  // namespace sensors