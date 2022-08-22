//
// Created by Robert Wang on 2022/8/22.
//

#include "VehicleIMU.hpp"

using namespace matrix;

namespace sensors {
    void VehicleIMU::ValidateGyroData(sensor_gyro_s &gyro) {

        sensor_gyro_s ref_gyro{};
        if (_reference_gyro_sub.copy(&ref_gyro)) {

            ApplyGyroAttack(gyro, ref_gyro);

            _last_gyro_residual(0) = gyro.x - ref_gyro.x;
            _last_gyro_residual(1) = gyro.y - ref_gyro.y;
            _last_gyro_residual(2) = gyro.z - ref_gyro.z;
            const float inv_gyr_noise = 1.f / fmaxf(_param_iv_gyr_noise.get(), 0.01f);

            _gyro_validator.validate(_last_gyro_residual * inv_gyr_noise);
            if (_gyro_validator.test_ratio() > 1.f) {
                // Declare gyro failure immediately by add error count
                gyro.error_count = math::max(gyro.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
            }
        } else {
            _last_gyro_residual.zero();
            ApplyGyroAttack(gyro);
        }
    }

    void VehicleIMU::ValidateAccelData(sensor_accel_s &accel) {

        sensor_accel_s ref_accel{};
        if (_reference_accel_sub.copy(&ref_accel)) {

            ApplyAccelAttack(accel, ref_accel);
            
            _last_accel_residual(0) = accel.x - ref_accel.x;
            _last_accel_residual(1) = accel.y - ref_accel.y;
            _last_accel_residual(2) = accel.z - ref_accel.z;
            const float inv_acc_noise = 1.f / fmaxf(_param_iv_acc_noise.get(), 0.01f);

            _accel_validator.validate(_last_accel_residual * inv_acc_noise);
            if (_accel_validator.test_ratio() > 1.f) {
                // Declare accel failure immediately by add error count
                accel.error_count = math::max(accel.error_count + NORETURN_ERRCOUNT, NORETURN_ERRCOUNT + 1U);
            }
        } else {
            _last_accel_residual.zero();
            ApplyAccelAttack(accel);
        }
    }
}  // namespace sensors