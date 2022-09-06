//
// Created by Robert Wang on 2022/8/22.
//

#include "VehicleIMU.hpp"

namespace sensors {
    
    void VehicleIMU::ApplyGyroAttack(sensor_gyro_s &gyro) {
        const int inst_id = _vehicle_imu_pub.get_instance();
        const bool apply_attack = _param_atk_apply_type.get() & sensor_attack::ATK_MASK_GYRO;
        const bool is_affected = (inst_id >= 0) && (_param_atk_multi_imu.get() & (1 << inst_id));
        if (apply_attack && is_affected) {
            // Apply Non-Stealthy Attack to Roll Axis
            gyro.x += _param_atk_gyr_bias.get();
        }
    }

    void VehicleIMU::ApplyGyroAttack(sensor_gyro_s &gyro, const sensor_gyro_s &ref_gyro) {
        // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
        const int inst_id = _vehicle_imu_pub.get_instance();
        const bool apply_attack = _param_atk_apply_type.get() & sensor_attack::ATK_MASK_GYRO;
        const bool is_affected = (inst_id >= 0) && (_param_atk_multi_imu.get() & (1 << inst_id));
        if (apply_attack & is_affected) {
            const uint8_t type_mask = _param_atk_stealth_type.get();

            // Which stealthy?
            float max_deviation = NAN;
            if (type_mask & sensor_attack::DET_CUSUM && (_param_iv_gyr_mshift.get() > 0.f)) {
                max_deviation = _param_iv_gyr_mshift.get();
            }

            if (type_mask & sensor_attack::DET_EWMA && (_param_iv_gyr_ema_h.get() > 0.f)) {
                if (PX4_ISFINITE(max_deviation)) {
                    max_deviation = fminf(max_deviation, _param_iv_gyr_ema_h.get());
                } else {
                    max_deviation = _param_iv_gyr_ema_h.get();
                }
            }
            // TODO Time-Window Detection

            if (!PX4_ISFINITE(max_deviation)) {
                // No applicable stealthy attack, fallback to non-stealthy (overt) attack
                ApplyGyroAttack(gyro);
            } else {
                max_deviation = max_deviation * _param_iv_gyr_noise.get();
                // Inject deviation at X axis
                gyro.x = ref_gyro.x + .99f * max_deviation;
                // Ensure other axis is under control
                gyro.y = math::constrain(gyro.y, ref_gyro.y - .99f * max_deviation, ref_gyro.y + .99f * max_deviation);
                gyro.z = math::constrain(gyro.z, ref_gyro.z - .99f * max_deviation, ref_gyro.z + .99f * max_deviation);
            }
        }
    }

    void VehicleIMU::ApplyAccelAttack(sensor_accel_s &accel) {
        const int inst_id = _vehicle_imu_pub.get_instance();
        const bool apply_attack = _param_atk_apply_type.get() & sensor_attack::ATK_MASK_ACCEL;
        const bool is_affected = (inst_id >= 0) && (_param_atk_multi_imu.get() & (1 << inst_id));
        if (apply_attack && is_affected) {
            // Apply Non-Stealthy Attack to X Axis
            accel.x += _param_atk_acc_bias.get();
        }
    }

    void VehicleIMU::ApplyAccelAttack(sensor_accel_s &accel, const sensor_accel_s &ref_accel) {
        // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
        const int inst_id = _vehicle_imu_pub.get_instance();
        const bool apply_attack = _param_atk_apply_type.get() & sensor_attack::ATK_MASK_ACCEL;
        const bool is_affected = (inst_id >= 0) && (_param_atk_multi_imu.get() & (1 << inst_id));
        if (apply_attack & is_affected) {
            const uint8_t type_mask = _param_atk_stealth_type.get();

            // Which stealthy?
            float max_deviation = NAN;
            if (type_mask & sensor_attack::DET_CUSUM && (_param_iv_acc_mshift.get() > 0.f)) {
                max_deviation = _param_iv_acc_mshift.get();
            }
            // We do not apply other stealthy attack because we won't use other detectors

            if (!PX4_ISFINITE(max_deviation)) {
                // No applicable stealthy attack, fallback to non-stealthy (overt) attack
                ApplyAccelAttack(accel);
            } else {
                max_deviation = max_deviation * _param_iv_gyr_noise.get();
                // Inject deviation at X axis
                accel.x = ref_accel.x + .99f * max_deviation;
                // Ensure other axis is under control
                accel.y = math::constrain(accel.y, ref_accel.y - .99f * max_deviation, ref_accel.y + .99f * max_deviation);
                accel.z = math::constrain(accel.z, ref_accel.z - .99f * max_deviation, ref_accel.z + .99f * max_deviation);
            }
        }
    }

} // namespace sensors