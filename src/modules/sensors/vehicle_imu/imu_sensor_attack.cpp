//
// Created by Robert Wang on 2022/8/22.
//

#include "VehicleIMU.hpp"

namespace sensors {
    bool VehicleIMU::attack_enabled(const uint8_t &attack_type) {

        return _attack_flag_prev & attack_type && _attack_timestamp != 0 &&
               hrt_absolute_time() >= _attack_timestamp;
    }

    void VehicleIMU::ApplyGyroAttack(sensor_gyro_s &gyro) {
        if (attack_enabled(sensor_attack::ATK_MASK_GYRO)) {
            // Apply Non-Stealthy Attack to Roll Axis
            gyro.x += _param_atk_gyr_bias.get();
        }
    }

    void VehicleIMU::ApplyGyroAttack(sensor_gyro_s &gyro, const sensor_gyro_s &ref_gyro) {
        // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
        if (attack_enabled(sensor_attack::ATK_MASK_GYRO)) {
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

            Vector3f extra_offset{0.f, 0.f, 0.f};
            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_gyr_twin_h.get() > 0.f) && (_param_iv_gyr_rst_cnt.get() >= 1)) {
                const float twin_deviation = _param_iv_gyr_twin_h.get() / _param_iv_gyr_rst_cnt.get();
                if (!PX4_ISFINITE(max_deviation) || twin_deviation <= max_deviation) {
                    max_deviation = twin_deviation;
                }
            }

            if (!PX4_ISFINITE(max_deviation)) {
                // No applicable stealthy attack, fallback to non-stealthy (overt) attack
                ApplyGyroAttack(gyro);
            } else {
                max_deviation = .99f * max_deviation * _param_iv_gyr_noise.get();
                // Inject deviation at X axis
                gyro.x = ref_gyro.x + max_deviation;
                // Ensure other axis is under control
                gyro.y = math::constrain(gyro.y, ref_gyro.y - max_deviation, ref_gyro.y + max_deviation);
                gyro.z = math::constrain(gyro.z, ref_gyro.z - max_deviation, ref_gyro.z + max_deviation);
            }
        }
    }

    void VehicleIMU::ApplyAccelAttack(sensor_accel_s &accel) {
        if (attack_enabled(sensor_attack::ATK_MASK_ACCEL)) {
            // Apply Non-Stealthy Attack to X Axis
            accel.x += _param_atk_acc_bias.get();
        }
    }

    void VehicleIMU::ApplyAccelAttack(sensor_accel_s &accel, const sensor_accel_s &ref_accel) {
        // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
        if (attack_enabled(sensor_attack::ATK_MASK_ACCEL)) {
            const uint8_t type_mask = _param_atk_stealth_type.get();

            // Which stealthy?
            float max_deviation = NAN;
            if (type_mask & sensor_attack::DET_CUSUM && (_param_iv_acc_mshift.get() > 0.f)) {
                max_deviation = _param_iv_acc_mshift.get();
            }

            if (type_mask & sensor_attack::DET_EWMA && (_param_iv_acc_ema_h.get() > 0.f)) {
                if (PX4_ISFINITE(max_deviation)) {
                    max_deviation = fminf(max_deviation, _param_iv_acc_ema_h.get());
                } else {
                    max_deviation = _param_iv_acc_ema_h.get();
                }
            }

            // We do not apply ema stealthy attack because we won't use ema detector for accelerometer
            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_acc_twin_h.get() > 0.f) && (_param_iv_acc_rst_cnt.get() >= 1)) {
                const float twin_deviation = _param_iv_acc_twin_h.get() / _param_iv_acc_rst_cnt.get();
                if (!PX4_ISFINITE(max_deviation) || twin_deviation <= max_deviation) {
                    max_deviation = twin_deviation;
                }
            }

            if (!PX4_ISFINITE(max_deviation)) {
                // No applicable stealthy attack, fallback to non-stealthy (overt) attack
                ApplyAccelAttack(accel);
            } else {
                max_deviation = .99f * max_deviation * _param_iv_acc_noise.get();
                // Inject deviation at X axis
                accel.x = ref_accel.x + max_deviation;
                // Ensure other axis is under control
                accel.y = math::constrain(accel.y, ref_accel.y - max_deviation, ref_accel.y + max_deviation);
                accel.z = math::constrain(accel.z, ref_accel.z - max_deviation, ref_accel.z + max_deviation);
            }
        }
    }

} // namespace sensors