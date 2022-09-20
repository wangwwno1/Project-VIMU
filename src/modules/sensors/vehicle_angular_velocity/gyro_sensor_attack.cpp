//
// Created by Robert Wang on 2022/9/6.
//

#include "VehicleAngularVelocity.hpp"

namespace sensors {
    bool VehicleAngularVelocity::attack_enabled() const {
        return _apply_gyro_attack && (hrt_absolute_time() >= _attack_timestamp);
    }

    void VehicleAngularVelocity::ApplyGyroAttack(matrix::Vector3f &angular_velocity_uncalibrated) {
        if (attack_enabled()) {
            // Apply Non-Stealthy Attack to Roll Axis
            angular_velocity_uncalibrated(0) += _param_atk_gyr_bias.get();
        }
    }

    void VehicleAngularVelocity::ApplyGyroAttack(matrix::Vector3f &angular_velocity_uncalibrated,
                                                 const matrix::Vector3f &reference_angular_velocity) {
        // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
        if (attack_enabled()) {
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

            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_gyr_l1tw_h.get() > 0.f) && (_param_iv_gyr_rst_cnt.get() >= 1)) {
                const float l1tw_deviation = _param_iv_gyr_l1tw_h.get() / _param_iv_gyr_rst_cnt.get();
                if (PX4_ISFINITE(max_deviation)) {
                    max_deviation = fminf(max_deviation, l1tw_deviation);
                } else {
                    max_deviation = l1tw_deviation;
                }
            }

            if (!PX4_ISFINITE(max_deviation)) {
                // No applicable stealthy attack, fallback to non-stealthy (overt) attack
                ApplyGyroAttack(angular_velocity_uncalibrated);
            } else {
                max_deviation = max_deviation * _param_iv_gyr_noise.get();
                // Inject deviation at X axis
                angular_velocity_uncalibrated(0) = reference_angular_velocity(0) + .99f * max_deviation;
                // Ensure other axis is under control
                angular_velocity_uncalibrated(1) = math::constrain(angular_velocity_uncalibrated(1),
                                         reference_angular_velocity(1) - .99f * max_deviation,
                                         reference_angular_velocity(1) + .99f * max_deviation);
                angular_velocity_uncalibrated(2) = math::constrain(angular_velocity_uncalibrated(2),
                                         reference_angular_velocity(2) - .99f * max_deviation,
                                         reference_angular_velocity(2) + .99f * max_deviation);
            }
        }
    }
}  // sensors