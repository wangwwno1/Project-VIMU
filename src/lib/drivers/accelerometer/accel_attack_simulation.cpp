//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Accelerometer.hpp"

bool PX4Accelerometer::attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const {
    return _attack_flag_prev & attack_type && _attack_timestamp != 0 && timestamp_sample >= _attack_timestamp;
}

void PX4Accelerometer::applyAccelAttack(sensor_accel_s &accel) {
    // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
    if (attack_enabled(sensor_attack::ATK_MASK_ACCEL, accel.timestamp_sample)) {
        const float max_deviation = getMaxDeviation();
        if (PX4_ISFINITE(max_deviation) && _curr_ref_accel.timestamp_sample >= _attack_timestamp) {
            _last_deviation[0] = _curr_ref_accel.x + max_deviation - accel.x;
            _last_deviation[1] = math::constrain(accel.y, _curr_ref_accel.y - max_deviation, _curr_ref_accel.y + max_deviation) - accel.y;
            _last_deviation[2] = math::constrain(accel.z, _curr_ref_accel.z - max_deviation, _curr_ref_accel.z + max_deviation) - accel.z;
        } else {
            _last_deviation[0] = _param_atk_acc_bias.get();
            _last_deviation[1] = 0.f;
            _last_deviation[2] = 0.f;
        }

        accel.x += _last_deviation[0];
        accel.y += _last_deviation[1];
        accel.z += _last_deviation[2];

    } else {
        _last_deviation[0] = 0.f;
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;
    }
}

void PX4Accelerometer::applyAccelAttack(sensor_accel_s &accel, sensor_accel_fifo_s &accel_fifo) {
    // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
    applyAccelAttack(accel);

    if (attack_enabled(sensor_attack::ATK_MASK_ACCEL, accel_fifo.timestamp_sample)) {
        // Also apply the deviation to FIFO samples
        const uint8_t N = accel_fifo.samples;
        for (int n = 0; n < N; n++) {
            accel_fifo.x[n] += static_cast<int16_t>(_last_deviation[0] / _scale);
            accel_fifo.y[n] += static_cast<int16_t>(_last_deviation[1] / _scale);
            accel_fifo.z[n] += static_cast<int16_t>(_last_deviation[2] / _scale);
        }
    }
}

float PX4Accelerometer::getMaxDeviation() const {
    // Which stealthy?
    if (_param_atk_stealth_type.get() == sensor_attack::NO_STEALTHY) {
        return NAN;
    } else {
        return _param_atk_acc_bias.get();
    }
}
