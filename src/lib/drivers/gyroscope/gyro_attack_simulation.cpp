//
// Created by Robert Wang on 2022/8/22.
//

#include "PX4Gyroscope.hpp"

bool PX4Gyroscope::attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const {
    return _attack_flag_prev & attack_type && _attack_timestamp != 0 && timestamp_sample >= _attack_timestamp;
}

void PX4Gyroscope::applyGyroAttack(sensor_gyro_s &gyro) {
    // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro.timestamp_sample)) {
        const float max_deviation = getMaxDeviation();
        if (PX4_ISFINITE(max_deviation) && _curr_ref_gyro.timestamp_sample >= _attack_timestamp) {
            _last_deviation[0] = _curr_ref_gyro.x + max_deviation - gyro.x;
            _last_deviation[1] = math::constrain(gyro.y, _curr_ref_gyro.y - max_deviation, _curr_ref_gyro.y + max_deviation) - gyro.y;
            _last_deviation[2] = math::constrain(gyro.z, _curr_ref_gyro.z - max_deviation, _curr_ref_gyro.z + max_deviation) - gyro.z;
        } else {
            _last_deviation[0] = _param_atk_gyr_bias.get();
            _last_deviation[1] = 0.f;
            _last_deviation[2] = 0.f;
        }

        gyro.x += _last_deviation[0];
        gyro.y += _last_deviation[1];
        gyro.z += _last_deviation[2];

    } else {
        _last_deviation[0] = 0.f;
        _last_deviation[1] = 0.f;
        _last_deviation[2] = 0.f;
    }
}

void PX4Gyroscope::applyGyroAttack(sensor_gyro_s &gyro, sensor_gyro_fifo_s &gyro_fifo) {
    // Attempt to Apply Stealthy Attack, else fall back to Non-Stealthy Attack
    applyGyroAttack(gyro);

    if (attack_enabled(sensor_attack::ATK_MASK_GYRO, gyro_fifo.timestamp_sample)) {
        // Also apply the deviation to FIFO samples
        const uint8_t N = gyro_fifo.samples;
        for (int n = 0; n < N; n++) {
            gyro_fifo.x[n] += static_cast<int16_t>(_last_deviation[0] / _scale);
            gyro_fifo.y[n] += static_cast<int16_t>(_last_deviation[1] / _scale);
            gyro_fifo.z[n] += static_cast<int16_t>(_last_deviation[2] / _scale);
        }
    }
}

float PX4Gyroscope::getMaxDeviation() const {
    // Which stealthy?
    if (_param_atk_stealth_type.get() == sensor_attack::NO_STEALTHY) {
        return NAN;
    } else {
        return _param_atk_gyr_bias.get();
    }
}
