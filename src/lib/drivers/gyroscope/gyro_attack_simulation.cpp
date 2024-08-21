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
        if (!_attack_has_start) {
            // Declare the attack is started
            _attack_has_start = 1;
            PX4_WARN("Debug - Invoke GYRO attack for instance %d at timestamp_sample: %" PRIu64,
                     get_instance(), gyro.timestamp_sample);
        }

        const float max_deviation = getMaxDeviation();
        if (PX4_ISFINITE(max_deviation) && _curr_ref_gyro.timestamp_sample >= _attack_timestamp) {
            _last_deviation[0] = _curr_ref_gyro.x + max_deviation - gyro.x;
            _last_deviation[1] = math::constrain(gyro.y, _curr_ref_gyro.y - max_deviation, _curr_ref_gyro.y + max_deviation) - gyro.y;
            _last_deviation[2] = math::constrain(gyro.z, _curr_ref_gyro.z - max_deviation, _curr_ref_gyro.z + max_deviation) - gyro.z;
        } else if (_param_atk_gyr_freq.get() > 1.e-4f) {
            static double attack_time_sec = 0.0;
            static float amp_offset = 0.f;

            attack_time_sec = static_cast<double>((gyro.timestamp_sample - _attack_timestamp) / 1.e6);
            amp_offset = _param_atk_gyr_amp.get() * cosf(static_cast<float>(2.0 * attack_time_sec) * M_PI_F * _param_atk_gyr_freq.get() + _param_atk_gyr_phase.get() * M_DEG_TO_RAD_F);
            _last_deviation[0] = amp_offset;
            _last_deviation[1] = amp_offset;
            _last_deviation[2] = amp_offset;
        } else {
            _last_deviation[0] = _param_atk_gyr_amp.get();
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
        return _param_atk_gyr_amp.get();
    }
}
