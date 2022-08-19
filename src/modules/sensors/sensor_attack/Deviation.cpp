//
// Created by Robert Wang on 2022/7/27.
//

#include "Deviation.h"

namespace sensor_attack {
    Deviation::Deviation(DeviationParams *params) : _params(params) {
        reset();
    }

    void Deviation::reset() {
        start_time = 0;
    }

    Vector3f Deviation::calculate_deviation(const hrt_abstime &now) {
        float delta_time = 0.f;
        if (start_time == 0) {
            start_time = now;
        } else {
            delta_time = 1.e-6 * double(now - start_time);
        }

        const float deviation_length = math::min(get_deviation_length(delta_time), _params->max_deviation);
        const float heading = math::radians(_params->heading_deg);
        const float pitch = math::radians(_params->pitch_deg);
        const Vector3f deviation_vector{
                deviation_length * cos(heading) * cos(pitch),
                deviation_length * sin(heading) * cos(pitch),
                deviation_length * sin(pitch)
        };

        return deviation_vector;
    }

    float BiasDeviation::get_deviation_length(float delta_time) {
        return _params->initial_value;
    }

    float LinearDeviation::get_deviation_length(float delta_time) {
        delta_time = math::min(delta_time, time_to_max_deviation());
        return _params->initial_value + _params->rate_of_rise * delta_time;
    }

    float LinearDeviation::time_to_max_deviation() {
        if (_params->rate_of_rise > 0.f) {
            return math::max((_params->max_deviation - _params->initial_value) / _params->rate_of_rise, 0.f);
        } else {
            return INFINITY;
        }
    }

    float ExponentialDeviation::get_deviation_length(float delta_time) {
        float time_to_max_dev_s = time_to_max_deviation();
        if (PX4_ISFINITE(time_to_max_dev_s)) {
            delta_time = math::min(delta_time, time_to_max_dev_s);
        }
        return _params->initial_value * pow(_params->rate_of_rise, delta_time);
    }

    float ExponentialDeviation::time_to_max_deviation() {
        if ((_params->rate_of_rise > 1.f) && (_params->max_deviation > 0.f) && (_params->initial_value > 0.f)) {
            return math::max(log(_params->max_deviation / _params->initial_value) / log(_params->rate_of_rise), 0.f);
        } else {
            return NAN;
        }
    }


} // sensor_attack