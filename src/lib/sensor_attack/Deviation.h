//
// Created by Robert Wang on 2022/7/27.
//

#pragma once

#include "common.h"

using matrix::Vector3f;

namespace sensor_attack {

    class Deviation {
    public:
        Deviation() = delete;
        Deviation(DeviationParams *params);
        ~Deviation() = default;

        void reset();
        Vector3f calculate_deviation(const hrt_abstime &now);
        virtual float time_to_max_deviation() = 0;

    protected:
        virtual float get_deviation_length(float delta_time) = 0;
        hrt_abstime         start_time{0};
        DeviationParams     *_params;
    };

    class BiasDeviation : public Deviation {
    public:
        BiasDeviation() = delete;
        BiasDeviation(DeviationParams *params) : Deviation(params) {}
        ~BiasDeviation() = default;
        float time_to_max_deviation() override { return 0.f; }

    protected:
        float get_deviation_length(float delta_time) override;
    };

    class LinearDeviation : public Deviation {
    public:
        LinearDeviation() = delete;
        LinearDeviation(DeviationParams *params) : Deviation(params) {}
        ~LinearDeviation() = default;
        float time_to_max_deviation() override;
    protected:
        float get_deviation_length(float delta_time) override;
    };

    class ExponentialDeviation : public Deviation {
    public:
        ExponentialDeviation() = delete;
        ExponentialDeviation(DeviationParams *params) : Deviation(params) {}
        ~ExponentialDeviation() = default;
        float time_to_max_deviation() override;
    protected:
        float get_deviation_length(float delta_time) override;
    };

    Deviation* CreateAttackInstance(const int32_t attack_class, DeviationParams *params);

} // sensor_attack

