//
// Created by Robert Wang on 2022/5/20.
//

#pragma once

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <modules/ekf2/EKF/common.h>

using estimator::imuSample;

struct RateSample {
    hrt_abstime time_us{0};
    matrix::Vector3f angular_rate{0.f, 0.f, 0.f};
    float delta_ang_dt{0.f};
};

class RateDownSampler
{
public:
    explicit RateDownSampler(int32_t &target_dt_us);
    ~RateDownSampler() = default;

    bool update(const imuSample &imu);

    RateSample getAverageRateAndTriggerReset()
    {
        RateSample rate{};
        rate.time_us = _rate_down_sampled.time_us;
        rate.angular_rate = _rate_down_sampled.angular_rate / float(_sample_count);
        rate.delta_ang_dt = _rate_down_sampled.delta_ang_dt;

        reset();

        return rate;
    }
private:
    void reset();

    RateSample _rate_down_sampled{};
    int32_t &_target_dt_us;
    float _target_dt_s{0.010f};

    hrt_abstime _last_reset{0};
    hrt_abstime _last_timestamp_sample{0};
    hrt_abstime _timestamp_sum{0};
    uint8_t _sample_count{0};

};