//
// Created by Robert Wang on 2022/7/13.
//

#include "RateDownSampler.hpp"

RateDownSampler::RateDownSampler(int32_t &target_dt_us) : _target_dt_us(target_dt_us)
{
    reset();
}

bool RateDownSampler::update(const imuSample &imu) {
    _last_timestamp_sample = imu.time_us;
    _rate_down_sampled.angular_rate += imu.delta_ang / math::max(imu.delta_ang_dt, 1e-4f);
    _timestamp_sum += (imu.time_us - hrt_abstime((double) imu.delta_ang_dt * (1e6 * 0.5))) / 1000;
    _sample_count++;

    return (imu.time_us - _last_reset) * 1.e-6f > _target_dt_s;
}

void RateDownSampler::reset()
{
    _target_dt_s = math::constrain(_target_dt_us, (int32_t)1000, (int32_t)100000) * 1.e-6f;
    _last_reset = _last_timestamp_sample;
    _rate_down_sampled = {};
    _timestamp_sum = 0;
    _sample_count = 0;
}