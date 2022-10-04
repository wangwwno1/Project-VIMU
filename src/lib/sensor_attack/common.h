//
// Created by Robert Wang on 2022/7/27.
//

#pragma once

#include <iostream>
#include <px4_platform_common/defines.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/sensor_gps.h>

namespace sensor_attack {
    using matrix::Vector3f;

    static constexpr uint8_t ATK_MASK_GYRO  = (1 << 0);
    static constexpr uint8_t ATK_MASK_ACCEL = (1 << 1);
    static constexpr uint8_t ATK_GPS_POS    = (1 << 2);
    static constexpr uint8_t ATK_GPS_VEL    = (1 << 3);
    static constexpr uint8_t BLK_BARO_HGT   = (1 << 4);
    static constexpr uint8_t BLK_MAG_FUSE   = (1 << 5);

    static constexpr uint8_t NO_STEALTHY        = 0;
    static constexpr uint8_t DET_CUSUM          = (1 << 0);
    static constexpr uint8_t DET_EWMA           = (1 << 1);
    static constexpr uint8_t DET_TIME_WINDOW    = (1 << 2);

    enum AttackType {
        Bias = 0,
        Linear = 1,
        Exponential = 2
    };

    struct DeviationParams {
        float initial_value = 1.f;
        float rate_of_rise  = 1.f;
        float max_deviation = 1.f;
        float heading_deg   = 0.f;
        float pitch_deg     = 0.f;
    };

    void gps_position_spoofing(sensor_gps_s &gps_output, const Vector3f &actual_deviation);

    void gps_velocity_spoofing(sensor_gps_s &gps_output, const Vector3f &actual_deviation);
}
