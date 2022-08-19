//
// Created by Robert Wang on 2022/7/27.
//

#pragma once

#include <iostream>
#include <px4_platform_common/defines.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/vehicle_gps_position.h>

namespace sensor_attack {
    using matrix::Vector3f;

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

    void gps_position_spoofing(vehicle_gps_position_s &gps_output, const Vector3f &actual_deviation);

    void gps_velocity_spoofing(vehicle_gps_position_s &gps_output, const Vector3f &actual_deviation);
}
