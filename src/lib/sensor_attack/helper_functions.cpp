//
// Created by Robert Wang on 2022/7/27.
//

#include "common.h"
#include "Deviation.h"

namespace sensor_attack {
    using matrix::Vector3f;

    void gps_position_spoofing(sensor_gps_s &gps_output, const Vector3f &actual_deviation) {
        // Actual Deviation is opposite to signal deviation.
        float north_m = -actual_deviation(0), east_m = -actual_deviation(1), down_m = -actual_deviation(2);

        if (fabs(north_m) > 0.f || fabs(east_m) > 0.f) {
            double gps_lat = gps_output.lat / 1.0e7;
            double gps_lon = gps_output.lon / 1.0e7;
            const MapProjection fake_ref{gps_lat, gps_lon};
            fake_ref.reproject(north_m, east_m, gps_lat, gps_lon);

            gps_output.lat = (int32_t) (gps_lat * 1.0e7);
            gps_output.lon = (int32_t) (gps_lon * 1.0e7);
        }

        if (fabs(down_m) > 0.f) {
            gps_output.alt = gps_output.alt + (int32_t) (double(down_m) * 1.0e3);
        }
    }

    void gps_velocity_spoofing(sensor_gps_s &gps_output, const Vector3f &actual_deviation) {
        // Actual Deviation is opposite to signal deviation.
        if (actual_deviation.norm_squared() > 0.f) {
            float north_m_s = -actual_deviation(0), east_m_s = -actual_deviation(1), down_m_s = -actual_deviation(2);
            gps_output.vel_n_m_s = gps_output.vel_n_m_s + north_m_s;
            gps_output.vel_e_m_s = gps_output.vel_e_m_s + east_m_s;
            gps_output.vel_d_m_s = gps_output.vel_d_m_s + down_m_s;
            gps_output.vel_m_s = sqrtf(gps_output.vel_n_m_s * gps_output.vel_n_m_s +
                                       gps_output.vel_e_m_s * gps_output.vel_e_m_s +
                                       gps_output.vel_d_m_s * gps_output.vel_d_m_s);
            // TODO also modify cog_rad, the moving direction on the ground
        }
    }

    Deviation *CreateAttackInstance(const int32_t attack_class, DeviationParams *params) {
        switch (attack_class) {
            case AttackType::Bias:
                return new BiasDeviation(params);
            case AttackType::Linear:
                return new LinearDeviation(params);
            case AttackType::Exponential:
                return new ExponentialDeviation(params);
            default:
                return nullptr;
        }
    }
}