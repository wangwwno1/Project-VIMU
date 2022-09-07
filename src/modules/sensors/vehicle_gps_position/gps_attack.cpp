//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"

namespace sensors
{
    void VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position)
    {
        if (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_VEL) {
            if (!_vel_deviation) {
                PX4_INFO("Initiate GPS Velocity Spoofing Attack");
                _vel_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_v_cls.get(), &_vel_atk_params));

                const float time_to_max_deviation_s = (*_vel_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_vel_deviation) {
                const Vector3f deviation = (*_vel_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_velocity_spoofing(gps_position, deviation);
            }

        } else if (_vel_deviation) {
            _vel_deviation.reset();
            PX4_INFO("GPS Velocity Spoofing Stopped");
        }
    }

    bool VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position, const Vector3f &ref_vel_board) {
        // Check if start condition satisfied
        bool attack_applied = false;
        const uint8_t type_mask = _param_atk_stealth_type.get();
        bool start_condition = (type_mask != sensor_attack::NO_STEALTHY) &&
                               (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_VEL);

        if (start_condition) {
            float max_deviation = NAN;

            if (_param_iv_gps_v_mshift.get() > 0.f && (type_mask & sensor_attack::DET_CUSUM)) {
                max_deviation = _param_iv_gps_v_mshift.get();
            }

            if (_param_iv_gps_v_ema_h.get() > 0.f && (type_mask & sensor_attack::DET_EWMA)) {
                // Consider set the max deviation to EMA if we attempt to circumvent them
                // If not (stealthy_attack_flag & sensor_attack::DET_CUSUM) then we replace cusum limit with ema
                max_deviation = (PX4_ISFINITE(max_deviation)) ?
                                fminf(max_deviation, _param_iv_gps_v_ema_h.get()) : _param_iv_gps_v_ema_h.get();
            }

            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_gps_v_l1tw_h.get() > 0.f) && (_param_iv_gps_v_rst_cnt.get() >= 1)) {
                const float l1tw_deviation = _param_iv_gps_v_l1tw_h.get() / _param_iv_gps_v_rst_cnt.get();
                if (PX4_ISFINITE(max_deviation)) {
                    max_deviation = fminf(max_deviation, l1tw_deviation);
                } else {
                    max_deviation = l1tw_deviation;
                }
            }

            max_deviation *= _param_ekf2_gps_v_noise.get();

            if (PX4_ISFINITE(max_deviation)) {
                // Apply stealthy attack to east axis (positive deviation will make drone fly west)
                gps_position.vel_n_m_s = math::constrain(gps_position.vel_n_m_s,
                                                         ref_vel_board(0) - .99f * max_deviation,
                                                         ref_vel_board(0) + .99f * max_deviation);

                gps_position.vel_e_m_s = ref_vel_board(1) + .99f * max_deviation;
                gps_position.vel_d_m_s = math::constrain(gps_position.vel_d_m_s,
                                                         ref_vel_board(2) - .99f * max_deviation,
                                                         ref_vel_board(2) + .99f * max_deviation);
                gps_position.vel_m_s = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s +
                        gps_position.vel_e_m_s * gps_position.vel_e_m_s +
                        gps_position.vel_d_m_s * gps_position.vel_d_m_s);

                attack_applied = true;
            }
        }

        if (attack_applied && _vel_deviation) {
            _vel_deviation.reset();
            PX4_INFO("GPS Velocity Spoofing - switch to stealthy attack");
        }

        return attack_applied;
    }

    void VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position) {
        if (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_POS) {
            if (!_pos_deviation) {
                PX4_INFO("Initiate GPS Position Spoofing Attack");
                _pos_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_p_cls.get(), &_pos_atk_params));

                const float time_to_max_deviation_s = (*_pos_deviation).time_to_max_deviation();
                if (PX4_ISFINITE(time_to_max_deviation_s)) {
                    PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                } else {
                    PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                }

            }

            if (_pos_deviation) {
                const Vector3f deviation = (*_pos_deviation).calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_position_spoofing(gps_position, deviation);
            }

        } else if (_pos_deviation) {
            _pos_deviation.reset();
            PX4_INFO("GPS Position Spoofing Stopped");
        }

    }

    bool VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position, const Vector3f &ref_pos_board)
    {
        // Check if start condition satisfied
        bool attack_applied = false;
        const uint8_t type_mask = _param_atk_stealth_type.get();
        bool start_condition = (type_mask != sensor_attack::NO_STEALTHY) &&
                               (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_POS);

        if (start_condition) {
            float max_deviation = NAN;

            if (_param_iv_gps_p_mshift.get() > 0.f && (type_mask & sensor_attack::DET_CUSUM)) {
                max_deviation = _param_iv_gps_p_mshift.get();
            }

            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_gps_p_l1tw_h.get() > 0.f) && (_param_iv_gps_p_rst_cnt.get() >= 1)) {
                const float l1tw_deviation = _param_iv_gps_p_l1tw_h.get() / _param_iv_gps_p_rst_cnt.get();
                if (PX4_ISFINITE(max_deviation)) {
                    max_deviation = fminf(max_deviation, l1tw_deviation);
                } else {
                    max_deviation = l1tw_deviation;
                }
            }

            max_deviation *= _param_ekf2_gps_p_noise.get();

            if (PX4_ISFINITE(max_deviation)) {
                Vector2f horz_pos{};
                double lat = gps_position.lat * 1.e-7;
                double lon = gps_position.lon * 1.e-7;
                float relative_alt = gps_position.alt * 1.e-3f - _gps_alt_ref;
                _global_origin.project(lat, lon, horz_pos(0), horz_pos(1));

                // Apply stealthy attack to East axis, positive deviation will cause the vehicle fly west.
                horz_pos(1) = ref_pos_board(1) + .99f * max_deviation;

                // Contain other axis below threshold
                horz_pos(0) = math::constrain(horz_pos(0),
                                              ref_pos_board(0) - .99f * max_deviation,
                                              ref_pos_board(0) + .99f * max_deviation);

                // ref_pos altitude is opposite to the gps altitude, where the latter one is Up direction
                relative_alt = math::constrain(relative_alt,
                                               - ref_pos_board(2) - .99f * max_deviation,
                                               - ref_pos_board(2) + .99f * max_deviation);

                _global_origin.reproject(horz_pos(0), horz_pos(1), lat, lon);
                gps_position.lat = (int) (lat * 1e7);
                gps_position.lon = (int) (lon * 1e7);
                gps_position.alt = (int) ((relative_alt + _gps_alt_ref) * 1e3f);
                attack_applied = true;
            }
        }

        if (attack_applied && _pos_deviation) {
            _pos_deviation.reset();
            PX4_INFO("GPS Position Spoofing - switch to stealthy attack");
        }

        return attack_applied;

    }

}  // sensors