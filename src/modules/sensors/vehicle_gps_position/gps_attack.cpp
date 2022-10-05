//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"

namespace sensors
{
    bool VehicleGPSPosition::attack_enabled(const uint8_t &attack_type) const {
        return _param_atk_apply_type.get() & attack_type &&
                _attack_timestamp != 0 && hrt_absolute_time() >= _attack_timestamp;
    }

    void VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position)
    {
        if (attack_enabled(sensor_attack::ATK_GPS_VEL)) {
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
        if (type_mask != sensor_attack::NO_STEALTHY && attack_enabled(sensor_attack::ATK_GPS_VEL)) {
            float max_deviation = NAN;

            if (_param_iv_gps_v_mshift.get() > 0.f && type_mask & sensor_attack::DET_CUSUM) {
                max_deviation = _param_iv_gps_v_mshift.get();
            }

            if (_param_iv_gps_v_ema_h.get() > 0.f && type_mask & sensor_attack::DET_EWMA) {
                // Consider set the max deviation to EMA if we attempt to circumvent them
                // If not (stealthy_attack_flag & sensor_attack::DET_CUSUM) then we replace cusum limit with ema
                max_deviation = (PX4_ISFINITE(max_deviation)) ?
                                fminf(max_deviation, _param_iv_gps_v_ema_h.get()) : _param_iv_gps_v_ema_h.get();
            }

            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                _param_iv_gps_v_twin_h.get() > 0.f && _param_iv_gps_v_rst_cnt.get() >= 1) {
                const float twin_deviation = _param_iv_gps_v_twin_h.get() / _param_iv_gps_v_rst_cnt.get();
                if (!PX4_ISFINITE(max_deviation) || twin_deviation <= max_deviation) {
                    max_deviation = twin_deviation;
                }
            }

            if (PX4_ISFINITE(max_deviation)) {
                const float vel_gate = fmaxf(_param_ekf2_gps_v_gate.get(), 1.f);
                const float vel_noise = fmaxf(_param_ekf2_gps_v_noise.get(), 0.01f);
                max_deviation *= 0.99f * vel_noise;
                
                // Apply stealthy attack to east axis (positive deviation will make drone fly west)
                gps_position.vel_n_m_s = math::constrain(gps_position.vel_n_m_s,
                                                         ref_vel_board(0) - max_deviation,
                                                         ref_vel_board(0) + max_deviation);

                gps_position.vel_e_m_s = ref_vel_board(1) + max_deviation;
                gps_position.vel_d_m_s = math::constrain(gps_position.vel_d_m_s,
                                                         ref_vel_board(2) - max_deviation,
                                                         ref_vel_board(2) + max_deviation);

                // constrain velocity within EKF vel gate
                const float ekf_vel_noise = fmaxf(vel_noise, gps_position.s_variance_m_s);
                gps_position.vel_n_m_s = math::constrain(gps_position.vel_n_m_s,
                                                         ref_vel_board(0) - vel_gate * ekf_vel_noise,
                                                         ref_vel_board(0) + vel_gate * ekf_vel_noise);
                gps_position.vel_e_m_s = math::constrain(gps_position.vel_e_m_s,
                                                         ref_vel_board(1) - vel_gate * ekf_vel_noise,
                                                         ref_vel_board(1) + vel_gate * ekf_vel_noise);
                gps_position.vel_d_m_s = math::constrain(gps_position.vel_d_m_s,
                                                         ref_vel_board(2) - vel_gate * ekf_vel_noise * 1.5f,
                                                         ref_vel_board(2) + vel_gate * ekf_vel_noise * 1.5f);

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
        if (attack_enabled(sensor_attack::ATK_GPS_POS)) {
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
        if (type_mask != sensor_attack::NO_STEALTHY && attack_enabled(sensor_attack::ATK_GPS_POS)) {
            float max_deviation = NAN;

            if (_param_iv_gps_p_mshift.get() > 0.f && (type_mask & sensor_attack::DET_CUSUM)) {
                max_deviation = _param_iv_gps_p_mshift.get();
            }

            if (_param_iv_gps_p_ema_h.get() > 0.f && type_mask & sensor_attack::DET_EWMA) {
                // Consider set the max deviation to EMA if we attempt to circumvent them
                // If not (stealthy_attack_flag & sensor_attack::DET_CUSUM) then we replace cusum limit with ema
                max_deviation = (PX4_ISFINITE(max_deviation)) ?
                                fminf(max_deviation, _param_iv_gps_p_ema_h.get()) : _param_iv_gps_p_ema_h.get();
            }

            Vector3f extra_offset{0.f, 0.f, 0.f};
            if (type_mask & sensor_attack::DET_TIME_WINDOW &&
                (_param_iv_gps_p_twin_h.get() > 0.f) && (_param_iv_gps_p_rst_cnt.get() >= 1)) {
                const float twin_deviation = _param_iv_gps_p_twin_h.get() / _param_iv_gps_p_rst_cnt.get();
                if (!PX4_ISFINITE(max_deviation) || twin_deviation <= max_deviation) {
                    max_deviation = twin_deviation;
                }
            }

            if (PX4_ISFINITE(max_deviation)) {
                Vector2f horz_pos{};
                const float pos_gate = fmaxf(_param_ekf2_gps_p_gate.get(), 1.f);
                const float pos_noise = fmaxf(_param_ekf2_gps_p_noise.get(), 0.01f);
                double lat = gps_position.lat * 1.e-7;
                double lon = gps_position.lon * 1.e-7;
                float relative_alt = gps_position.alt * 1.e-3f - _gps_alt_ref;
                _global_origin.project(lat, lon, horz_pos(0), horz_pos(1));

                max_deviation *= 0.99f * pos_noise;
                // Apply stealthy attack to East axis, positive deviation will cause the vehicle fly west.
                horz_pos(1) = ref_pos_board(1) + max_deviation;

                // Contain other axis below threshold
                horz_pos(0) = math::constrain(horz_pos(0),
                                              ref_pos_board(0) - max_deviation,
                                              ref_pos_board(0) + max_deviation);

                // ref_pos altitude is opposite to the gps altitude, where the latter one is Up direction
                relative_alt = math::constrain(relative_alt,
                                               - ref_pos_board(2) - max_deviation,
                                               - ref_pos_board(2) + max_deviation);
                
                // constrain the deviation within the EKF POS GATE
                const float hpos_noise = fmaxf(gps_position.eph, pos_noise);
                const float vpos_noise = fmaxf(gps_position.epv, pos_noise);
                horz_pos = matrix::constrain(horz_pos,
                                             Vector2f(ref_pos_board.xy()) - pos_gate * hpos_noise,
                                             Vector2f(ref_pos_board.xy()) + pos_gate * hpos_noise);

                relative_alt = math::constrain(relative_alt,
                                               - ref_pos_board(2) - pos_gate * vpos_noise * 1.5f,
                                               - ref_pos_board(2) + pos_gate * vpos_noise * 1.5f);

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