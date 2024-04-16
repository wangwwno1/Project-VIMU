//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"
#include <lib/geo/geo.h>

namespace sensors
{
    bool VehicleGPSPosition::attack_enabled(const uint8_t &attack_type) const {
        return _param_atk_apply_type.get() & attack_type &&
                _attack_timestamp != 0 && hrt_absolute_time() >= _attack_timestamp;
    }

    void VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position) {
        if (attack_enabled(sensor_attack::ATK_GPS_VEL)) {
            if (_vel_deviation.load() == nullptr) {
                PX4_INFO("Initiate GPS Velocity Spoofing Attack");
                sensor_attack::Deviation *inst = sensor_attack::CreateAttackInstance(_param_atk_gps_v_cls.get(), &_vel_atk_params);
                if (inst) {
                    const float time_to_max_deviation_s = inst->time_to_max_deviation();
                    if (PX4_ISFINITE(time_to_max_deviation_s)) {
                        PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                    } else {
                        PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                    }

                    _vel_deviation.store(inst);
                } else {
                    PX4_WARN("Failed to initiate GPS Position Spoofing Attack!");
                }
            }

            if (_vel_deviation.load() != nullptr) {
                const Vector3f deviation = _vel_deviation.load()->calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_velocity_spoofing(gps_position, deviation);
            }

        } else if (_vel_deviation.load()) {
            delete _vel_deviation.load();
            _vel_deviation.store(nullptr);
            PX4_INFO("GPS Velocity Spoofing Stopped");
        }
    }

    bool VehicleGPSPosition::ConductVelocitySpoofing(sensor_gps_s &gps_position, const Vector3f &ref_vel_board) {
        // Check if start condition satisfied
        bool attack_applied = false;
        const uint8_t type_mask = _param_atk_stealth_type.get();
        if (type_mask != sensor_attack::NO_STEALTHY && attack_enabled(sensor_attack::ATK_GPS_VEL)) {
            const float max_deviation = _param_atk_gps_v_iv.get();
            if (PX4_ISFINITE(max_deviation)) {
                const float vel_gate = fmaxf(_param_ekf2_gps_v_gate.get(), 1.f);
                const float vel_noise = fmaxf(_param_ekf2_gps_v_noise.get(), 0.01f);
                
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

        if (attack_applied && _vel_deviation.load() != nullptr && _vel_deviation.load()->get_start_time() > 0) {
            _vel_deviation.load()->reset();
            PX4_INFO("GPS Velocity Spoofing - switch to stealthy attack");
        }

        return attack_applied;
    }

    void VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position) {
        if (attack_enabled(sensor_attack::ATK_GPS_POS)) {
            if (_pos_deviation.load() == nullptr) {
                PX4_INFO("Initiate GPS Position Spoofing Attack");
                sensor_attack::Deviation *inst = sensor_attack::CreateAttackInstance(_param_atk_gps_p_cls.get(), &_pos_atk_params);
                if (inst) {
                    const float time_to_max_deviation_s = inst->time_to_max_deviation();
                    if (PX4_ISFINITE(time_to_max_deviation_s)) {
                        PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
                    } else {
                        PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
                    }
                    _pos_deviation.store(inst);
                } else {
                    PX4_WARN("Failed to initiate GPS Position Spoofing Attack!");
                }
            }

            if (_pos_deviation.load() != nullptr) {
                const Vector3f deviation = _pos_deviation.load()->calculate_deviation(gps_position.timestamp);
                sensor_attack::gps_position_spoofing(gps_position, deviation);
            }

        } else if (_pos_deviation.load()) {
            delete _pos_deviation.load();
            _pos_deviation.store(nullptr);
            PX4_INFO("GPS Position Spoofing Stopped");
        }
    }

    bool VehicleGPSPosition::ConductPositionSpoofing(sensor_gps_s &gps_position, const Vector3f &ref_pos_board)
    {
        // Check if start condition satisfied
        bool attack_applied = false;
        const uint8_t type_mask = _param_atk_stealth_type.get();
        if (type_mask != sensor_attack::NO_STEALTHY && attack_enabled(sensor_attack::ATK_GPS_POS)) {
            const float max_deviation = _param_atk_gps_p_iv.get();

            Vector3f extra_offset{0.f, 0.f, 0.f};
            if (PX4_ISFINITE(max_deviation)) {
                Vector2f horz_pos{};
                const float pos_gate = fmaxf(_param_ekf2_gps_p_gate.get(), 1.f);
                const float pos_noise = fmaxf(_param_ekf2_gps_p_noise.get(), 0.01f);
                double lat = gps_position.lat * 1.e-7;
                double lon = gps_position.lon * 1.e-7;
                float relative_alt = gps_position.alt * 1.e-3f - _gps_alt_ref;
                _global_origin.project(lat, lon, horz_pos(0), horz_pos(1));

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

        if (attack_applied && _pos_deviation.load() != nullptr && _pos_deviation.load()->get_start_time() > 0) {
            _pos_deviation.load()->reset();
            PX4_INFO("GPS Position Spoofing - switch to stealthy attack");
        }

        return attack_applied;
    }

    bool VehicleGPSPosition::ConductVPSpoofing(sensor_gps_s &gps_position) {
        bool attack_applied = false;
        if (attack_enabled(sensor_attack::ATK_GPS_V_P)) {
            attack_applied = true;
            if (_pos_record == 1) {
                // fixme this spoofing can only start once, change the pipeline
                _pos_lat_ori = gps_position.lat;
                _pos_lon_ori = gps_position.lon;
                _pos_record = 0;
                PX4_INFO("Initiate Joint Position & Velocity GPS Spoofing.");
            }

            double false_velocity = 0.0;
            double abs_atk_time = static_cast<double>(gps_position.timestamp - _attack_timestamp) / 1000000.0;
            if (abs_atk_time < 60) {
                if (abs_atk_time < 3) {
                    false_velocity = 0.05 * abs_atk_time - 0.15;
                } else if (abs_atk_time < 3.65) {
                    false_velocity = 0.153846 * abs_atk_time - 0.46154;
                } else if (abs_atk_time < 3.85) {
                    false_velocity = 0.5 * abs_atk_time - 1.725;
                } else if (abs_atk_time < 11.5) {
                    false_velocity = -0.01307 * abs_atk_time + 0.250327;
                } else if (abs_atk_time < 13.5) {
                    false_velocity = -0.05 * abs_atk_time + 0.675;
                } else if (abs_atk_time < 15.05) {
                    false_velocity = -0.06452 * abs_atk_time + 0.870968;
                } else if (abs_atk_time < 21) {
                    false_velocity = 0.016807 * abs_atk_time - 0.35294;
                } else if (abs_atk_time < 22) {
                    false_velocity = 0.1 * abs_atk_time - 2.1;
                } else {
                    false_velocity = 0.0;
                }
                double delta_x = (false_velocity + _false_velocity_prev) / 2 * static_cast<double>(gps_position.timestamp - _atk_timestamp_prev) / 1000000.0;

                double gps_lat = _pos_lat_prev / 1.0e7;
                double gps_lon = _pos_lon_prev / 1.0e7;
                const MapProjection fake_ref{gps_lat, gps_lon};
                fake_ref.reproject(0.0, delta_x, gps_lat, gps_lon);

                gps_position.lat = (int32_t) (gps_lat * 1.0e7);
                gps_position.lon = (int32_t) (gps_lon * 1.0e7);

                gps_position.vel_e_m_s = false_velocity;
                gps_position.vel_m_s = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s +
                                            gps_position.vel_e_m_s * gps_position.vel_e_m_s +
                                            gps_position.vel_d_m_s * gps_position.vel_d_m_s);

            } else {
                gps_position.lat = _pos_lat_ori;
                gps_position.lon = _pos_lon_ori;
                gps_position.vel_e_m_s = 0.0;
                gps_position.vel_m_s = sqrtf(gps_position.vel_n_m_s * gps_position.vel_n_m_s +
                                            gps_position.vel_e_m_s * gps_position.vel_e_m_s +
                                            gps_position.vel_d_m_s * gps_position.vel_d_m_s);
            }

        }

        _false_velocity_prev = gps_position.vel_m_s;
        _atk_timestamp_prev = gps_position.timestamp;
        _pos_lat_prev = gps_position.lat;
        _pos_lon_prev = gps_position.lon;

        return attack_applied;

    }

}  // sensors
