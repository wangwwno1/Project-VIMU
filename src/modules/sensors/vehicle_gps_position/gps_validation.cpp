//
// Created by Robert Wang on 2022/8/29.
//

#include "VehicleGPSPosition.hpp"

using matrix::Dcmf;
using matrix::Quatf;

namespace sensors
{
    void VehicleGPSPosition::UpdateReferenceState() {
        if (_ref_gps_buffer == nullptr) {
            const uint8_t buffer_length = roundf(fmaxf(_param_ekf2_gps_delay.get() * 1.5f / (_param_ekf2_predict_us.get() * 1.e-3f), 2.f));
            RingBuffer<RefGpsSample> *inst = new RingBuffer<RefGpsSample> (buffer_length);
            if (inst && inst->valid()) {
                _ref_gps_buffer = inst;
                PX4_INFO("Ref GPS buffer allocated");
            } else {
                PX4_ERR("Ref GPS buffer allocation failed");
                return;
            }
        }

        if (!_reference_states_sub.advertised()) return;

        if (!_reference_states_sub.registered()) {
            _reference_states_sub.registerCallback();
            return;
        }

        // Check global origin Update first
        vehicle_local_position_s local_pos{};
        if (_vehicle_local_position_sub.update(&local_pos)) {
            if (local_pos.ref_timestamp != _global_origin.getProjectionReferenceTimestamp()) {
                // Update reference origin
                _global_origin.initReference(local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp);
                _gps_alt_ref = local_pos.ref_alt;
            }
        }

        // Update reference state, generate reference gps sample
        estimator_states_s ref_states;
        estimator_offset_states_s offset_states;
        if (_reference_states_sub.update(&ref_states) && _vehicle_offset_states_sub.copy(&offset_states)) {
            _last_reference_timestamp = ref_states.timestamp;

            RefGpsSample sample_delayed{};
            sample_delayed.q = Quatf(ref_states.states[0], ref_states.states[1],
                                     ref_states.states[2], ref_states.states[3]);

            // Velocity in NED inertial frame
            sample_delayed.vel(0) = ref_states.states[4];
            sample_delayed.vel(1) = ref_states.states[5];
            sample_delayed.vel(2) = ref_states.states[6];

            // Position in NED inertial frame
            sample_delayed.pos(0) = ref_states.states[7];
            sample_delayed.pos(1) = ref_states.states[8];
            sample_delayed.pos(2) = ref_states.states[9];

            // State Variances
            // Velocity Covariances in NED inertial frame
            sample_delayed.vel_var(0) = ref_states.covariances[4];
            sample_delayed.vel_var(1) = ref_states.covariances[5];
            sample_delayed.vel_var(2) = ref_states.covariances[6];

            // Position Covariances in NED inertial frame
            sample_delayed.pos_var(0) = ref_states.covariances[7];
            sample_delayed.pos_var(1) = ref_states.covariances[8];
            sample_delayed.pos_var(2) = ref_states.covariances[9];

            sample_delayed.ang_rate_delayed_raw = Vector3f(
                    offset_states.ang_rate_delayed_raw[0],
                    offset_states.ang_rate_delayed_raw[1],
                    offset_states.ang_rate_delayed_raw[2]
            );
            sample_delayed.dt_ekf_avg = offset_states.dt_ekf_avg;

            sample_delayed.time_us = ref_states.timestamp_sample;

            _ref_gps_buffer->push(sample_delayed);
        }
    }

    void VehicleGPSPosition::ValidateGpsData(sensor_gps_s &gps_position) {

        if (_reference_states_sub.advertised() && _ref_gps_buffer) {
            // Start stealthy attack & sensor validation
            if (hrt_elapsed_time(&_last_reference_timestamp) > 1_s) {
                // reset validators and buffer
                _ref_gps_buffer->pop_first_older_than(_ref_gps_buffer->get_newest().time_us, &_ref_gps_delayed);
                _ref_gps_delayed.time_us = 0;
                _last_pos_error.zero();
                _last_vel_error.zero();
                _pos_validator.reset();
                _vel_validator.reset();
            }

            const float dt_ekf_avg = (_ref_gps_delayed.time_us == 0) ? _ref_gps_delayed.dt_ekf_avg : 0.f;
            hrt_abstime actual_timestamp = gps_position.timestamp;
            actual_timestamp -= static_cast<hrt_abstime>(_param_ekf2_gps_delay.get() * 1e3f);
            actual_timestamp -= static_cast<hrt_abstime>(dt_ekf_avg * 5e5f);
            const bool ref_gps_ready = _ref_gps_buffer->pop_first_older_than(gps_position.timestamp, &_ref_gps_delayed);

            if (_global_origin.isInitialized() && ref_gps_ready) {

                // State Variances
                // Velocity Covariances in NED inertial frame
                const float vel_obs_var = math::sq(fmaxf(gps_position.s_variance_m_s, _param_ekf2_gps_v_noise.get()));
                _last_vel_vars = _ref_gps_delayed.vel_var;
                _last_vel_vars.xy() += vel_obs_var;
                _last_vel_vars(2) += vel_obs_var * math::sq(1.5f);

                // Position Covariances in NED inertial frame
                const float lower_limit = fmaxf(_param_ekf2_gps_p_noise.get(), 0.01f);
                const float pos_xy_var = math::sq(fmaxf(gps_position.eph, lower_limit));
                const float pos_z_var = math::sq(fmaxf(gps_position.epv, lower_limit) * 1.5f);
                _last_pos_vars = _ref_gps_delayed.pos_var;
                _last_pos_vars.xy() += pos_xy_var;
                _last_pos_vars(2) += pos_z_var;

                // Convert Reference GPS from NED frame to sensor board frame
                // Calculate rotation matrix for position correction
                const Dcmf R_to_earth{_ref_gps_delayed.q};
                const Vector3f vel_offset_body = _ref_gps_delayed.ang_rate_delayed_raw % _gps_pos_body;
                const Vector3f vel_offset_earth = R_to_earth * vel_offset_body;
                const Vector3f ref_vel_board = _ref_gps_delayed.vel + vel_offset_earth;

                // Convert reference GPS to board frame - the reverse of gps correction
                const Vector3f pos_offset_earth = R_to_earth * _gps_pos_body;
                Vector3f ref_pos_board{};
                ref_pos_board(0) = _ref_gps_delayed.pos(0) + pos_offset_earth(0);
                ref_pos_board(0) = _ref_gps_delayed.pos(1) + pos_offset_earth(1);
                ref_pos_board(2) = _ref_gps_delayed.pos(2) - pos_offset_earth(2);  // z-offset is down axis

                // Attempt to apply Stealthy Attack, if failed, fallback to overt attack
                if (!ConductPositionSpoofing(gps_position, ref_pos_board)) {
                    ConductPositionSpoofing(gps_position);
                }

                if (!ConductVelocitySpoofing(gps_position, ref_vel_board)) {
                    ConductVelocitySpoofing(gps_position);
                }

                // Update validator
                // calculate velocity innovations
                const Vector3f gps_vel{gps_position.vel_n_m_s, gps_position.vel_e_m_s, gps_position.vel_d_m_s};
                _last_vel_error = ref_vel_board - gps_vel;

                // calculate position innovations
                double lat = gps_position.lat * 1.e-7;
                double lon = gps_position.lon * 1.e-7;
                // horizontal position innovations
                _last_pos_error.xy() = Vector2f(ref_pos_board.xy()) - _global_origin.project(lat, lon);
                // vertical position innovation - gps measurement has opposite sign to earth z axis
                _last_pos_error(2) = ref_pos_board(2) + (gps_position.alt * 1.e-3f - _gps_alt_ref);

                // test ratio = residual / std_error = residual / sqrt(vars)
                _pos_validator.validate(_last_pos_error.edivide(_last_pos_vars.sqrt()));
                _vel_validator.validate(_last_vel_error.edivide(_last_vel_vars.sqrt()));

                PublishSensorStatus();
                PublishErrorStatus();
                ReplaceGpsPosVelData(gps_position, ref_pos_board, ref_vel_board);
            }

        } else {
            // Fallback to Overt Attack & Skip detection
            ConductVelocitySpoofing(gps_position);
            ConductPositionSpoofing(gps_position);
        }

    }

    void VehicleGPSPosition::PublishSensorStatus() {

        bool healthy = (_pos_validator.test_ratio() < 1.f) && (_vel_validator.test_ratio() < 1.f);
        if ((healthy != _last_healthy) || (hrt_elapsed_time(&_last_health_status_publish) > 1_s)) {
            sensors_status_gps_s status{};
            status.pos_test_ratio = _pos_validator.test_ratio();
            status.vel_test_ratio = _vel_validator.test_ratio();
            status.test_ratio = fmaxf(_pos_validator.test_ratio(), _vel_validator.test_ratio());
            status.healthy = healthy;
            status.ref_gps_enabled = !healthy;
            status.timestamp = hrt_absolute_time();
            _sensors_status_gps_pub.publish(status);

            if (healthy != _last_healthy) {
                if (healthy) {
                    PX4_INFO("GPS fault diminished, switch back to GPS measurement.");
                } else {
                    PX4_WARN("GPS fault detected, switch to reference GPS.");
                }
            }

            _last_healthy = healthy;
            _last_health_status_publish = status.timestamp;
        }

    }

    void VehicleGPSPosition::PublishErrorStatus() {
        // Publish error between sensor and reference
        sensor_gps_error_s gps_error{};
        _last_pos_error.copyTo(gps_error.position_error);
        _last_vel_error.copyTo(gps_error.velocity_error);
        gps_error.timestamp = hrt_absolute_time();
        _sensor_gps_error_pub.publish(gps_error);

        // Publish variances from reference estimator
        sensor_gps_error_s gps_variances{};
        _last_pos_vars.copyTo(gps_variances.position_error);
        _last_vel_vars.copyTo(gps_variances.velocity_error);
        gps_variances.timestamp = hrt_absolute_time();
        _sensor_gps_error_variances_pub.publish(gps_variances);

        // Publish test ratios = error / variances
        sensor_gps_error_s gps_test_ratios{};
        _last_pos_error.edivide(_last_pos_vars).copyTo(gps_test_ratios.position_error);
        _last_vel_error.edivide(_last_vel_vars).copyTo(gps_test_ratios.velocity_error);
        gps_test_ratios.timestamp = hrt_absolute_time();
        _sensor_gps_error_test_ratios_pub.publish(gps_test_ratios);
    }

    void VehicleGPSPosition::ReplaceGpsPosVelData(sensor_gps_s &gps_position,
                                                  const Vector3f &ref_pos_board,
                                                  const Vector3f &ref_vel_board) {
        // Replace corresponding information if gps is unhealthy
        if ((_pos_validator.test_ratio() > 1.f) || (_vel_validator.test_ratio() > 1.f)) {
            // Replace Position Info with reference
            double lat, lon;
            float alt = _gps_alt_ref - ref_pos_board(2);  // local position height is Downward
            _global_origin.reproject(ref_pos_board(0), ref_pos_board(1), lat, lon);
            gps_position.lat = (int) (lat * 1e7);
            gps_position.lon = (int) (lon * 1e7);
            gps_position.alt = (int) (alt * 1e3f);

            // Replace Velocity info with reference
            gps_position.vel_n_m_s = ref_vel_board(0);
            gps_position.vel_e_m_s = ref_vel_board(1);
            gps_position.vel_d_m_s = ref_vel_board(2);
            gps_position.vel_m_s = ref_vel_board.norm();
        }
    }

}  // sensors