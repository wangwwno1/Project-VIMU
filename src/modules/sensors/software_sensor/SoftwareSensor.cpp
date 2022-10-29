//
// Created by Robert Wang on 2022/8/26.
//

#include "SoftwareSensor.h"

using matrix::wrap_pi;

SoftwareSensor::SoftwareSensor():
        ModuleParams(nullptr),
        ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

SoftwareSensor::~SoftwareSensor() {
    Stop();
    // free perf_counters
}

bool SoftwareSensor::multi_init(int instance) {
    // advertise all topics to ensure consistent uORB instance numbering
    _reference_angular_acceleration_pub.advertise();
    _reference_angular_velocity_pub.advertise();
    _reference_accel_pub.advertise();
    _reference_gyro_pub.advertise();
    _reference_imu_pub.advertise();
    _reference_combined_pub.advertise();
    _reference_state_pub.advertise();

    const int status_instance = _reference_imu_pub.get_instance();

    if ((status_instance >= 0)
        && (_reference_angular_velocity_pub.get_instance() == status_instance)
        && (_reference_accel_pub.get_instance() == status_instance)
        && (_reference_gyro_pub.get_instance() == status_instance)) {

        _instance = status_instance;

        ScheduleNow();
        return true;
    }

    PX4_ERR("publication instance problem: %d ref-rate: %d ref-accel: %d ref-gyro: %d", status_instance,
            _reference_angular_velocity_pub.get_instance(), _reference_accel_pub.get_instance(), _reference_gyro_pub.get_instance());

    return false;
}

bool SoftwareSensor::Start() {
    ScheduleNow();
    return true;
}

void SoftwareSensor::Stop() {
    _actuator_outputs_sub.unregisterCallback();

    ScheduleClear();
}

void SoftwareSensor::reset() {
    _pos_model.reset_state();
    _vel_model.reset_state();
    _att_model.reset_state();
    _rate_model.reset_state();
    _angular_accel_filter.reset(Vector3f{0.f, 0.f, 0.f});
}

void SoftwareSensor::Run() {
    // Check if parameters have changed
    ParameterUpdate(!_callback_registered);

    if (!_callback_registered) {
        _callback_registered = _actuator_outputs_sub.registerCallback();
        if (!_callback_registered) {
            PX4_WARN("SoftwareSensor - failed to register callback, retrying");
            ScheduleDelayed(10_ms);
            return;
        }

        reset();
    }

    PublishReferenceIMU();
    UpdateImuStatus();

    // Run() will be invoked when actuator_output receive update
    actuator_outputs_s act{};
    if (_actuator_outputs_sub.copy(&act)) {
        if (!math::isInRange(_last_update_us, act.timestamp - _rate_ctrl_interval_us, act.timestamp + _rate_ctrl_interval_us)) {
            _last_update_us = act.timestamp;
        }

        const hrt_abstime update_start = hrt_absolute_time();
        const hrt_abstime target_time_us = math::max(update_start - _rate_ctrl_interval_us, act.timestamp);
        while (_last_update_us <= target_time_us) {
            // Note: Always update land status before actuator output & angular velocity
            UpdateCopterStatus();

            // From now on we are forward into future
            _last_update_us += _rate_ctrl_interval_us;

            // State Estimation
            UpdatePosVelState();
            UpdateAttitude();
            UpdateAngularVelocityAndAcceleration();

            // Update virtual imu instances, use estimated future state for integration
            _gyro_integrator.put(_rate_model.getOutputState(), _rate_ctrl_interval);

            // Publish Angular Rate & Acceleration
            // The Rate Control frequency are faster than IMU integral, so we publish it every time actuator output generated
            if (_copter_status.publish) {
                PublishAngularVelocityAndAcceleration();
                PublishReferenceGyroAndAccelerometer();
                PublishReferenceState();
            }
        }
    }

}

void SoftwareSensor::UpdateCopterStatus() {
    // TODO How to detect landing and detach from software sensor?
    vehicle_land_detected_s vehicle_land_detected;
    if (_vehicle_land_detected_sub.update(&vehicle_land_detected)) {
        if (!vehicle_land_detected.at_rest && vehicle_land_detected.landed) {
            // Record landed time, defer reset until several EKF interval is passed.
            if (!_copter_status.landed) {
                _last_landed_us = hrt_absolute_time();
            }
        }

        _copter_status.at_rest = vehicle_land_detected.at_rest;
        _copter_status.landed = vehicle_land_detected.landed;
    }

    // Always check land status even vehicle_land_detected is not updated.
    if (_copter_status.landed && _copter_status.in_air) {
        _copter_status.in_air = false;
        _copter_status.publish = false;
        reset();
        PX4_INFO("Vehicle landed, disarm software sensor");

    } else if (!_copter_status.in_air && !(_copter_status.at_rest || _copter_status.landed)){
        _copter_status.in_air = true;
        _last_takeoff_us = hrt_absolute_time();
        PX4_INFO("Takeoff confirmed, start publish reference sensors 30 second later");
    } else if (!_copter_status.publish && _copter_status.in_air && hrt_elapsed_time(&_last_takeoff_us) > 30_s) {
        _copter_status.publish = true;
        PX4_INFO("Start publish software sensor reference!");
    }
}

void SoftwareSensor::UpdateImuStatus() {
    sensors_status_imu_s sensors_status_imu;
    if (_sensors_status_imu_sub.update(&sensors_status_imu)) {
        _all_gyro_compromised = true;
        for (unsigned i = 0; i < MAX_SENSOR_COUNT; i++) {
            // check for gyros with excessive difference to mean using accumulated error
            if (sensors_status_imu.gyro_device_ids[i] != 0 && sensors_status_imu.gyro_healthy[i]) {
                _all_gyro_compromised = false;
                break;
            }
        }
    }
}

void SoftwareSensor::UpdatePosVelState() {
    if (_local_pos_sub.updated()) {
        vehicle_local_position_setpoint_s lpos_sp{};
        if (_local_pos_sp_sub.update(&lpos_sp)) {
            const Vector3f pos_sp{lpos_sp.x, lpos_sp.y, lpos_sp.z};
            const Vector3f vel_sp{lpos_sp.vx, lpos_sp.vy, lpos_sp.vz};

            // Convert setpoints to initial frame before update target state
            _pos_model.setTargetState(pos_sp);
            _vel_model.setTargetState(vel_sp);
        }

        _pos_model.update();
        _state.pos = _pos_model.getOutputState();

        // Use model internal state to eliminate change in offset
        const Vector3f prev_vel = _vel_model.getOutputState();
        _vel_model.update();
        _delta_vel = _vel_model.getOutputState() - prev_vel;
        _state.vel = _vel_model.getOutputState();

        // Add acceleration due to gravity
        const matrix::Dcmf R_earth_to_body{_state.att};
        _avg_acceleration = _delta_vel / _filter_update_period;
        _avg_acceleration -= R_earth_to_body.transpose() * Vector3f(0.f, 0.f, CONSTANTS_ONE_G);
    }
}

void SoftwareSensor::UpdateAttitude() {
    // TODO Also publish reference attitude to controller
    // Check setpoint update first
    vehicle_attitude_setpoint_s att_sp{};
    if (_vehicle_attitude_setpoint_sub.update(&att_sp)) {
        const Eulerf target{att_sp.roll_body, att_sp.pitch_body, att_sp.yaw_body};
        _att_model.setTargetState(target);
    }

    if (_all_gyro_compromised) {
        // use supplementary compensation to correct attitude output
        vehicle_magnetometer_s mag{};
        if (_vehicle_magnetometer_sub.update(&mag)){
            sensor_combined_s combined{};
            _sensor_combined_sub.copy(&combined);
            const Vector3f acceleration{combined.accelerometer_m_s2};
            const Vector3f mag_meas{mag.magnetometer_ga};

            const float roll = atan2(acceleration(1), -acceleration(2));
            const float pitch = atan2(acceleration(0), -acceleration(2));
            const float c_r = cos(roll);
            const float s_r = sin(roll);
            const float c_p = cos(pitch);
            const float s_p = sin(pitch);

            // We take different formula here because PX4 attitude rotation is 3-2-1
            const float yaw = atan2(mag_meas(1) * c_r - mag_meas(2) * s_r,
                                    mag_meas(0) * c_p + mag_meas(1) * s_p * s_r - mag_meas(2) * c_r * s_p);

            // Update attitude model's internal state
            _state.att = Vector3f(roll, pitch, wrap_pi(yaw));
            _att_model.setState(_state.att);
        }
    } else {
        // Note Linear Model cannot handle yaw warping at -pi and +pi
        _att_model.update();
        _state.att = _att_model.getOutputState();
        if (abs(wrap_pi(_state.att(2)) - _state.att(2)) > 0.01f) {
            _state.att(2) = wrap_pi(_state.att(2));
            _att_model.setState(_state.att);
        }
    }

}

void SoftwareSensor::UpdateAngularVelocityAndAcceleration() {
    // Check setpoint update first
    vehicle_rates_setpoint_s rate_sp{};
    if (_vehicle_rates_setpoint_sub.update(&rate_sp)) {
        const Vector3f target{rate_sp.roll, rate_sp.pitch, rate_sp.yaw};
        _rate_model.setTargetState(target);
    }

    // Suppose tick T = now
    // 1. Record prediction of T s(T) from T - 1
    // 2. Forward prediction to T+1
    // 3. Predict and filter angular acceleration between T and T+1
    // 4. Publish Rate and Angular Acceleration at T+1 for next rate control (also at T+1)
    const Vector3f prev_rates = _rate_model.getOutputState();
    _rate_model.update();
    _state.rates = _rate_model.getOutputState();
    _angular_accel_filter.update((_rate_model.getOutputState() - prev_rates) / _rate_ctrl_interval);
}


void SoftwareSensor::ParameterUpdate(bool force) {
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        const auto imu_integ_rate_prev = _param_imu_integ_rate.get();
        const auto rate_control_rate_prev = _param_imu_gyro_ratemax.get();

        updateParams(); // update module parameters (in DEFINE_PARAMETERS)

        _filter_update_period_us = _param_ekf2_predict_us.get();
        _filter_update_period = fmaxf(_filter_update_period_us * 1.e-6f, 0.001f);

        // constrain IMU integration time 1-10 milliseconds (100-1000 Hz)
        int32_t rate_control_rate_hz = (_param_imu_gyro_ratemax.get() == 0) ? 1000 : _param_imu_gyro_ratemax.get();
        int32_t imu_integration_rate_hz = math::constrain(_param_imu_integ_rate.get(),
                                                          (int32_t)50, math::min(rate_control_rate_hz, (int32_t) 1000));

        if (!_interval_configured
            || (_param_imu_gyro_ratemax.get() != rate_control_rate_prev)
            || (_param_imu_integ_rate.get() != imu_integ_rate_prev)) {

            // Configure virtual imu integrators.
            _imu_integration_interval = 1.f / imu_integration_rate_hz;
            _imu_integration_interval_us = 1e6f / imu_integration_rate_hz;
            _rate_ctrl_interval = 1.f / rate_control_rate_hz;
            _rate_ctrl_interval_us = 1e6f / rate_control_rate_hz;

            // This integration_interval is a threshold for IMU delay.
            const uint8_t integral_samples = math::max(1, (int)roundf(_imu_integration_interval / _rate_ctrl_interval));
            _gyro_integrator.set_reset_interval((integral_samples - 0.5f) * _rate_ctrl_interval);
            _gyro_integrator.set_reset_samples(integral_samples);
            _interval_configured = true;
        }
    }
}

void SoftwareSensor::PublishAngularVelocityAndAcceleration() {
    // Publish vehicle_angular_acceleration
    vehicle_angular_acceleration_s v_angular_acceleration;
    v_angular_acceleration.timestamp_sample = _last_update_us;
    _angular_accel_filter.getState().copyTo(v_angular_acceleration.xyz);
    v_angular_acceleration.timestamp = hrt_absolute_time();
    _reference_angular_acceleration_pub.publish(v_angular_acceleration);

    // Publish vehicle_angular_velocity
    vehicle_angular_velocity_s v_angular_velocity;
    v_angular_velocity.timestamp_sample = _last_update_us;
    _state.rates.copyTo(v_angular_velocity.xyz);
    v_angular_velocity.timestamp = hrt_absolute_time();
    _reference_angular_velocity_pub.publish(v_angular_velocity);
}

void SoftwareSensor::PublishReferenceIMU() {
    const hrt_abstime now = hrt_absolute_time();
    if (_interval_configured &&
        _gyro_integrator.integral_ready() &&
        (now > _last_integrator_reset)) {
        // Publish when vehicle in air and is ready, fill entries with fake value
        // Add timestamp check to guard against double publish in SITL
        uint16_t delta_angle_dt;
        Vector3f delta_angle;
        // Even without publish, we still reset integrator
        // because we need to synchronize the reset timestamp with the IMU timestamp
        _gyro_integrator.reset(delta_angle, delta_angle_dt);

        uint16_t delta_velocity_dt = delta_angle_dt;
        Vector3f delta_velocity = _avg_acceleration * (delta_velocity_dt * 1.e-6f);
        _last_integrator_reset = _last_update_us;

        if (_copter_status.publish) {
            // Publish reference_imu topic
            vehicle_imu_s imu{};
            delta_angle.copyTo(imu.delta_angle);
            delta_velocity.copyTo(imu.delta_velocity);
            imu.delta_angle_dt = delta_angle_dt;
            imu.delta_velocity_dt = delta_velocity_dt;
            imu.accel_device_id = 0;
            imu.gyro_device_id = 0;
            imu.delta_velocity_clipping = 0;
            imu.gyro_calibration_count = 0;
            imu.accel_calibration_count = 0;
            imu.timestamp_sample = _last_update_us;
            imu.timestamp = hrt_absolute_time();  // Record the actual publish time
            _reference_imu_pub.publish(imu);

            // also publish reference_combined
            const Vector3f avg_rates = delta_angle / (delta_angle_dt * 1e-3f);
            const Vector3f avg_accel = delta_velocity / (delta_velocity_dt * 1e-3f);
            sensor_combined_s ref_sensors{};
            avg_rates.copyTo(ref_sensors.gyro_rad);
            ref_sensors.gyro_integral_dt = delta_angle_dt;
            ref_sensors.accelerometer_timestamp_relative = 0;
            avg_accel.copyTo(ref_sensors.accelerometer_m_s2);
            ref_sensors.accelerometer_integral_dt = delta_velocity_dt;
            ref_sensors.timestamp = _last_update_us;
            _reference_combined_pub.publish(ref_sensors);
        }
    }
}

void SoftwareSensor::PublishReferenceState() {
    if (_copter_status.publish && _estimator_states_sub.update(&_reference_states)) {
        // Replace internal states
        // Attitude Quaternion
        const Quatf q{_state.att};
        _reference_states.states[0] = q(0);
        _reference_states.states[1] = q(1);
        _reference_states.states[2] = q(2);
        _reference_states.states[3] = q(3);

        // Velocity in NED inertial frame
        _reference_states.states[4] = _state.vel(0);
        _reference_states.states[5] = _state.vel(1);
        _reference_states.states[6] = _state.vel(2);

        // Position in NED inertial frame
        _reference_states.states[7] = _state.pos(0);
        _reference_states.states[8] = _state.pos(1);
        _reference_states.states[9] = _state.pos(2);
        // Keep all other states AS-IS

        _reference_states.timestamp_sample = _last_update_us;
        _reference_states.timestamp = hrt_absolute_time();
        _reference_state_pub.publish(_reference_states);
    }
}

void SoftwareSensor::PublishReferenceGyroAndAccelerometer() {
    // Publish Reference Accelerometer & Gyroscope for IMU Detection
    sensor_accel_s ref_accel{};
    ref_accel.timestamp_sample = _last_update_us;
    ref_accel.x = _avg_acceleration(0);
    ref_accel.y = _avg_acceleration(1);
    ref_accel.z = _avg_acceleration(2);
    ref_accel.samples = 1;
    ref_accel.timestamp = hrt_absolute_time();
    _reference_accel_pub.publish(ref_accel);

    sensor_gyro_s ref_gyro{};
    ref_gyro.timestamp_sample = _last_update_us;
    ref_gyro.x = _state.rates(0);
    ref_gyro.y = _state.rates(1);
    ref_gyro.z = _state.rates(2);
    ref_gyro.samples = 1;
    ref_gyro.timestamp = hrt_absolute_time();
    _reference_gyro_pub.publish(ref_gyro);
}

void SoftwareSensor::PrintStatus() {

}






