/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "VirtualIMU.hpp"

VirtualIMU::VirtualIMU():
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _ekf_params(_ekf.getParamHandle()),
    _param_ekf2_gyr_noise(_ekf_params->gyro_noise),
    _param_vm_mass(_phys_model_params.mass),
    _param_vm_thr_factor(_phys_model_params.Ct),
    _param_vm_motor_tau(_phys_model_params.motor_time_constant),
    _param_vm_drag_factor(_phys_model_params.Cd),
    _param_vm_ang_acc_noise(_ekf_params->process_noise),
    _param_iv_imu_delay_us(_ekf_params->imu_fuse_delay_us),
    _param_vimu_predict_us(_ekf_params->filter_update_interval_us)
{
    _imu_integration_interval_us = 1e6f / _param_imu_integ_rate.get();
    uint8_t integral_samples = 1;
    if (_param_imu_gyro_ratemax.get() > 0.f) {
        _actuator_outputs_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
        integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _actuator_outputs_interval_us));
    }

    _gyro_integrator.set_reset_interval(_imu_integration_interval_us);
    _gyro_integrator.set_reset_samples(integral_samples);

    _accel_integrator.set_reset_interval(_imu_integration_interval_us);
    _accel_integrator.set_reset_samples(integral_samples);

}

VirtualIMU::~VirtualIMU()
{
    Stop();
//	perf_free(_loop_perf);
//	perf_free(_loop_interval_perf);
}

bool VirtualIMU::multi_init(int instance) {
    // advertise all topics to ensure consistent uORB instance numbering
    _reference_angular_acceleration_pub.advertise();
    _reference_angular_velocity.advertise();
    _reference_accel_pub.advertise();
    _reference_gyro_pub.advertise();
    _reference_imu_pub.advertise();
    _reference_combined_pub.advertise();

    const int status_instance = _reference_imu_pub.get_instance();
    const bool changed_instance = _estimator_sensor_bias_sub.ChangeInstance(status_instance) && _estimator_aero_wrench_sub.ChangeInstance(status_instance);

    if ((status_instance >= 0) && changed_instance
        && (_reference_angular_velocity.get_instance() == status_instance)
        && (_reference_accel_pub.get_instance() == status_instance)
        && (_reference_gyro_pub.get_instance() == status_instance)) {

        _instance = status_instance;

        ScheduleNow();
        return true;
    }

    PX4_ERR("publication instance problem: %d ref-rate: %d ref-accel: %d ref-gyro: %d", status_instance,
            _reference_angular_velocity.get_instance(), _reference_accel_pub.get_instance(), _reference_gyro_pub.get_instance());

    return false;
}

bool VirtualIMU::Start() {
    ScheduleNow();
    return true;
}

void VirtualIMU::Stop() {
    _actuator_outputs_sub.unregisterCallback();

    ScheduleClear();
}

void VirtualIMU::reset() {
    _current_actuator_setpoint.zero();
    _accel_bias.zero();
    _last_update_us = 0;

    _ekf.reset_state();
}

void VirtualIMU::Run()
{
    // TODO handle perf counter
//	perf_begin(_loop_perf);
//	perf_count(_loop_interval_perf);

    // Check if parameters have changed
    ParameterUpdate(!_callback_registered);

    if (!_callback_registered) {
        _callback_registered = _actuator_outputs_sub.registerCallback();
        if (!_callback_registered) {
            PX4_WARN("VirtualIMU - failed to register callback, retrying");
            ScheduleDelayed(10_ms);
            return;
        }

        // Reset
        for (uint8_t idx = 0; idx < MAX_SENSOR_COUNT; ++idx) {
            _imu_health_status[idx] = false;
        }

        reset();
    }

    // backup schedule
    ScheduleDelayed(_backup_schedule_timeout_us);

    // Note: Always update land status before actuator output & angular velocity
    UpdateCopterStatus();

    // Measurement Update
    UpdateIMUData();

    UpdateSensorBias();
    UpdateAerodynamicWrench();

    // Run() will be invoked when actuator_output receive update.
    // Update Actuator Outputs
    actuator_outputs_s act{};
    if (_actuator_outputs_sub.update(&act)) {
        if (_actuator_outputs_sub.get_last_generation() != _actuator_outputs_last_generation + 1) {
//            _data_gap = true;
//            perf_count(_accel_generation_gap_perf);
            _actuator_outputs_interval_mean.reset();
        } else {
            if ((_current_act_sp_timestamp != 0) && (act.timestamp > _current_act_sp_timestamp)) {
                float interval_us = act.timestamp - _current_act_sp_timestamp;
                _actuator_outputs_interval_mean.update(matrix::Vector<float, 1>{interval_us});
            }

            const int interval_count = _actuator_outputs_interval_mean.count();

            // check measured interval periodically
            if ((_actuator_outputs_interval_mean.valid() && (interval_count % 10 == 0)) && (interval_count > 1000)) {

                const float interval_mean = _actuator_outputs_interval_mean.mean()(0);

                // update sample rate if previously invalid or changed
                const float interval_delta_us = fabsf(interval_mean - _actuator_outputs_interval_us);
                const float percent_changed = interval_delta_us / _actuator_outputs_interval_us;

                if (!PX4_ISFINITE(_actuator_outputs_interval_us) || (percent_changed > 0.001f)) {
                    if (PX4_ISFINITE(interval_mean)) {
                        // update integrator configuration if interval has changed by more than 10%
                        if (interval_delta_us > 0.1f * _accel_interval_us) {
                            _update_integrator_config = true;
                        }

                        _actuator_outputs_interval_us = interval_mean;

                    } else {
                        _actuator_outputs_interval_mean.reset();
                    }
                }
            }

        }

        _actuator_outputs_last_generation = _actuator_outputs_sub.get_last_generation();

        // Forward to actuator timestamp before update the actuator setpoint.
        if (_last_update_us != 0) {
            UpdateVirtualIMU(act.timestamp);
        } else {
            _last_update_us = act.timestamp;
        }

        for (int i = 0; i < noutputs; ++i) {
            // fixme pwm min max may changes by the ESC component, should check and sync
            _current_actuator_setpoint(i) = (act.output[i] - PWM_DEFAULT_MIN) / (PWM_DEFAULT_MAX - PWM_DEFAULT_MIN);
            _current_actuator_setpoint(i) = math::constrain(_current_actuator_setpoint(i), 0.f, 1.f);
        }
        _current_act_sp_timestamp = act.timestamp;
    }

    // reconfigure integrators if calculated control interval have changed
    if (_update_integrator_config || !_interval_configured) {
        UpdateIntegratorConfiguration();
    }

    // Update till the newest timestamp or next actuator timestamp
    const hrt_abstime now = hrt_absolute_time();
    if (now > _last_update_us + _backup_schedule_timeout_us) {
        UpdateVirtualIMU(now);
    }

    PublishReferenceIMU();
    if (_copter_status.publish && (now > _last_publish + _publish_interval_min_us)) {
        // Publish Angular Rate & Acceleration
        // The Rate Control frequency are faster than IMU integral, so we publish it every time actuator output generated
        PublishAngularVelocityAndAcceleration();
        PublishSensorReference();
    }

}

void VirtualIMU::UpdateCopterStatus() {
    // TODO How to detect landing and detach from VIMU?
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
        PX4_INFO("Vehicle landed, disarm virtual imu");

    } else if (!_copter_status.in_air && !(_copter_status.at_rest || _copter_status.landed)){
        _copter_status.in_air = true;
        _last_takeoff_us = hrt_absolute_time();
        PX4_INFO("Takeoff confirmed, start publish reference imu 5 second later");
    } else if (_copter_status.in_air && hrt_elapsed_time(&_last_takeoff_us) > 5_s) {
        _copter_status.publish = true;
    }
}

void VirtualIMU::UpdateIMUData() {
    // Update IMU health status before attempt any fuse
    sensors_status_imu_s imu_status;
    if (_sensors_status_imu_sub.update(&imu_status)) {
        for (uint8_t imu = 0; imu < MAX_SENSOR_COUNT; ++imu) {
            // Although we only take gyro measurement, we should also check accelerometer status
            // Since the attack against accelerometer could be also used against gyroscope.
            _imu_health_status[imu] = imu_status.gyro_healthy[imu] && imu_status.accel_healthy[imu];
        }
    }

    // Check imu status & bias, fuse healthy gyro state.
    for (uint8_t uorb_idx = 0; uorb_idx < MAX_SENSOR_COUNT; ++uorb_idx) {
        vehicle_imu_s imu{};
        if (_vehicle_imu_sub[uorb_idx].update(&imu)) {
            // We need to call the subscription once, so it won't take previously unreceived data as an update.
            // Only fuse sample that has passed validation
            if (_copter_status.at_rest || _copter_status.landed || _imu_health_status[uorb_idx]) {
                // IMU OK, or we are on the ground, continue calculation
                imuSample imu_sample{};
                imu_sample.time_us = imu.timestamp_sample;
                imu_sample.delta_ang_dt = imu.delta_angle_dt * 1.e-6f;
                imu_sample.delta_ang = Vector3f{imu.delta_angle};
                imu_sample.delta_vel_dt = imu.delta_velocity_dt * 1.e-6f;
                imu_sample.delta_vel = Vector3f{imu.delta_velocity};

                _ekf.setGyroData(imu_sample, uorb_idx);
            } else {
                // Discard imu update, also clear up its buffer.
                _ekf.reset_imu_buffer(uorb_idx);
            }
        }
    }
}

void VirtualIMU::UpdateSensorBias() {
    estimator_sensor_bias_s bias{};
    if (_copter_status.in_air && _estimator_sensor_bias_sub.update(&bias)) {
        // Update gyro bias after in air
        if (bias.gyro_device_id == 0) {
            _ekf.setGyroBias(0.8f * _ekf.getGyroBias() + 0.2f * matrix::Vector3f(bias.gyro_bias));
        }

        if (bias.accel_device_id == 0) {
            const Vector3f acc_bias{bias.accel_bias};
            if (acc_bias.abs().max() > 0.01f) {
                _accel_bias = 0.99f * _accel_bias + 0.01f * acc_bias;
            } else {
                _accel_bias = acc_bias;
            }
        }
    }
}

void VirtualIMU::UpdateAerodynamicWrench() {
    estimator_aero_wrench_s aero_wrench;
    if (_estimator_aero_wrench_sub.update(&aero_wrench)) {
        _external_accel = static_cast<Vector3f>(aero_wrench.acceleration);
        _external_angular_accel = static_cast<Vector3f>(aero_wrench.angular_acceleration);
    }
}

void VirtualIMU::UpdateVirtualIMU(const hrt_abstime &now, bool force = false) {
    const float dt = (now - _last_update_us) * 1.e-6f;
    if (dt > 1e-6f) {
        // Greater than 1 microsecond
        // Use AlphaFilter to approximate the motor's spin-up effect
        // fixme replace exponential with LUT for dynamic time & saving computation
        if (_phys_model_params.motor_time_constant > 0.f) {
            _actuator_state_lpf.setAlpha(1.f - expf(- dt / _phys_model_params.motor_time_constant));
        } else {
            _actuator_state_lpf.setAlpha(1.f);
        }

        _actuator_state_lpf.update(_current_actuator_setpoint);
        const VectorThrust act_state = _actuator_state_lpf.getState();
        if (_copter_status.in_air) {
            // todo replace with ControlAllocator Style Cd (CW < 0, CCW > 0)
            const float thrust_fac = _phys_model_params.Ct / _phys_model_params.Mass;
            _control_acceleration = static_cast<Vector3f> (QuadThrustAxis * act_state).emult(thrust_fac);
            _control_torque.zero();
            for (int i = 0; i < noutputs; i++) {
                const Vector3f rel_pos_to_cog = static_cast<Vector3f> (QuadMotorPosition.col(i)).emult(_phys_model_params.length) - _phys_model_params.center_of_gravity;
                const Vector3f axis = static_cast<Vector3f> (QuadThrustAxis.col(i));
                _control_torque += _phys_model_params.Ct * rel_pos_to_cog.cross(axis);
            }
            _control_torque += static_cast<Vector3f>(QuadRotationAxis * act_state) * _phys_model_params.Cd;
        }

    }

    // State Estimation
    _ekf.integrateTorque(_control_torque, _external_angular_accel, dt, now);

    // Update virtual imu instances, use estimated future state for integration
    _accel_integrator.put(getBodyAcceleration(), dt);
    _gyro_integrator.put(_ekf.getAngularRate(), dt);
    _last_update_us = now;
}

void VirtualIMU::ParameterUpdate(bool force) {
    if (_parameter_update_sub.updated() || force) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        const auto imu_integ_rate_prev = _param_imu_integ_rate.get();
        const auto rate_control_rate_prev = _param_imu_gyro_ratemax.get();

        updateParams(); // update module parameters (in DEFINE_PARAMETERS)

        // Configure virtual imu integrators.
        if (!_interval_configured ||
            (_param_imu_integ_rate.get() != imu_integ_rate_prev) ||
            (_param_imu_gyro_ratemax.get() != rate_control_rate_prev)) {
            // constrain IMU integration time 1-10 milliseconds (100-1000 Hz)
            int32_t imu_integration_rate_hz = math::constrain(_param_imu_integ_rate.get(),
                                                              (int32_t)50, math::min(rate_control_rate_hz, (int32_t) 1000));
            _imu_integration_interval_us = 1e6f / imu_integration_rate_hz;

            if (_param_imu_integ_rate.get() != imu_integ_rate_prev) {
                // force update
                _update_integrator_config = true;
            }

            if (_param_imu_gyro_ratemax.get() > 0.f) {
                // determine number of sensor samples that will get closest to the desired rate
                const float configured_interval_us = 1e6f / _param_imu_gyro_ratemax.get();
                // publish interval (constrained 100 Hz - 8 kHz)
                _publish_interval_min_us = math::constrain(
                        (int)roundf(configured_interval_us - (_actuator_outputs_interval_us * 0.5f)),
                        125, 10000
                );

            } else {
                _publish_interval_min_us = 0;
            }

        }

        // inertia matrix
        const float inertia[3][3] = {
                {_param_vm_inertia_xx.get(), _param_vm_inertia_xy.get(), _param_vm_inertia_xz.get()},
                {_param_vm_inertia_xy.get(), _param_vm_inertia_yy.get(), _param_vm_inertia_yz.get()},
                {_param_vm_inertia_xz.get(), _param_vm_inertia_yz.get(), _param_vm_inertia_zz.get()}
        };
        _ekf.setInertiaMatrix(SquareMatrix3f(inertia));

        // Update length scale & center of gravity offset
        _phys_model_params.length(0) = _param_vm_len_scale_x.get();
        _phys_model_params.length(1) = _param_vm_len_scale_y.get();
        _phys_model_params.length(2) = _param_vm_len_scale_z.get();
        _phys_model_params.center_of_gravity(0) = _param_vm_cog_off_x.get();
        _phys_model_params.center_of_gravity(1) = _param_vm_cog_off_y.get();
        _phys_model_params.center_of_gravity(2) = _param_vm_cog_off_z.get();
    }
}

void VirtualIMU::UpdateIntegratorConfiguration() {
    if (PX4_ISFINITE(_actuator_outputs_interval_us)) {
        // This integration_interval is a threshold for IMU delay.
        const uint8_t integral_samples = math::max(1, (int)roundf(_imu_integration_interval_us / _actuator_outputs_interval_us));
        const float relaxed_interval = roundf((integral_samples - 0.5f) * _actuator_outputs_interval_us);
        _gyro_integrator.set_reset_interval(relaxed_interval);
        _gyro_integrator.set_reset_samples(integral_samples);

        _accel_integrator.set_reset_interval(relaxed_interval);
        _accel_integrator.set_reset_samples(integral_samples);

        // Control Interval is constrained between 100 Hz and 8 kHz (max gyro rate)
        // fixme what if there is no minimum control interval (i.e., IMU_GYRO_RATEMAX = 0) ?
        _backup_schedule_timeout_us = math::constrain((int) _actuator_outputs_interval_us, 125, 10000);

        _interval_configured = true;
        _update_integrator_config = false;
    }
}

void VirtualIMU::PublishAngularVelocityAndAcceleration() {
    // Publish vehicle_angular_acceleration
    vehicle_angular_acceleration_s v_angular_acceleration;
    v_angular_acceleration.timestamp_sample = _last_update_us;
    _ekf.getAngularAcceleration().copyTo(v_angular_acceleration.xyz);
    v_angular_acceleration.timestamp = hrt_absolute_time();
    _reference_angular_acceleration_pub.publish(v_angular_acceleration);

    // Publish vehicle_angular_velocity
    vehicle_angular_velocity_s v_angular_velocity;
    v_angular_velocity.timestamp_sample = _last_update_us;
    _ekf.getAngularRate().copyTo(v_angular_velocity.xyz);
    v_angular_velocity.timestamp = hrt_absolute_time();
    _reference_angular_velocity.publish(v_angular_velocity);
}

void VirtualIMU::PublishSensorReference() {
    // Publish Reference Accelerometer & Gyroscope for IMU Detection
    const Vector3f accels = getBodyAcceleration();
    sensor_accel_s ref_accel{};
    ref_accel.timestamp_sample = _last_update_us;
    ref_accel.x = accels(0);
    ref_accel.y = accels(1);
    ref_accel.z = accels(2);
    ref_accel.samples = 1;
    ref_accel.timestamp = hrt_absolute_time();
    _reference_accel_pub.publish(ref_accel);

    const Vector3f rates = _ekf.getAngularRate();
    sensor_gyro_s ref_gyro{};
    ref_gyro.timestamp_sample = _last_update_us;
    ref_gyro.x = rates(0);
    ref_gyro.y = rates(1);
    ref_gyro.z = rates(2);
    ref_gyro.samples = 1;
    ref_gyro.timestamp = hrt_absolute_time();
    _reference_gyro_pub.publish(ref_gyro);
}

void VirtualIMU::PublishReferenceIMU() {
    const hrt_abstime now = hrt_absolute_time();
    if (_interval_configured &&
        _accel_integrator.integral_ready() &&
        _gyro_integrator.integral_ready() &&
        (now > _last_integrator_reset)) {
        // Publish when vehicle in air and is ready, fill entries with fake value
        // Add timestamp check to guard against double publish in SITL
        Vector3f delta_velocity{};
        Vector3f delta_angle{};
        uint16_t delta_angle_dt{};
        uint16_t delta_velocity_dt{};

        // Even without publish, we still reset integrator
        // because we need to synchronize the reset timestamp with the actuator timestamp
        _gyro_integrator.reset(delta_angle, delta_angle_dt);
        _accel_integrator.reset(delta_velocity, delta_velocity_dt);
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

            _last_publish = imu.timestamp;
        }
    }
}

void VirtualIMU::PrintStatus() {
    const Vector3f gyro_bias = _ekf.getGyroBias();
    const Vector3f accel_bias = _accel_bias;
    PX4_INFO("ang_vel_bias: [%.4f, %.4f, %.4f]",
             (double)gyro_bias(0), (double)gyro_bias(1), (double)gyro_bias(2));
    PX4_INFO("body_accel_bias: [%.4f, %.4f, %.4f]",
             (double)accel_bias(0), (double)accel_bias(1), (double)accel_bias(2));

    // TODO Provide IMU receive timestamps
    // TODO Add more information e.g. Delay Last read imu
}

//int VirtualIMU::task_spawn(int argc, char *argv[])
//{
//	VirtualIMU *instance = new VirtualIMU();
//
//	if (instance) {
//		_object.store(instance);
//		_task_id = task_id_is_work_queue;
//
//		if (instance->init()) {
//			return PX4_OK;
//		}
//
//	} else {
//		PX4_ERR("alloc failed");
//	}
//
//	delete instance;
//	_object.store(nullptr);
//	_task_id = -1;
//
//	return PX4_ERROR;
//}

//int VirtualIMU::print_status()
//{
//	perf_print_counter(_loop_perf);
//	perf_print_counter(_loop_interval_perf);
//	return 0;
//}

//int VirtualIMU::custom_command(int argc, char *argv[])
//{
//	return print_usage("unknown command");
//}

//int VirtualIMU::print_usage(const char *reason)
//{
//	if (reason) {
//		PX4_WARN("%s\n", reason);
//	}
//
//	PRINT_MODULE_DESCRIPTION(
//		R"DESCR_STR(
//### Description
//Example of a simple module running out of a work queue.
//
//)DESCR_STR");
//
//	PRINT_MODULE_USAGE_NAME("work_item_example", "template");
//	PRINT_MODULE_USAGE_COMMAND("start");
//	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
//
//	return 0;
//}

