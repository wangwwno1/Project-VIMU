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

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/mathlib/math/WelfordMean.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/estimator_aero_wrench.h>
#include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensors_status_imu.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_imu.h>

#include "../vehicle_imu/Integrator.hpp"
#include "AuxEKF.hpp"

using namespace time_literals;

using matrix::AxisAnglef;
using matrix::Quatf;
using matrix::Vector3f;
using Vector16f = matrix::Vector<float, 16>;

using namespace estimator;

static const uint8_t noutputs = 4;
using VectorThrust = matrix::Vector<float, noutputs>;

// todo replace with ControlAllocator Parameter (CA_ROTOR*)
static const matrix::Matrix<float, 3, noutputs> QuadMotorPosition((float[16]) { +0.5, -0.5, +0.5, -0.5,
                                                                                +0.5, -0.5, -0.5, +0.5,
                                                                                 0.0,  0.0,  0.0,  0.0});
static const matrix::Matrix<float, 3, noutputs> QuadRotationAxis((float[16]) { -0, +0, +0, -0,
                                                                               +0, -0, +0, -0,
                                                                               +1, +1, -1, -1});;
static const matrix::Matrix<float, 3, noutputs> QuadThrustAxis((float[16]) { 0,  0,  0,  0,
                                                                             0,  0,  0,  0,
                                                                            -1, -1, -1, -1});

class VirtualIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
    VirtualIMU();
	~VirtualIMU() override;

    bool Start();
    void Stop();

    void PrintStatus();

    bool multi_init(int instance);

    const Vector3f getBodyAcceleration() const { return _control_acceleration + _external_accel - _accel_bias;}

    static constexpr uint8_t MAX_SENSOR_COUNT = 4;

    // This device id is corresponding to
    // 11111111       00000001  00001  000
    // which decodes to
    // DEVTYPE_UNUSED dev 1     bus 1  DeviceBusType_UNKNOWN
    static const constexpr unsigned VIMU_DEVICE_ID = 16711944;
    static const constexpr unsigned VIMU_ACCEL_DEVICE_ID = 0;   // fixme replace with VIMU_DEVICE_ID - require modification in VehicleAcceleration module
    static const constexpr unsigned VIMU_GYRO_DEVICE_ID = VIMU_DEVICE_ID;

private:

	void Run() override;

    void reset();

    void ParameterUpdate(bool force = false);
    void UpdateControlInterval(const hrt_abstime &actuator_timestamp);
    void UpdateIntegratorConfiguration();

    void UpdateCopterStatus();
    void UpdateIMUData();
    void UpdateBiasAndAerodynamicWrench();
    void UpdateVirtualIMU(const hrt_abstime &now);

    void PublishAngularVelocityAndAcceleration();
    void PublishReferenceIMU();
    void ForecastAndPublishDetectionReference();

    void CalculateThrustAndTorque(VectorThrust &actuator_state, const VectorThrust &setpoint, const float dt,
                                  Vector3f &thrust, Vector3f &torque, bool update_state = true) {
        float update_weight = 1.f;
        if (_phys_model_params.motor_time_constant > 1e-5f) {
            if (dt > 1.e-6f) {
                update_weight = 1 - expf(- dt / _phys_model_params.motor_time_constant);
            } else {
                update_weight = 0.f;
            }
        }

        Vector3f thrust_tmp{0.f, 0.f, 0.f};
        Vector3f torque_tmp{0.f, 0.f, 0.f};
        const float thr_mdl_fac = _param_vm_motor_mdl_fac.get();
        const bool using_induced_vel_xy = _phys_model_params.Ct_vxy > 0.f;
        const bool using_induced_vel_z = _phys_model_params.Ct_vz > 0.f;
        const bool using_induced_thrust = using_induced_vel_xy || using_induced_vel_z;
        for (int i = 0; i < noutputs; i++) {
            // Get the ith motor position relative to center of gravity
            // for calculating the torque generated by the motor thrust
            const Vector3f rel_pos_to_cog = static_cast<Vector3f> (QuadMotorPosition.col(i)).emult(_phys_model_params.length) - (_phys_model_params.center_of_gravity - _phys_model_params.center_of_thrust);
            const Vector3f thr_axis = static_cast<Vector3f> (QuadThrustAxis.col(i));

            const float actuatorLastState = actuator_state(i);
            const float actuatorCommand = setpoint(i);
            const float delta_state = update_weight * (actuatorCommand - actuatorLastState);
            const float actuatorState = actuatorLastState + delta_state;
            if (update_state) {
                actuator_state(i) = actuatorState;
            }

            // Calculate the thrust vector of ith motor
            const float thrust_coef = _phys_model_params.Ct * _rotor_param[i].thrust_scale;
            const float rel_signal = (1.f - thr_mdl_fac) * actuatorState + thr_mdl_fac * actuatorState * actuatorState;
            float ith_thrust = thrust_coef * rel_signal;
            if (using_induced_thrust) {
                const Vector3f ith_induced_vel = _rel_wind_body + _ekf.getAngularRate().cross(rel_pos_to_cog);
                if (using_induced_vel_xy) {
                    ith_thrust += _phys_model_params.Ct_vxy * ith_induced_vel.xy().norm_squared();
                }

                if (using_induced_vel_z) {
                    ith_thrust += _phys_model_params.Ct_vz * ith_induced_vel(2) * actuatorState;
                }
            }
            const Vector3f ith_thrust_vec = ith_thrust * thr_axis;
            thrust_tmp += ith_thrust_vec;

            // Calculate the torque generated by the thrust of ith motor.
            const Vector3f rot_axis = static_cast<Vector3f> (QuadRotationAxis.col(i));
            Vector3f thrust_torque = rel_pos_to_cog.cross(ith_thrust_vec);  // todo apply yaw scale to thrust torque
            thrust_torque(0) = thrust_torque(0) * _rotor_param[i].roll_scale;
            thrust_torque(1) = thrust_torque(1) * _rotor_param[i].pitch_scale;
            torque_tmp += thrust_torque;
            torque_tmp += _phys_model_params.Cd * _rotor_param[i].yaw_scale * rel_signal * rot_axis;
            if (fabsf(_rotor_param[i].yaw_delta_state_scale) > 1.e-6f) {
                torque_tmp += _rotor_param[i].yaw_delta_state_scale * delta_state * rot_axis;
            }
        }

        // Return the result
        torque = torque_tmp;
        thrust = thrust_tmp;
    }

    static constexpr float sq(float x) { return x * x; };

    bool _callback_registered{false};

    // Copter flight status
    struct CopterStatus {
        bool at_rest{true};
        bool landed{true};
        bool in_air{false};
        bool publish{false};
    };

    CopterStatus _copter_status{};
    hrt_abstime _last_landed_us{0};
    hrt_abstime _last_takeoff_us{0};

    // Imu Integrators
    bool                _interval_configured{false};
    bool                _update_integrator_config{false};
    uint32_t            _imu_integration_interval_us;
    hrt_abstime         _last_integrator_reset{0};
    Integrator          _accel_integrator{};
    IntegratorConing    _gyro_integrator{};

    // Publish timestamps & Schedule Intervals
    hrt_abstime         _last_imu_reference_publish{0};
    hrt_abstime         _last_detection_reference_publish{0};
    hrt_abstime         _last_rate_ctrl_reference_publish{0};
    uint32_t            _publish_interval_min_us{0};
    uint32_t            _backup_schedule_timeout_us{10000};

    // Actuator timestamps and state
    hrt_abstime     _last_state_update_us{0};
    hrt_abstime     _current_act_sp_timestamp{0};
    VectorThrust    _current_actuator_state{};
    VectorThrust    _current_actuator_setpoint{};

    unsigned _actuator_outputs_last_generation{0};
    float _actuator_outputs_interval_us{NAN};
    math::WelfordMean<float, 1> _actuator_outputs_interval_mean{};

    // External Wrench
    Vector3f _rel_wind_body{0.f, 0.f, 0.f};
    Vector3f _external_accel{0.f, 0.f, 0.f};
    Vector3f _external_angular_accel{0.f, 0.f, 0.f};

    // Sensor Bias
    Vector3f _control_acceleration{0.f, 0.f, 0.f};
    Vector3f _control_torque{0.f, 0.f, 0.f};
    Vector3f _accel_bias{};

    // (Copter) Physical Model Parameters
    struct CopterPhysModelParams {
        float mass{0.8f};
        float Ct{4.00f};
        float Cd{0.05f};
        float Ct_vxy{0.0f};
        float Ct_vz{0.0f};
        float motor_time_constant{0.005f};  // 5ms for jMAVSim default
        Vector3f length{1.f, 1.f, 1.f};  // 0.233345 for jMAVSim
        Vector3f center_of_gravity{0.f, 0.f, 0.f};
        Vector3f center_of_thrust{0.f, 0.f, 0.f};
    };

    // Battery Scale
    AlphaFilter<float>  _voltage_scaler{0.03f};  // CUTOFF_FREQ = 0.5 Hz @ 100Hz Sampling Rate  // todo acquire sampling rate

    CopterPhysModelParams _phys_model_params{};

    struct RotorParamHandle {
        float	roll_scale{1.0f};		            /**< scales roll for this rotor */
        float	pitch_scale{1.0f};	                /**< scales pitch for this rotor */
        float	yaw_scale{1.0f};		            /**< scales yaw for this rotor */
        float   yaw_delta_state_scale{-1.0f};       /**< scales yaw by delta state for this rotor */
        float	thrust_scale{-1.0f};                /**< scales thrust for this rotor */
    };

    RotorParamHandle _rotor_param[noutputs];

    // IMU health status that control AuxEKF angular velocity fusion
    bool _imu_health_status[MAX_SENSOR_COUNT];
    bool _all_imu_compromised{false};

    // Auxiliary EKF for angular velocity estimate
    AuxEKF _ekf{};
    AuxEKFParam *_ekf_params;
    bool _ekf_inertia_initialized{false};

    int _instance{0};

    // Publications
    uORB::PublicationMulti<vehicle_angular_acceleration_s>   _reference_angular_acceleration_pub{ORB_ID(reference_angular_acceleration)};
    uORB::PublicationMulti<vehicle_angular_velocity_s>       _reference_angular_velocity_pub{ORB_ID(reference_angular_velocity)};
    uORB::PublicationMulti<sensor_accel_s>                   _reference_accel_pub{ORB_ID(reference_accel)};
    uORB::PublicationMulti<sensor_gyro_s>                    _reference_gyro_pub{ORB_ID(reference_gyro)};
    uORB::PublicationMulti<sensor_gyro_s>                    _recovery_gyro_pub{ORB_ID(recovery_gyro)};
    uORB::PublicationMulti<sensor_combined_s>                _reference_combined_pub{ORB_ID(reference_combined)};
    uORB::PublicationMulti<vehicle_imu_s>                    _reference_imu_pub{ORB_ID(reference_imu)};

	// Subscriptions
    uORB::SubscriptionCallbackWorkItem                  _actuator_outputs_sub{this, ORB_ID(actuator_outputs)};      // subscription that schedules VirtualIMU when updated
    uORB::SubscriptionMultiArray<vehicle_imu_s>         _vehicle_imu_sub{ORB_ID::vehicle_imu};
	uORB::SubscriptionInterval                          _parameter_update_sub{ORB_ID(parameter_update), 1_s};       // subscription limited to 1 Hz updates
    uORB::Subscription                                  _battery_status_sub{ORB_ID(battery_status), 0};             // todo which battery?
    uORB::Subscription                                  _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription                                  _estimator_aero_wrench_sub{ORB_ID(estimator_aero_wrench)};
    uORB::Subscription                                  _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
    uORB::Subscription                                  _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription                                  _sensors_status_imu_sub{ORB_ID(sensors_status_imu)};

    // Performance (perf) counters  // TODO Perf Counters
//	perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
//	perf_counter_t	_loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Parameters
    DEFINE_PARAMETERS(
            (ParamInt<px4::params::IMU_INTEG_RATE>)         _param_imu_integ_rate,
            (ParamInt<px4::params::IMU_GYRO_RATEMAX>)       _param_imu_gyro_ratemax,
            (ParamExtFloat<px4::params::EKF2_GYR_NOISE>)    _param_ekf2_gyr_noise,
            (ParamExtFloat<px4::params::VM_MASS>)           _param_vm_mass,
            (ParamExtFloat<px4::params::VM_THR_FACTOR>)     _param_vm_thr_factor,
            (ParamExtFloat<px4::params::VM_MOTOR_TAU>)      _param_vm_motor_tau,
            (ParamInt<px4::params::VM_MOTOR_MIN_PWM>)       _param_vm_motor_min_pwm,
            (ParamFloat<px4::params::VM_MOTOR_MDL_FAC>)     _param_vm_motor_mdl_fac,
            (ParamExtFloat<px4::params::VM_TCOEF_VI_XY>)    _param_vm_tcoef_vi_xy,
            (ParamExtFloat<px4::params::VM_TCOEF_VI_Z>)     _param_vm_tcoef_vi_z,
            (ParamExtFloat<px4::params::VM_DRAG_FACTOR>)    _param_vm_drag_factor,
            (ParamExtFloat<px4::params::VM_ANG_ACC_NOISE>)  _param_vm_ang_acc_noise,
            (ParamFloat<px4::params::VM_INERTIA_XX>)        _param_vm_inertia_xx,
            (ParamFloat<px4::params::VM_INERTIA_YY>)        _param_vm_inertia_yy,
            (ParamFloat<px4::params::VM_INERTIA_ZZ>)        _param_vm_inertia_zz,
            (ParamFloat<px4::params::VM_INERTIA_XY>)        _param_vm_inertia_xy,
            (ParamFloat<px4::params::VM_INERTIA_XZ>)        _param_vm_inertia_xz,
            (ParamFloat<px4::params::VM_INERTIA_YZ>)        _param_vm_inertia_yz,
            (ParamFloat<px4::params::VM_LEN_SCALE_X>)       _param_vm_len_scale_x,
            (ParamFloat<px4::params::VM_LEN_SCALE_Y>)       _param_vm_len_scale_y,
            (ParamFloat<px4::params::VM_LEN_SCALE_Z>)       _param_vm_len_scale_z,
            (ParamFloat<px4::params::VM_COG_OFF_X>)         _param_vm_cog_off_x,
            (ParamFloat<px4::params::VM_COG_OFF_Y>)         _param_vm_cog_off_y,
            (ParamFloat<px4::params::VM_COG_OFF_Z>)         _param_vm_cog_off_z,
            (ParamFloat<px4::params::VM_COT_OFF_X>)         _param_vm_cot_off_x,
            (ParamFloat<px4::params::VM_COT_OFF_Y>)         _param_vm_cot_off_y,
            (ParamFloat<px4::params::VM_COT_OFF_Z>)         _param_vm_cot_off_z,
            (ParamFloat<px4::params::VM_BAT_INTR_CELL>)     _param_vm_bat_intr_cell,
            (ParamFloat<px4::params::VM_BAT_REF_VOLT>)      _param_vm_bat_ref_volt,
            (ParamExtInt<px4::params::IV_IMU_DELAY_US>)     _param_iv_imu_delay_us,
            (ParamExtInt<px4::params::VIMU_PREDICT_US>)     _param_vimu_predict_us,
            (ParamInt<px4::params::VIMU_FUSE_GYRO>)         _param_vimu_fuse_gyro,
            (ParamInt<px4::params::VIMU_TAKE_BIAS>)         _param_vimu_take_bias
    )
};
