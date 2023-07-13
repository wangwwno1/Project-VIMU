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
// Correct Mixer for Roll 0304/08_08_49 - Pitch & Yaw Reversed
// Note: This is the most likely correct mixer - 0304/09_05_33
//static const float flat_mat[3 * 4] = {-1,  1,  1,  -1,
//                                      1, -1,  1,  -1,
//                                      1,  1, -1,  -1};
// todo replace with ControlAllocator Parameter (CA_ROTOR*)
//static const matrix::Matrix<float, 3, noutputs> QuadMotorPosition((float[16]) { -1, +1, +1, -1,
//                                                                                +1, -1, +1, -1,
//                                                                                 0,  0,  0,  0});
static const matrix::Matrix<float, 3, noutputs> QuadMotorPosition((float[16]) { +0.5, -0.5, +0.5, -0.5,
                                                                                +0.5, -0.5, -0.5, +0.5,
                                                                                 0.0,  0.0,  0.0,  0.0});
static const matrix::Matrix<float, 3, noutputs> QuadRotationAxis((float[16]) { -0, +0, +0, -0,
                                                                               +0, -0, +0, -0,
                                                                               +1, +1, -1, -1});;
static const matrix::Matrix<float, 3, noutputs> QuadThrustAxis((float[16]) { 0,  0,  0,  0,
                                                                             0,  0,  0,  0,
                                                                            -1, -1, -1, -1});
// The thrust axis of ZD550 is not vertical to the body frame, ~ 85 deg to the vertical
// With deviated CoG, the yaw estimation could quickly go off
// const ta_x = 0.06163
// const ta_z = 0.9962
//static const matrix::Matrix<float, 3, noutputs> QuadThrustAxis((float[16]) {-ta_x, +ta_x  -ta_x, +ta_x,
//                                                                            -ta_x, +ta_x, +ta_x, -ta_x,
//                                                                            -ta_z, -ta_z, -ta_z, -ta_z});

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

    bool CalculateActuatorState(const float dt, VectorThrust &actuator_state, const VectorThrust &setpoint) {
        if (_phys_model_params.motor_time_constant > 1e-5f) {
            if (dt > 1.e-6f) {
                // fixme replace exponential with LUT for dynamic time & saving computation
                const float weight = 1.f - expf(- dt / _phys_model_params.motor_time_constant);
                actuator_state += (setpoint - actuator_state) * weight;
                return true;
            }
        } else {
            actuator_state = setpoint;
            return true;
        }

        return false;
    };

    Vector3f CalculateThrust(const float actuator_state, const Vector3f &thrust_axis) const {
        return _phys_model_params.Ct * actuator_state * thrust_axis;
    }

    Vector3f CalculateThrust(const VectorThrust &actuator_state) const {
        return static_cast<Vector3f> (QuadThrustAxis * actuator_state) * _phys_model_params.Ct;
    }

    void CalculateThrustAndTorque(const VectorThrust &actuator_state, Vector3f &thrust, Vector3f &torque) {
        Vector3f thrust_tmp{};
        Vector3f torque_tmp{};
        for (int i = 0; i < noutputs; i++) {
            // Get the ith motor position relative to center of gravity
            // for calculating the torque generated by the motor thrust
            // TODO: Store the rel position to CoG, only update them when the parameter is changed.
            const Vector3f rel_pos_to_cog = static_cast<Vector3f> (QuadMotorPosition.col(i)).emult(_phys_model_params.length) - _phys_model_params.center_of_gravity;
            const Vector3f axis = static_cast<Vector3f> (QuadThrustAxis.col(i));

            // Calculate the thrust vector of ith motor
            const Vector3f ith_thrust = CalculateThrust(actuator_state(i), axis);

            // Calculate the torque generated by the thrust of ith motor and accumulate the torque and thrust.
            torque_tmp += rel_pos_to_cog.cross(ith_thrust);
            thrust_tmp += ith_thrust;
        }
        // Calculate the torque generated by the spinning propeller
        // todo replace with ControlAllocator Style Cd (CW < 0, CCW > 0)
        torque_tmp += static_cast<Vector3f>(QuadRotationAxis * actuator_state) * _phys_model_params.Cd;

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
        float motor_time_constant{0.005f};  // 5ms for jMAVSim default
        Vector3f length{1.f, 1.f, 1.f};  // 0.233345 for jMAVSim
        Vector3f center_of_gravity{0.f, 0.f, 0.f};
    };

    CopterPhysModelParams _phys_model_params{};

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
    uORB::PublicationMulti<sensor_combined_s>                _reference_combined_pub{ORB_ID(reference_combined)};
    uORB::PublicationMulti<vehicle_imu_s>                    _reference_imu_pub{ORB_ID(reference_imu)};

	// Subscriptions
    uORB::SubscriptionCallbackWorkItem                  _actuator_outputs_sub{this, ORB_ID(actuator_outputs)};      // subscription that schedules VirtualIMU when updated
    uORB::SubscriptionMultiArray<vehicle_imu_s>         _vehicle_imu_sub{ORB_ID::vehicle_imu};
	uORB::SubscriptionInterval                          _parameter_update_sub{ORB_ID(parameter_update), 1_s};       // subscription limited to 1 Hz updates
    uORB::Subscription                                  _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};
	uORB::Subscription                                  _estimator_aero_wrench_sub{ORB_ID(estimator_aero_wrench)};
    uORB::Subscription                                  _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
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
            (ParamExtInt<px4::params::IV_IMU_DELAY_US>)     _param_iv_imu_delay_us,
            (ParamExtInt<px4::params::VIMU_PREDICT_US>)     _param_vimu_predict_us
    )
};
