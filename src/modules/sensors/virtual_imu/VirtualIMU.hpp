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
#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
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
// Correct Mixer for Roll 0304/08_08_49 - Pitch & Yaw Reversed
// Note: This is the most likely correct mixer - 0304/09_05_33
//static const float flat_mat[3 * 4] = {-1,  1,  1,  -1,
//                                      1, -1,  1,  -1,
//                                      1,  1, -1,  -1};
static const matrix::Matrix<float, 3, noutputs> QuadTorque((float[16]) { -1, +1, +1, -1,
                                                                         +1, -1, +1, -1,
                                                                         +1, +1, -1, -1});
static const matrix::Matrix<float, 3, noutputs> QuadThrust((float[16]) {0, 0, 0, 0,
                                                                        0, 0, 0, 0,
                                                                        1, 1, 1, 1});

class VirtualIMU : public ModuleParams, public px4::ScheduledWorkItem
{
public:
    VirtualIMU();
    VirtualIMU(bool delayed_motor_response);
	~VirtualIMU() override;

    bool Start();
    void Stop();

    void PrintStatus();

    bool multi_init(int instance);

    static constexpr uint8_t MAX_SENSOR_COUNT = 4;

private:

	void Run() override;

    void reset();

    void ParameterUpdate(bool force = false);

    void UpdateCopterStatus();
    void UpdateIMUData();
    void UpdateAerodynamicWrench();
    void UpdateSensorBias();

    void PublishAngularVelocityAndAcceleration();
    void PublishReferenceIMU();
    void PublishSensorReference();

    bool CheckAndSyncTimestamp(const uint8_t i, const hrt_abstime &imu_timestamp);

    static constexpr float sq(float x) { return x * x; };

    bool _delayed_motor_response{true};
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
    bool _interval_configured{false};
    float _imu_integration_interval;
    float _rate_ctrl_interval;
    hrt_abstime _rate_ctrl_interval_us;
    hrt_abstime _imu_integration_interval_us;
    hrt_abstime _last_integrator_reset{0};
    Integrator _accel_integrator{};
    IntegratorConing _gyro_integrator{};

    // Actuator timestamps and state
    hrt_abstime _last_update_us{0};
    Vector16f _current_actuator_setpoint;
    hrt_abstime _current_act_sp_timestamp{0};
    Vector16f _newest_actuator_setpoint;
    hrt_abstime _newest_act_sp_timestamp{0};
    AlphaFilter<Vector16f> _actuator_state_lpf{1.f};

    // External Wrench
    Vector3f _external_accel{0.f, 0.f, 0.f};
    Vector3f _external_angular_accel{0.f, 0.f, 0.f};

    // Sensor Bias
    Vector3f _body_acceleration{0.f, 0.f, 0.f};
    Vector3f _accel_bias{};

    // (Copter) Physical Model Parameters
    struct CopterPhysModelParams {
        float mass{0.8f};
        float Ct{4.00f};
        float Cd{0.05f};
        float motor_time_constant{0.005f};  // 5ms for jMAVSim default
        Vector2f length{0.33f / (2.0f * sqrt(2.0f)), 0.33f / (2.0f * sqrt(2.0f))};  // 0.11667262
        Vector3f torque_coeff{Ct * length(0), Ct * length(1), Cd};
        Vector3f thrust_coeff{0.f, 0.f, -Ct / mass};
    };

    CopterPhysModelParams _phys_model_params{};

    // IMU health status that control AuxEKF angular velocity fusion
    bool _imu_health_status[MAX_SENSOR_COUNT];

    // Auxiliary EKF for angular velocity estimate
    AuxEKF _ekf{};
    AuxEKFParam *_ekf_params;

    int _instance{0};

    // Publications
    uORB::PublicationMulti<vehicle_angular_acceleration_s>   _reference_angular_acceleration_pub{ORB_ID(reference_angular_acceleration)};
    uORB::PublicationMulti<vehicle_angular_velocity_s>       _reference_angular_velocity{ORB_ID(reference_angular_velocity)};
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
            (ParamFloat<px4::params::EKF2_ACC_NOISE>)       _param_ekf2_acc_noise,
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
            (ParamExtInt<px4::params::IV_IMU_DELAY_US>)     _param_iv_imu_delay_us,
            (ParamExtInt<px4::params::VIMU_PREDICT_US>)     _param_vimu_predict_us
    )
};
