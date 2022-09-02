//
// Created by Robert Wang on 2022/8/26.
//

#ifndef PX4_SOFTWARESENSOR_H
#define PX4_SOFTWARESENSOR_H

#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>

#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_imu.h>

#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_rates_setpoint.h>

#include "../vehicle_imu/Integrator.hpp"
#include "LinearStateModel/LinearStateModel.h"
#include "model_parameters.h"

using matrix::Eulerf;
using matrix::AxisAnglef;
using matrix::Quatf;
using matrix::Vector3f;
using lsm::LinearStateModel;
using namespace lsm_params;
using namespace time_literals;

class SoftwareSensor : public ModuleParams, public px4::ScheduledWorkItem
{
public:
    SoftwareSensor();
    ~SoftwareSensor() override;

    bool multi_init(int instance);

    bool Start();
    void Stop();

    void PrintStatus();

    static constexpr uint8_t MAX_SENSOR_COUNT = 4;
    static constexpr uint8_t REFERENCE_UORB_ID = 0;

private:
    hrt_abstime _filter_update_period_us{10_ms};
    float    _filter_update_period{_filter_update_period_us * 1e-6f};

    void Run() override;

    void reset();

    void ParameterUpdate(bool force = false);

    void UpdateCopterStatus();
    void UpdatePosVelState();
    void UpdateAttitude();
    void UpdateAngularVelocityAndAcceleration();

    void PublishAngularVelocityAndAcceleration();
    void PublishReferenceIMU();
    void PublishReferenceState();
    void PublishReferenceGyroAndAccelerometer();

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

    hrt_abstime _last_update_us{0};

    // Imu Integrators
    bool _interval_configured{false};
    float _imu_integration_interval;
    float _rate_ctrl_interval;
    hrt_abstime _rate_ctrl_interval_us;
    hrt_abstime _imu_integration_interval_us;
    hrt_abstime _last_integrator_reset{0};

    IntegratorConing _gyro_integrator{};

    // Linear State Model
    LinearStateModel  _pos_model{POS_MODEL_A, POS_MODEL_B, POS_MODEL_C, POS_MODEL_D};
    LinearStateModel  _vel_model{VEL_MODEL_A, VEL_MODEL_B, VEL_MODEL_C, VEL_MODEL_D};
    LinearStateModel  _att_model{ATT_MODEL_A, ATT_MODEL_B, ATT_MODEL_C, ATT_MODEL_D};
    LinearStateModel  _rate_model{RATE_MODEL_A, RATE_MODEL_B, RATE_MODEL_C, RATE_MODEL_D};

    Vector3f                _avg_acceleration{0.f, 0.f, 0.f};
    Vector3f                _delta_vel{0.f, 0.f, 0.f};
    AlphaFilter<Vector3f>   _angular_accel_filter{0.2f};  // TODO determine the best weight

    struct LocalPosVelOffset {
        hrt_abstime timestamp{0};
        Vector3f    pos_offset{0.f, 0.f, 0.f};
        Vector3f    vel_offset{0.f, 0.f, 0.f};
    };

    LocalPosVelOffset       _offsets{};
    uint8_t                 _vxy_reset_counter{0};
    uint8_t                 _vz_reset_counter{0};
    uint8_t                 _xy_reset_counter{0};
    uint8_t                 _z_reset_counter{0};

    struct VehicleState {
        Vector3f pos{0.f, 0.f, 0.f};
        Vector3f vel{0.f, 0.f, 0.f};
        Eulerf   att{0.f, 0.f, 0.f};
        Eulerf   rates{0.f, 0.f, 0.f};
    };

    VehicleState    _state{};


    MapProjection   _global_origin{};
    float           _gps_alt_ref{0.f};

    int _instance{0};

    estimator_states_s      _reference_states{};

    // Publications
    uORB::PublicationMulti<vehicle_angular_acceleration_s>   _reference_angular_acceleration_pub{ORB_ID(reference_angular_acceleration)};
    uORB::PublicationMulti<vehicle_angular_velocity_s>       _reference_angular_velocity_pub{ORB_ID(reference_angular_velocity)};
    uORB::PublicationMulti<sensor_accel_s>                   _reference_accel_pub{ORB_ID(reference_accel)};
    uORB::PublicationMulti<sensor_gyro_s>                    _reference_gyro_pub{ORB_ID(reference_gyro)};
    uORB::PublicationMulti<sensor_combined_s>                _reference_combined_pub{ORB_ID(reference_combined)};
    uORB::PublicationMulti<vehicle_imu_s>                    _reference_imu_pub{ORB_ID(reference_imu)};
    uORB::PublicationMulti<estimator_states_s>               _reference_state_pub{ORB_ID(vehicle_reference_states)};

    // Subscriptions
    uORB::SubscriptionCallbackWorkItem                  _actuator_outputs_sub{this, ORB_ID(actuator_outputs)};      // subscription that schedules SoftwareSensor when updated
    uORB::SubscriptionInterval                          _parameter_update_sub{ORB_ID(parameter_update), 1_s};       // subscription limited to 1 Hz updates
    uORB::Subscription                                  _estimator_states_sub{ORB_ID(estimator_states), REFERENCE_UORB_ID};
    uORB::Subscription                                  _local_pos_sub{ORB_ID(vehicle_local_position)};
    uORB::Subscription                                  _local_pos_sp_sub{ORB_ID(vehicle_local_position_setpoint)};
    uORB::Subscription                                  _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
    uORB::Subscription                                  _vehicle_attitude_setpoint_sub{ORB_ID(vehicle_attitude_setpoint)};
    uORB::Subscription                                  _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
    uORB::Subscription                                  _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
    uORB::Subscription                                  _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};

    // Parameters
    DEFINE_PARAMETERS(
            (ParamInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
            (ParamInt<px4::params::IMU_INTEG_RATE>)         _param_imu_integ_rate,
            (ParamInt<px4::params::IMU_GYRO_RATEMAX>)       _param_imu_gyro_ratemax
    )

    void AdjustOffsetAndGlobalOrigin();
};

#endif //PX4_SOFTWARESENSOR_H
