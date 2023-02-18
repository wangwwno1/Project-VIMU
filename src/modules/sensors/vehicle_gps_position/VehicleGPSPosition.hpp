/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

#include <memory>
#include <lib/fault_detector/fault_detector.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/sensor_attack/sensor_attack.hpp>
#include <lib/perf/perf_counter.h>
#include <modules/ekf2/EKF/RingBuffer.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_offset_states.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_gps_error.h>
#include <uORB/topics/sensors_status_gps.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_local_position.h>

#include "gps_blending.hpp"

using namespace time_literals;
using fault_detector::GPSPosValidator;
using fault_detector::GPSVelValidator;
using matrix::Quatf;

namespace sensors
{

struct RefGpsSample {
    hrt_abstime time_us{0};
    Quatf q;
    Vector3f pos;
    Vector3f pos_var;
    Vector3f vel;
    Vector3f vel_var;
    Vector3f ang_rate_delayed_raw;
    float    dt_ekf_avg;
};


class VehicleGPSPosition : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleGPSPosition();
	~VehicleGPSPosition() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void ParametersUpdate(bool force = false);
	void Publish(const sensor_gps_s &gps, uint8_t selected);
    void PublishErrorStatus();
    void PublishSensorStatus();

    void UpdateReferenceState();

    bool attack_enabled(const uint8_t &attack_type) const;
	void ConductVelocitySpoofing(sensor_gps_s &gps_position);
    bool ConductVelocitySpoofing(sensor_gps_s &gps_position, const Vector3f &ref_vel_board);
    void ConductPositionSpoofing(sensor_gps_s &gps_position);
    bool ConductPositionSpoofing(sensor_gps_s &gps_position, const Vector3f &ref_pos_board);
    void ValidateGpsData(sensor_gps_s &gps_position);
    void ReplaceGpsPosVelData(sensor_gps_s &gps_position, const Vector3f &ref_pos_board, const Vector3f &ref_vel_board);

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	uORB::Publication<vehicle_gps_position_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};
    // TODO Remove debug topics
    uORB::Publication<sensor_gps_error_s>     _sensor_gps_error_pub{ORB_ID(sensor_gps_error)};
    uORB::Publication<sensor_gps_error_s>     _sensor_gps_error_variances_pub{ORB_ID(sensor_gps_error_variances)};
    uORB::Publication<sensor_gps_error_s>     _sensor_gps_error_ratios_pub{ORB_ID(sensor_gps_error_ratios)};
    uORB::Publication<sensors_status_gps_s>   _sensors_status_gps_pub{ORB_ID(sensors_status_gps)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

    uORB::SubscriptionCallbackWorkItem  _vehicle_reference_states_sub{this, ORB_ID(vehicle_reference_states)};
    uORB::Subscription                  _reference_offset_states_sub{ORB_ID(estimator_offset_states)};
    uORB::Subscription                  _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

    Vector3f            _last_pos_error{0.f, 0.f, 0.f};
    Vector3f            _last_vel_error{0.f , 0.f, 0.f};
    Vector3f            _last_pos_vars{0.f, 0.f, 0.f};
    Vector3f            _last_vel_vars{0.f ,0.f, 0.f};
    MapProjection       _global_origin{};
    float               _gps_alt_ref{0.f};

    Vector3f            _gps_pos_body{};

    GPSPosValidator::ParamStruct    _pos_validator_params{};
    GPSVelValidator::ParamStruct    _vel_validator_params{};
    GPSPosValidator                 _pos_validator{&_pos_validator_params};
    GPSVelValidator                 _vel_validator{&_vel_validator_params};

    int                 _attack_flag_prev{0};
    hrt_abstime         _attack_timestamp{0};
    hrt_abstime         _last_health_status_publish{0};
    bool                _gps_healthy_prev{true};
    bool                _gps_healthy{true};

    RingBuffer<RefGpsSample> *_ref_gps_buffer{nullptr};
    RefGpsSample        _ref_gps_delayed{};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GpsBlending _gps_blending;

    sensor_attack::DeviationParams              _pos_atk_params{};
	std::unique_ptr<sensor_attack::Deviation>   _pos_deviation = nullptr;
    sensor_attack::DeviationParams              _vel_atk_params{};
    std::unique_ptr<sensor_attack::Deviation>   _vel_deviation = nullptr;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_GPS_MASK>)          _param_sens_gps_mask,
		(ParamFloat<px4::params::SENS_GPS_TAU>)         _param_sens_gps_tau,
		(ParamInt<px4::params::SENS_GPS_PRIME>)         _param_sens_gps_prime,

        (ParamInt<px4::params::EKF2_PREDICT_US>)        _param_ekf2_predict_us,
        (ParamFloat<px4::params::EKF2_GPS_DELAY>)       _param_ekf2_gps_delay,
        ///< GPS measurement delay relative to the IMU (mSec)
        (ParamFloat<px4::params::EKF2_GPS_V_NOISE>)     _param_ekf2_gps_v_noise,
        ///< minimum allowed observation noise for gps velocity fusion (m/sec)
        (ParamFloat<px4::params::EKF2_GPS_P_NOISE>)     _param_ekf2_gps_p_noise,
        ///< minimum allowed observation noise for gps position fusion (m)
        (ParamExtFloat<px4::params::EKF2_GPS_POS_X>)    _param_ekf2_gps_pos_x,
        ///< X position of GPS antenna in body frame (m)
        (ParamExtFloat<px4::params::EKF2_GPS_POS_Y>)    _param_ekf2_gps_pos_y,
        ///< Y position of GPS antenna in body frame (m)
        (ParamExtFloat<px4::params::EKF2_GPS_POS_Z>)    _param_ekf2_gps_pos_z,
        ///< Z position of GPS antenna in body frame (m)
        (ParamFloat<px4::params::EKF2_GPS_P_GATE>)  _param_ekf2_gps_p_gate,
        ///< GPS horizontal position innovation consistency gate size (STD)
        (ParamFloat<px4::params::EKF2_GPS_V_GATE>) _param_ekf2_gps_v_gate,
        ///< GPS velocity innovation consistency gate size (STD)

        (ParamInt<px4::params::ATK_APPLY_TYPE>)         _param_atk_apply_type,
        (ParamInt<px4::params::ATK_STEALTH_TYPE>)       _param_atk_stealth_type,
        (ParamInt<px4::params::ATK_COUNTDOWN_MS>)       _param_atk_countdown_ms,
        (ParamInt<px4::params::ATK_GPS_P_CLS>)          _param_atk_gps_p_cls,
		(ParamExtFloat<px4::params::ATK_GPS_P_IV>)      _param_atk_gps_p_iv,
		(ParamExtFloat<px4::params::ATK_GPS_P_RATE>)    _param_atk_gps_p_rate,
		(ParamExtFloat<px4::params::ATK_GPS_P_CAP>)     _param_atk_gps_p_cap,
		(ParamExtFloat<px4::params::ATK_GPS_P_HDG>)     _param_atk_gps_p_hdg,
		(ParamExtFloat<px4::params::ATK_GPS_P_PITCH>)   _param_atk_gps_p_pitch,
        (ParamInt<px4::params::ATK_GPS_V_CLS>)          _param_atk_gps_v_cls,
        (ParamExtFloat<px4::params::ATK_GPS_V_IV>)      _param_atk_gps_v_iv,
        (ParamExtFloat<px4::params::ATK_GPS_V_RATE>)    _param_atk_gps_v_rate,
        (ParamExtFloat<px4::params::ATK_GPS_V_CAP>)     _param_atk_gps_v_cap,
        (ParamExtFloat<px4::params::ATK_GPS_V_HDG>)     _param_atk_gps_v_hdg,
        (ParamExtFloat<px4::params::ATK_GPS_V_PITCH>)   _param_atk_gps_v_pitch,

        (ParamInt<px4::params::IV_DEBUG_LOG>)           _param_iv_debug_log,
        (ParamInt<px4::params::IV_DELAY_MASK>)          _param_iv_delay_mask,
        (ParamInt<px4::params::IV_TTD_DELAY_MS>)        _param_iv_ttd_delay_ms,
        (ParamExtFloat<px4::params::IV_GPS_P_CSUM_H>)   _param_iv_gps_p_csum_h,
        (ParamExtFloat<px4::params::IV_GPS_P_MSHIFT>)   _param_iv_gps_p_mshift,
        (ParamFloat<px4::params::IV_GPS_P_EMA_H>)       _param_iv_gps_p_ema_h,
        (ParamFloat<px4::params::IV_GPS_P_ALPHA>)       _param_iv_gps_p_alpha,
        (ParamFloat<px4::params::IV_GPS_P_EMA_CAP>)     _param_iv_gps_p_ema_cap,
        (ParamFloat<px4::params::IV_GPS_P_TWIN_H>)      _param_iv_gps_p_twin_h,
        (ParamInt<px4::params::IV_GPS_P_RST_CNT>)       _param_iv_gps_p_rst_cnt,
        (ParamInt<px4::params::IV_GPS_P_CD_CNT>)        _param_iv_gps_p_cd_cnt,
        (ParamExtFloat<px4::params::IV_GPS_V_CSUM_H>)   _param_iv_gps_v_csum_h,
        (ParamExtFloat<px4::params::IV_GPS_V_MSHIFT>)   _param_iv_gps_v_mshift,
        (ParamFloat<px4::params::IV_GPS_V_EMA_H>)       _param_iv_gps_v_ema_h,
        (ParamFloat<px4::params::IV_GPS_V_ALPHA>)       _param_iv_gps_v_alpha,
        (ParamFloat<px4::params::IV_GPS_V_EMA_CAP>)     _param_iv_gps_v_ema_cap,
        (ParamFloat<px4::params::IV_GPS_V_TWIN_H>)      _param_iv_gps_v_twin_h,
        (ParamInt<px4::params::IV_GPS_V_RST_CNT>)       _param_iv_gps_v_rst_cnt,
        (ParamInt<px4::params::IV_GPS_V_CD_CNT>)        _param_iv_gps_v_cd_cnt
	)
};
}; // namespace sensors
