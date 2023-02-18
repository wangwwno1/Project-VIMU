/****************************************************************************
 *
 *   Copyright (c) 2020-2022 PX4 Development Team. All rights reserved.
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

#include "data_validator/DataValidatorGroup.hpp"

#include <lib/sensor_calibration/Barometer.hpp>
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/systemlib/mavlink_log.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/differential_pressure.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_baro_error.h>
#include <uORB/topics/sensors_status.h>
#include <uORB/topics/vehicle_air_data.h>

#include <lib/sensor_attack/sensor_attack.hpp>
#include <lib/fault_detector/fault_detector.hpp>
#include <modules/ekf2/EKF/RingBuffer.h>
#include <uORB/topics/estimator_states.h>
#include <uORB/topics/estimator_offset_states.h>

using namespace time_literals;
using fault_detector::BaroValidator;

namespace sensors
{

struct RefBaroSample {
    hrt_abstime time_us{0};
    float alt_meter;
    float alt_var;
    float dt_ekf_avg;
    float hgt_offset;
};

class VehicleAirData : public ModuleParams, public px4::ScheduledWorkItem
{
public:

	VehicleAirData();
	~VehicleAirData() override;

	bool Start();
	void Stop();

	void PrintStatus();

private:
	void Run() override;

	void AirTemperatureUpdate();
	void CheckFailover(const hrt_abstime &time_now_us);
	bool ParametersUpdate(bool force = false);
	void UpdateStatus();
    void UpdateReferenceState();
    void ValidateBaroData(const uint8_t &instance, const hrt_abstime &timestamp_sample);

	float PressureToAltitude(float pressure_pa, float temperature = 15.f) const;
    float AltitudeToPressure(float altitude, float temperature = 15.f) const;

	static constexpr int MAX_SENSOR_COUNT = 4;

	uORB::Publication<sensors_status_s> _sensors_status_baro_pub{ORB_ID(sensors_status_baro)};
    uORB::Publication<sensor_baro_error_s> _sensor_baro_error_pub{ORB_ID(sensor_baro_error)};

	uORB::Publication<vehicle_air_data_s> _vehicle_air_data_pub{ORB_ID(vehicle_air_data)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::Subscription _differential_pressure_sub{ORB_ID(differential_pressure)};

	uORB::SubscriptionCallbackWorkItem _sensor_sub[MAX_SENSOR_COUNT] {
		{this, ORB_ID(sensor_baro), 0},
		{this, ORB_ID(sensor_baro), 1},
		{this, ORB_ID(sensor_baro), 2},
		{this, ORB_ID(sensor_baro), 3},
	};

    uORB::SubscriptionCallbackWorkItem  _vehicle_reference_states_sub{this, ORB_ID(vehicle_reference_states)};
    uORB::Subscription                  _reference_offset_states_sub{ORB_ID(estimator_offset_states)};

	calibration::Barometer _calibration[MAX_SENSOR_COUNT];

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	hrt_abstime _last_error_message{0};
	orb_advert_t _mavlink_log_pub{nullptr};

	DataValidatorGroup _voter{1};
	unsigned _last_failover_count{0};

    uint8_t     _attack_flag_prev{0};
    uint8_t     _instance_flag_prev{0};
    hrt_abstime _attack_timestamp{0};
    bool                        _forced_using_soft_baro{false};
    bool                        _status_updated{false};
    sensor_baro_error_s         _baro_error_status{};
    BaroValidator::ParamStruct  _baro_hgt_params{};
    BaroValidator               *_baro_validators[MAX_SENSOR_COUNT] {nullptr};
    RefBaroSample               _ref_baro_delayed{};
    RingBuffer<RefBaroSample>   *_ref_baro_buffer[MAX_SENSOR_COUNT]{nullptr};

	uint64_t _timestamp_sample_sum[MAX_SENSOR_COUNT] {0};
	float _data_sum[MAX_SENSOR_COUNT] {};
	float _temperature_sum[MAX_SENSOR_COUNT] {};
	int _data_sum_count[MAX_SENSOR_COUNT] {};
	hrt_abstime _last_publication_timestamp[MAX_SENSOR_COUNT] {};

	float _last_data[MAX_SENSOR_COUNT] {};
	bool _advertised[MAX_SENSOR_COUNT] {};

	float _sensor_diff[MAX_SENSOR_COUNT] {}; // filtered differences between sensor instances

	uint8_t _priority[MAX_SENSOR_COUNT] {};

	int8_t _selected_sensor_sub_index{-1};

	float _air_temperature_celsius{20.f}; // initialize with typical 20degC ambient temperature

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::SENS_BARO_QNH>) _param_sens_baro_qnh,
		(ParamFloat<px4::params::SENS_BARO_RATE>) _param_sens_baro_rate,

        (ParamInt<px4::params::EKF2_PREDICT_US>) _param_ekf2_predict_us,
        (ParamFloat<px4::params::EKF2_BARO_DELAY>) _param_ekf2_baro_delay,
        ///< barometer height measurement delay relative to the IMU (mSec),
        (ParamFloat<px4::params::EKF2_BARO_NOISE>) _param_ekf2_baro_noise,
        ///< observation noise for barometric height fusion (m)

        (ParamInt<px4::params::ATK_APPLY_TYPE>)         _param_atk_apply_type,
        (ParamInt<px4::params::ATK_COUNTDOWN_MS>)       _param_atk_countdown_ms,
        (ParamInt<px4::params::ATK_MULTI_BARO>)         _param_atk_multi_baro,
        (ParamInt<px4::params::IV_DEBUG_LOG>)           _param_iv_debug_log,
        (ParamExtFloat <px4::params::IV_BARO_TWIN_H>)   _param_iv_baro_twin_h,
        (ParamExtInt <px4::params::IV_BARO_RST_CNT>)    _param_iv_baro_rst_cnt,
        (ParamExtInt <px4::params::IV_BARO_CD_CNT>)     _param_iv_baro_cd_cnt
	)
};
}; // namespace sensors
