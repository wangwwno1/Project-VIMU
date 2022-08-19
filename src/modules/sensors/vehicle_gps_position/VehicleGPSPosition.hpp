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
#include <lib/mathlib/math/Limits.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/vehicle_gps_position.h>

#include "gps_blending.hpp"
#include "../sensor_attack/sensor_attack.h"

using namespace time_literals;

namespace sensors
{
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

	void ConductAttack(vehicle_gps_position_s &gps_position);

    // defines the gps attack mask position for sensor injection attack
    static constexpr uint8_t ATK_MASK_GPS_POS = (1 << 2);
    static constexpr uint8_t ATK_MASK_GPS_VEL = (1 << 3);

	// defines used to specify the mask position for use of different accuracy metrics in the GPS blending algorithm
	static constexpr uint8_t BLEND_MASK_USE_SPD_ACC  = 1;
	static constexpr uint8_t BLEND_MASK_USE_HPOS_ACC = 2;
	static constexpr uint8_t BLEND_MASK_USE_VPOS_ACC = 4;

	// define max number of GPS receivers supported
	static constexpr int GPS_MAX_RECEIVERS = 2;
	static_assert(GPS_MAX_RECEIVERS == GpsBlending::GPS_MAX_RECEIVERS_BLEND,
		      "GPS_MAX_RECEIVERS must match to GPS_MAX_RECEIVERS_BLEND");

	uORB::Publication<vehicle_gps_position_s> _vehicle_gps_position_pub{ORB_ID(vehicle_gps_position)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _sensor_gps_sub[GPS_MAX_RECEIVERS] {	/**< sensor data subscription */
		{this, ORB_ID(sensor_gps), 0},
		{this, ORB_ID(sensor_gps), 1},
	};

	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};

	GpsBlending _gps_blending;

    sensor_attack::DeviationParams              _pos_param{};
	std::unique_ptr<sensor_attack::Deviation>   _pos_deviation = nullptr;
    sensor_attack::DeviationParams              _vel_param{};
    std::unique_ptr<sensor_attack::Deviation>   _vel_deviation = nullptr;

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_GPS_MASK>)          _param_sens_gps_mask,
		(ParamFloat<px4::params::SENS_GPS_TAU>)         _param_sens_gps_tau,
		(ParamInt<px4::params::SENS_GPS_PRIME>)         _param_sens_gps_prime,
        (ParamInt<px4::params::ATK_APPLY_TYPE>)         _param_atk_apply_type,
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
        (ParamExtFloat<px4::params::ATK_GPS_V_PITCH>)   _param_atk_gps_v_pitch
	)
};
}; // namespace sensors
