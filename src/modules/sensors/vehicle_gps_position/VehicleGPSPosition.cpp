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

#include "VehicleGPSPosition.hpp"

#include <px4_platform_common/log.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

namespace sensors
{
VehicleGPSPosition::VehicleGPSPosition() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
    _param_atk_gps_p_iv(_pos_param.initial_value),
    _param_atk_gps_p_rate(_pos_param.rate_of_rise),
    _param_atk_gps_p_cap(_pos_param.max_deviation),
    _param_atk_gps_p_hdg(_pos_param.heading_deg),
    _param_atk_gps_p_pitch(_pos_param.pitch_deg),
    _param_atk_gps_v_iv(_vel_param.initial_value),
    _param_atk_gps_v_rate(_vel_param.rate_of_rise),
    _param_atk_gps_v_cap(_vel_param.max_deviation),
    _param_atk_gps_v_hdg(_vel_param.heading_deg),
    _param_atk_gps_v_pitch(_vel_param.pitch_deg)
{
	_vehicle_gps_position_pub.advertise();
}

VehicleGPSPosition::~VehicleGPSPosition()
{
	Stop();
	perf_free(_cycle_perf);
}

bool VehicleGPSPosition::Start()
{
	// force initial updates
	ParametersUpdate(true);

	ScheduleNow();

	return true;
}

void VehicleGPSPosition::Stop()
{
	Deinit();

	// clear all registered callbacks
	for (auto &sub : _sensor_gps_sub) {
		sub.unregisterCallback();
	}
}

void VehicleGPSPosition::ParametersUpdate(bool force)
{
	// Check if parameters have changed
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();

		if (_param_sens_gps_mask.get() == 0) {
			_sensor_gps_sub[0].registerCallback();

		} else {
			for (auto &sub : _sensor_gps_sub) {
				sub.registerCallback();
			}
		}

		_gps_blending.setBlendingUseSpeedAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_SPD_ACC);
		_gps_blending.setBlendingUseHPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_HPOS_ACC);
		_gps_blending.setBlendingUseVPosAccuracy(_param_sens_gps_mask.get() & BLEND_MASK_USE_VPOS_ACC);
		_gps_blending.setBlendingTimeConstant(_param_sens_gps_tau.get());
		_gps_blending.setPrimaryInstance(_param_sens_gps_prime.get());
	}
}

void VehicleGPSPosition::Run()
{
	perf_begin(_cycle_perf);
	ParametersUpdate();

	// Check all GPS instance
	bool any_gps_updated = false;
	bool gps_updated = false;

	for (uint8_t i = 0; i < GPS_MAX_RECEIVERS; i++) {
		gps_updated = _sensor_gps_sub[i].updated();

		sensor_gps_s gps_data;

		if (gps_updated) {
			any_gps_updated = true;

			_sensor_gps_sub[i].copy(&gps_data);
			_gps_blending.setGpsData(gps_data, i);

			if (!_sensor_gps_sub[i].registered()) {
				_sensor_gps_sub[i].registerCallback();
			}
		}
	}

	if (any_gps_updated) {
		_gps_blending.update(hrt_absolute_time());

		if (_gps_blending.isNewOutputDataAvailable()) {
			Publish(_gps_blending.getOutputGpsData(), _gps_blending.getSelectedGps());
		}
	}

	ScheduleDelayed(300_ms); // backup schedule

	perf_end(_cycle_perf);
}

void VehicleGPSPosition::Publish(const sensor_gps_s &gps, uint8_t selected)
{
	vehicle_gps_position_s gps_output{};

	gps_output.timestamp = gps.timestamp;
	gps_output.time_utc_usec = gps.time_utc_usec;
	gps_output.lat = gps.lat;
	gps_output.lon = gps.lon;
	gps_output.alt = gps.alt;
	gps_output.alt_ellipsoid = gps.alt_ellipsoid;
	gps_output.s_variance_m_s = gps.s_variance_m_s;
	gps_output.c_variance_rad = gps.c_variance_rad;
	gps_output.eph = gps.eph;
	gps_output.epv = gps.epv;
	gps_output.hdop = gps.hdop;
	gps_output.vdop = gps.vdop;
	gps_output.noise_per_ms = gps.noise_per_ms;
	gps_output.jamming_indicator = gps.jamming_indicator;
	gps_output.jamming_state = gps.jamming_state;
	gps_output.vel_m_s = gps.vel_m_s;
	gps_output.vel_n_m_s = gps.vel_n_m_s;
	gps_output.vel_e_m_s = gps.vel_e_m_s;
	gps_output.vel_d_m_s = gps.vel_d_m_s;
	gps_output.cog_rad = gps.cog_rad;
	gps_output.timestamp_time_relative = gps.timestamp_time_relative;
	gps_output.heading = gps.heading;
	gps_output.heading_offset = gps.heading_offset;
	gps_output.fix_type = gps.fix_type;
	gps_output.vel_ned_valid = gps.vel_ned_valid;
	gps_output.satellites_used = gps.satellites_used;

	gps_output.selected = selected;

	ConductAttack(gps_output);

	_vehicle_gps_position_pub.publish(gps_output);
}

void VehicleGPSPosition::ConductAttack(vehicle_gps_position_s &gps_output)
{
    // TODO implement stealthy attack - This would locate inside VIMU-EKF
	if (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_POS) {
		if (!_pos_deviation) {
			PX4_INFO("Initiate GPS Position Spoofing Attack");
			_pos_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_p_cls.get(), &_pos_param));

            const float time_to_max_deviation_s = (*_pos_deviation).time_to_max_deviation();
            if (PX4_ISFINITE(time_to_max_deviation_s)) {
                PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
            } else {
                PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
            }

		}

		if (_pos_deviation) {
			const Vector3f deviation = (*_pos_deviation).calculate_deviation(gps_output.timestamp);
            sensor_attack::gps_position_spoofing(gps_output, deviation);
		}

	} else if (_pos_deviation) {
		_pos_deviation.reset();
        PX4_INFO("GPS Position Spoofing Stopped");
	}

    if (_param_atk_apply_type.get() & sensor_attack::ATK_GPS_VEL) {
        if (!_vel_deviation) {
            PX4_INFO("Initiate GPS Velocity Spoofing Attack");
            _vel_deviation.reset(sensor_attack::CreateAttackInstance(_param_atk_gps_v_cls.get(), &_vel_param));

            const float time_to_max_deviation_s = (*_vel_deviation).time_to_max_deviation();
            if (PX4_ISFINITE(time_to_max_deviation_s)) {
                PX4_INFO("Time to Max Deviation is: %.3f sec", (double)time_to_max_deviation_s);
            } else {
                PX4_WARN("INFINITE time to max deviation, please check attack parameter setting!");
            }

        }

        if (_vel_deviation) {
            const Vector3f deviation = (*_vel_deviation).calculate_deviation(gps_output.timestamp);
            sensor_attack::gps_velocity_spoofing(gps_output, deviation);
        }

    } else if (_vel_deviation) {
        _vel_deviation.reset();
        PX4_INFO("GPS Velocity Spoofing Stopped");
    }
}

void VehicleGPSPosition::PrintStatus()
{
	//PX4_INFO_RAW("[vehicle_gps_position] selected GPS: %d\n", _gps_select_index);
}

}; // namespace sensors
