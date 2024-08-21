
/****************************************************************************
 *
 *   Copyright (c) 2018-2021 PX4 Development Team. All rights reserved.
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

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/conversion/rotation.h>
#include <lib/fault_detector/fault_detector.hpp>
#include <lib/sensor_attack/sensor_attack.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_gyro.h>
#include <uORB/topics/sensor_gyro_fifo.h>
#include <uORB/topics/sensor_gyro_errors.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_angular_acceleration.h>

using namespace time_literals;
using fault_detector::GyroValidator;

class PX4Gyroscope : public ModuleParams
{
public:
	PX4Gyroscope(uint32_t device_id, enum Rotation rotation = ROTATION_NONE);
	~PX4Gyroscope();

	uint32_t get_device_id() const { return _device_id; }

	int32_t get_max_rate_hz() const { return math::constrain(_imu_gyro_rate_max, static_cast<int32_t>(100), static_cast<int32_t>(4000)); }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype);
	void set_error_count(uint32_t error_count) { _error_count = error_count; }
	void set_range(float range) { _range = range; }
	void set_scale(float scale);
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(const hrt_abstime &timestamp_sample, float x, float y, float z);

	void updateFIFO(sensor_gyro_fifo_s &sample);

	int get_instance() { return _sensor_pub.get_instance(); };

private:
    // Note: this value is borrowed from data_validator
    static const constexpr unsigned NORETURN_ERRCOUNT = 10000;
    /**< if the error count reaches this value, return sensor as invalid */

    bool ParametersUpdate();
    void updateReference(const hrt_abstime &timestamp_sample);

    void validateGyro(sensor_gyro_s &gyro);

    bool attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const;

    void applyGyroAttack(sensor_gyro_s &gyro);
    void applyGyroAttack(sensor_gyro_s &gyro, sensor_gyro_fifo_s &gyro_fifo);

    float getMaxDeviation() const;

	uORB::PublicationMulti<sensor_gyro_s> _sensor_pub{ORB_ID(sensor_gyro)};
	uORB::PublicationMulti<sensor_gyro_fifo_s>  _sensor_fifo_pub{ORB_ID(sensor_gyro_fifo)};
    uORB::PublicationMulti<sensor_gyro_errors_s> _sensor_gyro_errors_pub{ORB_ID(sensor_gyro_errors)};

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription         _reference_gyro_sub{ORB_ID(reference_gyro)};
	uORB::Subscription         _vehicle_angular_velocity_sub{ORB_ID(vehicle_angular_velocity)};
	uORB::Subscription         _reference_angular_acceleration_sub{ORB_ID(reference_angular_acceleration)};

	uint32_t		_device_id{0};
	const enum Rotation	_rotation;

	int32_t			_imu_gyro_rate_max{0};

	float			_range{math::radians(2000.f)};
	float			_scale{1.f};
	float			_temperature{NAN};

	uint32_t		_error_count{0};

	int16_t			_last_sample[3] {};

    GyroValidator::ParamStruct  _gyro_validator_params{};
    GyroValidator               _gyro_validator{&_gyro_validator_params};
    sensor_gyro_s               _curr_ref_gyro{};
    sensor_gyro_s               _next_ref_gyro{};
    vehicle_angular_velocity_s  _last_angular_rates{};

    int  _attack_flag_prev{0};
    int  _attack_has_start{0};
    hrt_abstime _attack_timestamp{0};
    float   _last_deviation[3] {};

    DEFINE_PARAMETERS(
            (ParamInt<px4::params::ATK_APPLY_TYPE>) _param_atk_apply_type,
            (ParamInt<px4::params::ATK_STEALTH_TYPE>) _param_atk_stealth_type,
            (ParamInt<px4::params::ATK_COUNTDOWN_MS>) _param_atk_countdown_ms,
            (ParamInt<px4::params::ATK_MULTI_IMU>) _param_atk_multi_imu,
            (ParamFloat<px4::params::ATK_GYR_AMP>) _param_atk_gyr_amp,
            (ParamFloat<px4::params::ATK_GYR_FREQ>) _param_atk_gyr_freq,
            (ParamFloat<px4::params::ATK_GYR_PHASE>) _param_atk_gyr_phase,

            (ParamInt<px4::params::IV_DELAY_MASK>) _param_iv_delay_mask,
            (ParamInt<px4::params::IV_TTD_DELAY_MS>) _param_iv_ttd_delay_ms,
            (ParamFloat<px4::params::IV_GYR_NOISE>) _param_iv_gyr_noise,
            (ParamExtFloat <px4::params::IV_GYR_CSUM_H>) _param_iv_gyr_csum_h,
            (ParamExtFloat <px4::params::IV_GYR_MSHIFT>) _param_iv_gyr_mshift,
            (ParamExtFloat<px4::params::IV_GYR_EMA_H>) _param_iv_gyr_ema_h,
            (ParamExtFloat<px4::params::IV_GYR_ALPHA>) _param_iv_gyr_alpha,
            (ParamExtFloat<px4::params::IV_GYR_EMA_CAP>) _param_iv_gyr_ema_cap
    )
};
