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
#include <lib/geo/geo.h>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/sensor_accel_fifo.h>
#include <uORB/topics/sensor_accel_errors.h>

using namespace time_literals;
using fault_detector::AccelValidator;

class PX4Accelerometer : public ModuleParams
{
public:
	PX4Accelerometer(uint32_t device_id, enum Rotation rotation = ROTATION_NONE);
	~PX4Accelerometer();

	uint32_t get_device_id() const { return _device_id; }

	int32_t get_max_rate_hz() const { return math::constrain(_imu_gyro_rate_max, static_cast<int32_t>(100), static_cast<int32_t>(4000)); }

	void set_device_id(uint32_t device_id) { _device_id = device_id; }
	void set_device_type(uint8_t devtype);
	void set_error_count(uint32_t error_count) { _error_count = error_count; }
	void set_range(float range) { _range = range; UpdateClipLimit(); }
	void set_scale(float scale);
	void set_temperature(float temperature) { _temperature = temperature; }

	void update(const hrt_abstime &timestamp_sample, float x, float y, float z);

	void updateFIFO(sensor_accel_fifo_s &sample);

	int get_instance() { return _sensor_pub.get_instance(); };

private:
    // Note: this value is borrowed from data_validator
    static const constexpr unsigned NORETURN_ERRCOUNT = 10000;
    /**< if the error count reaches this value, return sensor as invalid */

	void UpdateClipLimit();

    bool ParametersUpdate();
    void updateReference(const hrt_abstime &timestamp_sample);

    void validateAccel(sensor_accel_s &accel);

    bool attack_enabled(const uint8_t &attack_type, const hrt_abstime &timestamp_sample) const;

    void applyAccelAttack(sensor_accel_s &accel);
    void applyAccelAttack(sensor_accel_s &accel, sensor_accel_fifo_s &accel_fifo);

    float getMaxDeviation() const;

	uORB::PublicationMulti<sensor_accel_s> _sensor_pub{ORB_ID(sensor_accel)};
	uORB::PublicationMulti<sensor_accel_fifo_s>  _sensor_fifo_pub{ORB_ID(sensor_accel_fifo)};
    uORB::PublicationMulti<sensor_accel_errors_s> _sensor_accel_errors_pub{ORB_ID(sensor_accel_errors)};

    uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
    uORB::Subscription  _reference_accel_sub{ORB_ID(reference_accel)};

	uint32_t		_device_id{0};
	const enum Rotation	_rotation;

	int32_t			_imu_gyro_rate_max{0}; // match gyro max rate

	float			_range{16 * CONSTANTS_ONE_G};
	float			_scale{1.f};
	float			_temperature{NAN};

	float			_clip_limit{_range / _scale};

	uint32_t		_error_count{0};

	int16_t			_last_sample[3] {};

    AccelValidator::ParamStruct  _accel_validator_params{};
    AccelValidator               _accel_validator{&_accel_validator_params};
    sensor_accel_s               _curr_ref_accel{};
    sensor_accel_s               _next_ref_accel{};

    int  _attack_flag_prev{0};
    hrt_abstime _attack_timestamp{0};
    float   _last_deviation[3] {};

    DEFINE_PARAMETERS(
        (ParamInt<px4::params::ATK_APPLY_TYPE>) _param_atk_apply_type,
        (ParamInt<px4::params::ATK_STEALTH_TYPE>) _param_atk_stealth_type,
        (ParamInt<px4::params::ATK_COUNTDOWN_MS>) _param_atk_countdown_ms,
        (ParamInt<px4::params::ATK_MULTI_IMU>) _param_atk_multi_imu,
        (ParamFloat<px4::params::ATK_ACC_BIAS>) _param_atk_acc_bias,

        (ParamInt<px4::params::IV_DELAY_MASK>) _param_iv_delay_mask,
        (ParamInt<px4::params::IV_TTD_DELAY_MS>) _param_iv_ttd_delay_ms,
        (ParamFloat<px4::params::IV_ACC_NOISE>) _param_iv_acc_noise,
        (ParamFloat <px4::params::IV_ACC_CSUM_H>) _param_iv_acc_csum_h,
        (ParamFloat <px4::params::IV_ACC_MSHIFT>) _param_iv_acc_mshift,
        (ParamFloat<px4::params::IV_ACC_EMA_H>) _param_iv_acc_ema_h,
        (ParamFloat<px4::params::IV_ACC_ALPHA>) _param_iv_acc_alpha,
        (ParamFloat<px4::params::IV_ACC_EMA_CAP>) _param_iv_acc_ema_cap,
        (ParamExtFloat <px4::params::IV_ACC_TWIN_H>) _param_iv_acc_twin_h,
        (ParamExtInt <px4::params::IV_ACC_RST_CNT>) _param_iv_acc_rst_cnt,
        (ParamExtInt <px4::params::IV_ACC_CD_CNT>) _param_iv_acc_cd_cnt
    )
};
