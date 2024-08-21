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


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>
#include <lib/parameters/param.h>

using namespace time_literals;

static constexpr int32_t sum(const int16_t samples[], uint8_t len)
{
	int32_t sum = 0;

	for (int n = 0; n < len; n++) {
		sum += samples[n];
	}

	return sum;
}

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, enum Rotation rotation) :
    ModuleParams(nullptr),
	_device_id{device_id},
	_rotation{rotation},
    _param_iv_gyr_csum_h(_gyro_validator_params.cusum_params.control_limit),
    _param_iv_gyr_mshift(_gyro_validator_params.cusum_params.mean_shift),
    _param_iv_gyr_ema_h(_gyro_validator_params.ema_params.control_limit),
    _param_iv_gyr_alpha(_gyro_validator_params.ema_params.alpha),
    _param_iv_gyr_ema_cap(_gyro_validator_params.ema_params.cap)
{
	// advertise immediately to keep instance numbering in sync
	_sensor_pub.advertise();
    _sensor_gyro_errors_pub.advertise();

	param_get(param_find("IMU_GYRO_RATEMAX"), &_imu_gyro_rate_max);
}

PX4Gyroscope::~PX4Gyroscope()
{
	_sensor_pub.unadvertise();
	_sensor_fifo_pub.unadvertise();
    _sensor_gyro_errors_pub.unadvertise();
}

void PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back
	_device_id = device_id.devid;
}

void PX4Gyroscope::set_scale(float scale)
{
	if (fabsf(scale - _scale) > FLT_EPSILON) {
		// rescale last sample on scale change
		float rescale = _scale / scale;

		for (auto &s : _last_sample) {
			s = roundf(s * rescale);
		}

		_scale = scale;
	}
}

void PX4Gyroscope::update(const hrt_abstime &timestamp_sample, float x, float y, float z)
{
	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	sensor_gyro_s report;

	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;
	report.x = x * _scale;
	report.y = y * _scale;
	report.z = z * _scale;
	report.samples = 1;

    // Apply simulated attack, check gyro data, replace error_count to alarm value if an attack is detected.
    ParametersUpdate();
    updateReference(timestamp_sample);
    applyGyroAttack(report);
    validateGyro(report);

    report.timestamp = hrt_absolute_time();
    _sensor_pub.publish(report);
}

void PX4Gyroscope::updateFIFO(sensor_gyro_fifo_s &sample)
{
	// rotate all raw samples and publish fifo
	const uint8_t N = sample.samples;

	for (int n = 0; n < N; n++) {
		rotate_3i(_rotation, sample.x[n], sample.y[n], sample.z[n]);
	}

	// publish
	sensor_gyro_s report;
	report.timestamp_sample = sample.timestamp_sample;
	report.device_id = _device_id;
	report.temperature = _temperature;
	report.error_count = _error_count;

	// trapezoidal integration (equally spaced)
	const float scale = _scale / (float)N;
	report.x = (0.5f * (_last_sample[0] + sample.x[N - 1]) + sum(sample.x, N - 1)) * scale;
	report.y = (0.5f * (_last_sample[1] + sample.y[N - 1]) + sum(sample.y, N - 1)) * scale;
	report.z = (0.5f * (_last_sample[2] + sample.z[N - 1]) + sum(sample.z, N - 1)) * scale;

    _last_sample[0] = sample.x[N - 1];
	_last_sample[1] = sample.y[N - 1];
	_last_sample[2] = sample.z[N - 1];

    // Try to update reference
    ParametersUpdate();
    updateReference(sample.timestamp_sample);
    applyGyroAttack(report, sample);
    validateGyro(report);

    report.samples = N;
	report.timestamp = hrt_absolute_time();
	_sensor_pub.publish(report);

    // Postpone fifo publish until detection completed
    sample.device_id = _device_id;
    sample.scale = _scale;
    sample.timestamp = hrt_absolute_time();
    _sensor_fifo_pub.publish(sample);
}

bool PX4Gyroscope::ParametersUpdate()
{
    bool updated = false;

    // Check if parameters have changed
    if (_parameter_update_sub.updated()) {
        // clear update
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);

        updateParams();

        if (_param_atk_apply_type.get() != _attack_flag_prev) {
            const int next_attack_flag = _param_atk_apply_type.get();
            if (next_attack_flag & sensor_attack::ATK_MASK_GYRO
                && _param_atk_multi_imu.get() & (1 << get_instance())) {
                // Enable attack, calculate new timestamp
                _attack_timestamp = param_update.timestamp + (hrt_abstime) (_param_atk_countdown_ms.get() * 1000);
                PX4_INFO("Debug - Enable GYRO attack for instance %d, expect start timestamp: %" PRIu64,
                         get_instance(), _attack_timestamp);

            } else if (_attack_timestamp != 0) {
                // Disable attack, reset timestamp and attack_has_start flag
                _attack_timestamp = 0;
                _attack_has_start = 0;
                PX4_INFO("Debug - Attack is disabled for GYRO%d, reset attack timestamp.", get_instance());
            }

            _attack_flag_prev = next_attack_flag;
        }

        updated = true;
    }

    return updated;
}