//
// Created by Robert Wang on 2022/8/31.
//

#include "VehicleAirData.hpp"

namespace sensors {
    void VehicleAirData::UpdateReferenceState() {
        if (_ref_baro_buffer[0] == nullptr) {
            // Wait until first buffer initiated
            return;
        }

        if (!_vehicle_local_position_sub.registered()) {
            _vehicle_local_position_sub.registerCallback();
        }

        // find corresponding estimated sensor bias
        if (_estimator_selector_status_sub.updated()) {
            estimator_selector_status_s estimator_selector_status;

            if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
                _estimator_offset_states_sub.ChangeInstance(estimator_selector_status.primary_instance);
            }
        }

        vehicle_local_position_s local_pos{};
        estimator_offset_states_s offset_states{};
        if (_vehicle_local_position_sub.update(&local_pos) && _estimator_offset_states_sub.copy(&offset_states)) {
            RefBaroSample sample{};
            sample.time_us = local_pos.timestamp_sample;
            sample.alt_meter = -local_pos.z;  // Convert downward relative position to altitude
            sample.dt_ekf_avg = offset_states.dt_ekf_avg;
            sample.hgt_offset = offset_states.baro_hgt_offset;

            for (auto &buffer : _ref_baro_buffer) {
                if (buffer) {
                    buffer->push(sample);
                }
            }
        }

    }

    void VehicleAirData::ValidateBaroData(const uint8_t &instance, const hrt_abstime &timestamp_sample) {
        // First check if validator initiated
        if (_baro_validators[instance] == nullptr) {
            BaroValidator *inst = new BaroValidator (&_baro_hgt_params);
            if (inst) {
                _baro_validators[instance] = inst;
                _baro_validators[instance]->reset();
                PX4_INFO("Baro validator %" PRId8 " allocated", instance);
            } else {
                PX4_ERR("Baro validator %" PRId8 " allocation failed", instance);
                return;
            }
        }

        if (_ref_baro_buffer[instance] == nullptr) {
            const uint8_t buffer_length = roundf(fmaxf(_param_ekf2_baro_delay.get() * 1.5f / (_param_ekf2_predict_us.get() * 1.e-3f), 1.f));
            // +1 because we need to take the first sample newer than the gps
            RingBuffer<RefBaroSample> *inst = new RingBuffer<RefBaroSample> (buffer_length + 1);
            if (inst && inst->valid()) {
                _ref_baro_buffer[instance] = inst;
                PX4_INFO("Ref Baro buffer %d allocated", instance);
            } else {
                PX4_ERR("Ref Baro buffer %d allocation failed", instance);
                return;
            }
        }

        // Find the real time of height state
        RingBuffer<RefBaroSample> *pBuffer = _ref_baro_buffer[instance];
        const float dt_ekf_avg = (pBuffer->get_newest().time_us != 0) ? pBuffer->get_newest().dt_ekf_avg : (_param_ekf2_predict_us.get() * 1.e-6f);
        const hrt_abstime time_delay = static_cast<uint64_t>(_param_ekf2_baro_delay.get() * 1000) + static_cast<uint64_t>(dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2
        if (timestamp_sample <= time_delay) {
            // Avoid overflow
            return;
        }

        const hrt_abstime ref_timestamp = timestamp_sample - time_delay;
        // In EKF we take the first baro sample older than delayed imu sample
        // _baro_data_ready = _baro_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &_baro_sample_delayed);
        // So at there we take the first reference barometer NEWER than the measurement sample
        pBuffer->pop_first_older_than(ref_timestamp, &_ref_baro_delayed);
        if (pBuffer->get_oldest().time_us != 0) {
            _ref_baro_delayed = pBuffer->get_oldest();
        }

        if (_ref_baro_delayed.time_us != 0) {
            float pressure_pa = _data_sum[instance] / _data_sum_count[instance];
            const float temperature = _temperature_sum[instance] / _data_sum_count[instance];

            float altitude = PressureToAltitude(pressure_pa, temperature);

            const float hgt_offset = (pBuffer->get_newest().time_us != 0) ?
                                     pBuffer->get_newest().hgt_offset : _ref_baro_delayed.hgt_offset;
            const float hgt_error = _ref_baro_delayed.alt_meter + hgt_offset - altitude;
            const float variance = math::sq(fmaxf(_param_ekf2_baro_noise.get(), 0.01f));
            _baro_validators[instance]->validate(hgt_error / sqrt(variance));

            _baro_error_status.timestamp_reference[instance] = _ref_baro_delayed.time_us;
            _baro_error_status.error[instance] = hgt_error;
            _baro_error_status.variance[instance] = variance;
            _baro_error_status.test_ratio[instance] = _baro_validators[instance]->test_ratio();
            _status_updated = true;
        }
    }
}  // sensors
