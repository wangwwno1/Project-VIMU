//
// Created by Robert Wang on 2022/8/31.
//

#include "VehicleAirData.hpp"

namespace sensors {
    void VehicleAirData::UpdateReferenceState() {
        if (_ref_baro_buffer == nullptr) {
            const uint8_t buffer_length = roundf(fmaxf(_param_ekf2_baro_delay.get() * 1.5f / (_param_ekf2_predict_us.get() * 1.e-3f), 2.f));
            RingBuffer<RefBaroSample> *inst = new RingBuffer<RefBaroSample> (buffer_length);
            if (inst && inst->valid()) {
                _ref_baro_buffer = inst;
                PX4_INFO("Ref Baro buffer allocated");
            } else {
                PX4_ERR("Ref Baro buffer allocation failed");
                return;
            }
        }

        if (!_reference_states_sub.advertised()) return;

        if (!_reference_states_sub.registered()) {
            _reference_states_sub.registerCallback();
        }

        estimator_states_s ref_states;
        estimator_offset_states_s offset_states;
        if (_reference_states_sub.update(&ref_states) && _vehicle_offset_states_sub.copy(&offset_states)) {
            RefBaroSample sample;
            sample.time_us = ref_states.timestamp_sample;
            sample.alt_meter = - ref_states.states[9];  // Convert downward relative position to altitude
            sample.alt_var = ref_states.covariances[9];
            sample.dt_ekf_avg = offset_states.dt_ekf_avg;
            sample.hgt_offset = offset_states.baro_hgt_offset;

            _ref_baro_buffer->push(sample);
        }

    }

    void VehicleAirData::ValidateBaroData(const uint8_t &instance, const hrt_abstime &timestamp_sample) {
        // First check if validator initiated
        if (_baro_validators[instance] == nullptr) {
            CuSumf *inst = new CuSumf (&_baro_hgt_params);
            if (inst) {
                _baro_validators[instance] = inst;
                _baro_validators[instance]->reset();
                PX4_INFO("Baro validator %" PRId8 " allocated", instance);
            } else {
                PX4_ERR("Baro validator %" PRId8 " allocation failed", instance);
                return;
            }
        }

        if (!_ref_baro_buffer || !_reference_states_sub.advertised()) return;

        // Find the real time of height state
        const float dt_ekf_avg = (_ref_baro_buffer->get_newest().time_us != 0) ?
                                  _ref_baro_buffer->get_newest().dt_ekf_avg :
                                 (_ref_baro_delayed.time_us != 0) ? _ref_baro_delayed.dt_ekf_avg : 0.f;
        hrt_abstime ref_timestamp = timestamp_sample;
        ref_timestamp -= static_cast<uint64_t>(_param_ekf2_baro_delay.get() * 1000);
        ref_timestamp -= static_cast<uint64_t>(dt_ekf_avg * 5e5f); // seconds to microseconds divided by 2

        const bool ref_baro_ready = _ref_baro_buffer->pop_first_older_than(ref_timestamp, &_ref_baro_delayed) ||
                                    ((_ref_baro_delayed.time_us != 0) && (ref_timestamp <= _ref_baro_delayed.time_us + 1_s));

        if (ref_baro_ready) {
            float pressure_pa = _data_sum[instance] / _data_sum_count[instance];
            const float temperature = _temperature_sum[instance] / _data_sum_count[instance];

            float altitude = PressureToAltitude(pressure_pa, temperature);

            const float hgt_offset = (_ref_baro_buffer->get_newest().time_us != 0) ?
                                     _ref_baro_buffer->get_newest().hgt_offset : _ref_baro_delayed.hgt_offset;
            const float ref_alt = _ref_baro_delayed.alt_meter + hgt_offset;
            const float variance = math::sq(fmaxf(_param_ekf2_baro_noise.get(), 0.01f)) + _ref_baro_delayed.alt_var;
            _baro_test_ratios[instance] = (ref_alt - altitude) / sqrt(variance);
            _baro_validators[instance]->validate(_baro_test_ratios[instance]);
        }
    }
}  // sensors