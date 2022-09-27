#pragma once

#include "common.hpp"
#include "Detector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N = 1>
    class AbsErrorTimeWindowVector : public DetectorVector<Type, N> {
    public:
        using ParamStruct = TimeWindowParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        AbsErrorTimeWindowVector() = delete;

        AbsErrorTimeWindowVector(ParamStruct *params) : _param(params) { reset(); }

        ~AbsErrorTimeWindowVector() = default;

        void reset() {
            _abs_error_cusum.setAll(+Type(0.) * _param->control_limit);
            reset_error_offset();

            _sample_counter = 0;
            _normal_sample_counter = 0;
            _safe_counter = 0;

            _is_normal = true;
            _is_running = false;
            _offset_ready = false;
            this->_error_mask = ERROR_FLAG_NO_ERROR;
        }

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            if (_param->control_limit > Type(1.e-6)) {
                if (!_is_normal && _safe_counter >= _param->safe_count) {
                    _is_normal = true;
                }

                // The original time window detector in SoftwareSensor requires _is_normal to reset time window.
                // if (_is_normal && (_sample_counter >= _param->reset_samples)) {
                // However, since the sum is always monotonically increasing,
                // this would create a deadlock after attack is detected (i.e. upper sum > control limit).
                // Therefore, we modify the original condition and use a larger safe window.
                if (_sample_counter >= _param->reset_samples) {
                    if (_is_normal && _normal_sample_counter >= _param->reset_samples) {
                        // Calculate average offset in last window
                        _error_offset = _error_cusum / _sample_counter;
                        _error_cusum.setAll(+Type(0.));
                        _normal_sample_counter = 0;
                        _offset_ready = true;
                    }
                    _abs_error_cusum.setAll(+Type(0.));
                    _sample_counter = 0;
                };

                // validate the ratio
                _is_running = true;
                const VectorN error = innov_ratios / _param->control_limit;

                _abs_error_cusum += (error - _error_offset).abs();
                _abs_error_cusum = matrix::constrain(_abs_error_cusum, Type(0.), +Type(1.05));
                _sample_counter++;

                if (_abs_error_cusum.max() >= +Type(1.0)) {
                    // Declare faulty, discard all previous normal samples, reset safe counter
                    _error_cusum.setAll(+Type(0.));
                    _normal_sample_counter = 0;
                    _safe_counter = 0;
                    _is_normal = false;
                    _offset_ready = false;
                } else {
                    if (!_is_normal) _safe_counter++;
                    // Only count error offsets in normal
                    _error_cusum += error;
                    _normal_sample_counter++;
                }

                this->_error_mask = (_is_normal) ? ERROR_FLAG_SUM_UCL_EXCEED : ERROR_FLAG_NO_ERROR;

                return _is_normal;
            } else {
                // Do nothing, inhibit the check
                if (_is_running) { reset(); }
                return true;
            }
        }

        const VectorN &error_sum() const { return _abs_error_cusum; }

        const Type error_offset() const { return _error_offset(0) * _param->control_limit; }

        const VectorN error_offsets() const { return _error_offset * _param->control_limit; }

        void reset_error_offset() { _error_offset.setAll(+Type(0.)); }

        const Type test_ratio() const {
            return _abs_error_cusum.max();
        }

        const VectorN test_ratios() const {
            return _abs_error_cusum;
        }

    private:
        ParamStruct *_param;
        VectorN _abs_error_cusum;
        VectorN _error_cusum;
        VectorN _error_offset;
        int32_t _sample_counter{0};
        int32_t _normal_sample_counter{0};
        int32_t _safe_counter{0};
        bool _is_normal{true};
        bool _is_running{false};
        bool _offset_ready{false};
    };

    using AbsErrorTimeWindowf = AbsErrorTimeWindowVector<float>;
    using AbsErrorTimeWindowVector2f = AbsErrorTimeWindowVector<float, 2>;
    using AbsErrorTimeWindowVector3f = AbsErrorTimeWindowVector<float, 3>;

} // fault_detector

