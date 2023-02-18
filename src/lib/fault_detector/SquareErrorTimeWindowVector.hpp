#pragma once

#include "common.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N = 1>
    class SquareErrorTimeWindowVector : public DetectorVector<Type, N> {
    public:
        using ParamStruct = TimeWindowParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        SquareErrorTimeWindowVector() = delete;

        SquareErrorTimeWindowVector(ParamStruct *params) : _param(params) { reset(); }

        ~SquareErrorTimeWindowVector() = default;

        void reset() {
            _sum_squared_error.setAll(+Type(0.));
            reset_error_offset();

            _sample_counter = 0;
            _normal_sample_counter = 0;
            _safe_counter = 0;

            _is_normal = true;
            _is_running = false;
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
                    }
                    _sum_squared_error.setAll(+Type(0.));
                    _sample_counter = 0;
                };

                // validate the ratio
                _is_running = true;
                const VectorN relative_error = innov_ratios / sqrt(detect_threshold());
                const VectorN corrected_error = relative_error - _error_offset;

                _sum_squared_error += corrected_error.emult(corrected_error);
                _sample_counter++;

                if (test_ratio() >= +Type(1.0)) {
                    // Declare faulty, discard all previous normal samples, reset safe counter
                    _error_cusum.setAll(+Type(0.));
                    _normal_sample_counter = 0;
                    _safe_counter = 0;
                    _is_normal = false;
                } else {
                    if (!_is_normal) _safe_counter++;
                    // Only count relative_error offsets in normal
                    _error_cusum += relative_error;
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

        const VectorN &error_sum() const { return _sum_squared_error; }

        const Type error_offset() const { return _error_offset(0) * sqrt(detect_threshold()); }

        const VectorN error_offsets() const { return _error_offset * sqrt(detect_threshold()); }

        void reset_error_offset() { _error_offset.setAll(+Type(0.)); }

        const Type detect_threshold() const {
            return _param->control_limit / static_cast<Type>(math::max(_param->reset_samples, 1));
        }

        const Type test_ratio() const {
            const Type test_ratio = test_ratios().max();
            return _is_normal ? test_ratio : math::max(test_ratio, Type(+1.0001));
        }

        const VectorN test_ratios() const {
            return _sum_squared_error / static_cast<Type>(math::max(_sample_counter, 1));
        }

    private:
        ParamStruct *_param;
        VectorN _sum_squared_error;
        VectorN _error_cusum;
        VectorN _error_offset;
        int32_t _sample_counter{0};
        int32_t _normal_sample_counter{0};
        int32_t _safe_counter{0};
        bool _is_normal{true};
        bool _is_running{false};
    };

    using SquareErrorTimeWindowVectorf = SquareErrorTimeWindowVector<float>;
    using SquareErrorTimeWindowVector2f = SquareErrorTimeWindowVector<float, 2>;
    using SquareErrorTimeWindowVector3f = SquareErrorTimeWindowVector<float, 3>;

} // fault_detector

