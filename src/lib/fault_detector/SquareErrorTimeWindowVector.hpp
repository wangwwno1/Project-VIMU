#pragma once

#include "common.hpp"
#include "AbsErrorTimeWindowVector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N>
    class AbsErrorTimeWindowVector;

    template<typename Type, size_t N = 1>
    class SquareErrorTimeWindowVector : public DetectorVector<Type, N> {
    public:
        using ParamStruct = TimeWindowParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        SquareErrorTimeWindowVector() = delete;

        SquareErrorTimeWindowVector(ParamStruct *params) : _param(params) { reset(); }

        ~SquareErrorTimeWindowVector() = default;

        void reset() {
            _sum_square_error.setAll(+Type(0.) * _param->control_limit);
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
                    _sum_square_error.setAll(+Type(0.));
                    _sample_counter = 0;
                };

                // validate the ratio
                _is_running = true;
                const VectorN error = innov_ratios / _param->control_limit;
                const VectorN corrected_error = error - _error_offset;

                _sum_square_error += corrected_error.emult(corrected_error);
                _sum_square_error = matrix::constrain(_sum_square_error, Type(0.), +Type(1.05));
                _sample_counter++;

                if (_sum_square_error.max() >= +Type(1.0)) {
                    // Declare faulty, discard all previous normal samples, reset safe counter
                    _error_cusum.setAll(+Type(0.));
                    _normal_sample_counter = 0;
                    _safe_counter = 0;
                    _is_normal = false;
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

        const VectorN &error_sum() const { return _sum_square_error; }

        const Type error_offset() const { return _error_offset(0) * _param->control_limit; }

        const VectorN error_offsets() const { return _error_offset * _param->control_limit; }

        void reset_error_offset() { _error_offset.setAll(+Type(0.)); }

        const Type test_ratio() const {
            return _is_normal ? _sum_square_error.max() : math::max(_sum_square_error.max(), Type(+1.0001));
        }

        const VectorN test_ratios() const {
            return _sum_square_error;
        }

    private:
        ParamStruct *_param;
        VectorN _sum_square_error;
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

