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
            _error_sum.setAll(+Type(0.));
            _normal_error_cusum.setAll(+Type(0.));
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
                _is_running = true;
                if (!_is_normal && _safe_counter >= _param->safe_count) {
                    _is_normal = true;
                }

                if (_sample_counter >= _param->reset_samples) {
                    if (_update_offset && _is_normal && _normal_sample_counter >= _param->reset_samples) {
                        // Calculate average offset in last window
                        _error_offset = _normal_error_cusum / _sample_counter;
                        _normal_error_cusum.setAll(+Type(0.));
                        _normal_sample_counter = 0;
                    }
                    _error_sum.setAll(+Type(0.));
                    _sample_counter = 0;
                };

                // validate the ratio
                const VectorN corrected_error = innov_ratios - _error_offset;
                _error_sum += corrected_error.emult(corrected_error) / detect_threshold();
                _sample_counter++;

                if (test_ratio_raw() >= +Type(1.0)) {
                    // Declare faulty, discard all previous normal samples, reset safe counter
                    _normal_error_cusum.setAll(+Type(0.));
                    _normal_sample_counter = 0;
                    _safe_counter = 0;
                    _is_normal = false;
                } else {
                    if (!_is_normal) _safe_counter++;
                    // Only count innov_ratios offsets in normal
                    _normal_error_cusum += innov_ratios;
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

        // used for synchronize offset update between detectors
        bool update_offset() { return _update_offset; }
        void set_update_offset(bool update_offset) { _update_offset = update_offset; }

        const VectorN &error_sum() const { return _error_sum; }

        const Type error_offset() const { return _error_offset(0); }

        const VectorN error_offsets() const { return _error_offset; }

        void reset_error_offset() { _error_offset.setAll(+Type(0.)); }

        const Type detect_threshold() const {
            return _param->control_limit;
        }

        const Type test_ratio() const {
            return _is_normal ? test_ratio_raw() : fmaxf(test_ratio_raw(), Type(+1.0001));
        }

        const Type test_ratio_raw() const {
            return test_ratios().max();
        }

        const VectorN test_ratios() const {
            return _error_sum / static_cast<Type>(fmaxf(_sample_counter, 1));
        }

    protected:
        ParamStruct *_param;
        VectorN _error_sum{};
        VectorN _normal_error_cusum{};
        VectorN _error_offset{};
        int32_t _sample_counter{0};
        int32_t _normal_sample_counter{0};
        int32_t _safe_counter{0};
        bool _is_normal{true};
        bool _is_running{false};
        bool _update_offset{true};
    };

    using SquareErrorTimeWindowf = SquareErrorTimeWindowVector<float>;
    using SquareErrorTimeWindowVector2f = SquareErrorTimeWindowVector<float, 2>;
    using SquareErrorTimeWindowVector3f = SquareErrorTimeWindowVector<float, 3>;

} // fault_detector

