#pragma once

#include "common.hpp"
#include "Detector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N = 1>
    class CuSumVector : public DetectorVector<Type, N> {
    public:
        using ParamStruct = CuSumParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        CuSumVector() = delete;

        CuSumVector(ParamStruct *params) : _param(params) { reset(); }

        ~CuSumVector() = default;

        void reset() {
            _error_sum.setAll(+Type(0.) * _param->control_limit);
            this->_error_mask = ERROR_FLAG_NO_ERROR;
            _is_running = false;
        }

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            this->_error_mask = ERROR_FLAG_NO_ERROR;
            if (_param->control_limit > Type(1.e-6)) {
                // validate the ratio
                _is_running = true;
                const Type bound = Type(1.05);
                _error_sum += matrix::constrain((innov_ratios.abs() - _param->mean_shift) / _param->control_limit, -bound, +bound);
                _error_sum = matrix::constrain(_error_sum, Type(0.), +bound);

                if (_error_sum.max() > +Type(1.0)) {
                    this->_error_mask |= ERROR_FLAG_SUM_UCL_EXCEED;
                }

                return (this->_error_mask == ERROR_FLAG_NO_ERROR);
            } else {
                // Do nothing, inhibit the check
                if (_is_running) { reset(); }
                return true;
            }
        }

        const VectorN &error_sum() const { return _error_sum; }

        const Type test_ratio() const {
            return test_ratios().max();
        }

        const VectorN test_ratios() const {
            return _error_sum;
        }

    private:
        ParamStruct *_param;
        VectorN _error_sum;
        bool _is_running{false};
    };

    using CuSumf = CuSumVector<float>;
    using CuSumVector2f = CuSumVector<float, 2>;
    using CuSumVector3f = CuSumVector<float, 3>;

} // fault_detector

