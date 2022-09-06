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
            _lower_sum.setAll(-Type(0.) * _param->control_limit);
            _upper_sum.setAll(+Type(0.) * _param->control_limit);
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
                _lower_sum += matrix::constrain((innov_ratios + _param->mean_shift) / _param->control_limit, -bound, +bound);
                _lower_sum = matrix::constrain(_lower_sum, -bound, Type(0.));
                _upper_sum += matrix::constrain((innov_ratios - _param->mean_shift) / _param->control_limit, -bound, +bound);
                _upper_sum = matrix::constrain(_upper_sum, Type(0.), +bound);

                if (_lower_sum.min() < -Type(1.0)) {
                    this->_error_mask |= ERROR_FLAG_SUM_LCL_EXCEED;
                }
                if (_upper_sum.max() > +Type(1.0)) {
                    this->_error_mask |= ERROR_FLAG_SUM_UCL_EXCEED;
                }

                return (this->_error_mask == ERROR_FLAG_NO_ERROR);
            } else {
                // Do nothing, inhibit the check
                if (_is_running) { reset(); }
                return true;
            }
        }

        const VectorN &lower_sum() const { return _lower_sum; }

        const VectorN &upper_sum() const { return _upper_sum; }

        const Type test_ratio() const {
            return math::max(_lower_sum.abs().max(), _upper_sum.max());
        }

        const VectorN test_ratios() const {
            return matrix::max(_lower_sum.abs(), _upper_sum);
        }

    private:
        ParamStruct *_param;
        VectorN _lower_sum;
        VectorN _upper_sum;
        bool _is_running{false};
    };

    using CuSumf = CuSumVector<float>;
    using CuSumVector2f = CuSumVector<float, 2>;
    using CuSumVector3f = CuSumVector<float, 3>;

} // fault_detector

