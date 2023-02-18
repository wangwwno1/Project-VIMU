//
// Created by Robert Wang on 2022/7/22.
//

#pragma once

#include "common.hpp"
#include "Detector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N = 1>
    class EMAVector : public DetectorVector<Type, N> {
    public:
        using ParamStruct = EMAParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        EMAVector() = delete;

        EMAVector(ParamStruct *params) : _param(params) { reset(); }

        ~EMAVector() = default;

        void reset() { _state.setZero(); }

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            this->_error_mask = ERROR_FLAG_NO_ERROR;
            if (_param->control_limit > Type(1.e-6)) {
                // validate the ratio
                _is_running = true;

                const Type state_bound = Type(1.05) * _param->control_limit;

                VectorN val = innov_ratios;
                if (_param->cap > Type(1e-5)) {
                    const Type val_bound = _param->cap;
                    val = matrix::constrain(innov_ratios, -val_bound, +val_bound);
                }

                _state = (Type(1.) - _param->alpha) * _state + _param->alpha * val;
                _state = matrix::constrain(_state, -state_bound, +state_bound);

                if (_state.min() < -_param->control_limit) {
                    this->_error_mask |= ERROR_FLAG_SUM_LCL_EXCEED;
                }
                if (_state.max() > +_param->control_limit) {
                    this->_error_mask |= ERROR_FLAG_SUM_UCL_EXCEED;
                }

                return (this->_error_mask == ERROR_FLAG_NO_ERROR);
            } else {
                // Do nothing, inhibit the check
                if (_is_running) { reset(); }
                return true;
            }
        }

        const VectorN &moving_average() { return _state; }

        const Type test_ratio() const {
            if (_param->control_limit > Type(0.)) {
                return _state.abs().max() / _param->control_limit;
            }
            return Type(0.);
        }

        const VectorN test_ratios() const {
            VectorN test_ratios{};
            if (_param->control_limit > Type(0.)) {
                test_ratios =  _state.abs() / _param->control_limit;
            } else {
                test_ratios.zero();
            }

            return test_ratios;
        }

    private:
        ParamStruct *_param;
        VectorN _state;
        bool _is_running{false};
    };

    using EMAf = EMAVector<float>;
    using EMAVector2f = EMAVector<float, 2>;
    using EMAVector3f = EMAVector<float, 3>;

} // fault_detector
