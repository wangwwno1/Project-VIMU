//
// Created by Robert Wang on 2022/7/25.
//

#pragma once

#include "common.hpp"
#include "Detector.hpp"
#include "CuSumVector.hpp"
#include "EMAVector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N>
    class CuSumVector;

    template<typename Type, size_t N>
    class EMAVector;

    template<typename Type, size_t N = 1>
    class EMACuSumVector : public DetectorVector<Type, N> {
    public:
        using typename DetectorVector<Type, N>::VectorN;
        using CuSumDetector = CuSumVector<Type, N>;
        using EMADetector = EMAVector<Type, N>;
        using ParamStruct = EMACuSumParams<Type>;

        EMACuSumVector() = delete;

        EMACuSumVector(ParamStruct *params) : EMACuSumVector(&params->cusum_params, &params->ema_params) {}

        EMACuSumVector(CuSumParams<Type> *csum_param, EMAParams<Type> *ema_param) :
                _cusum_detector(csum_param), _ema_detector(ema_param) {
            reset();
        }

        ~EMACuSumVector() = default;

        void reset() {
            _cusum_detector.reset();
            _ema_detector.reset();
        }

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            bool result = _cusum_detector.validate(innov_ratios);
            result = _ema_detector.validate(innov_ratios) && result;
            this->_error_mask = _cusum_detector.state() | _ema_detector.state();

            return result;
        }

        const Type test_ratio() const {
            return math::max(_cusum_detector.test_ratio(), _ema_detector.test_ratio());
        }

        const VectorN test_ratios() const {
            return matrix::max(_cusum_detector.test_ratios(), _ema_detector.test_ratios());
        }

        CuSumDetector &getCuSumDetector() { return _cusum_detector; }

        const CuSumDetector &getCuSumDetector() const { return _cusum_detector; }

        EMADetector &getEMADetector() { return _ema_detector; }

        const EMADetector &getEMADetector() const { return _ema_detector; }

        const VectorN &lower_sum() const { return _cusum_detector.lower_sum(); }

        const VectorN &upper_sum() const { return _cusum_detector.upper_sum(); }

        const VectorN &moving_average() { return _ema_detector.moving_average(); }

    private:
        CuSumDetector _cusum_detector;
        EMADetector _ema_detector;
    };

    using EMACuSumf = EMACuSumVector<float>;
    using EMACuSumVector2f = EMACuSumVector<float, 2>;
    using EMACuSumVector3f = EMACuSumVector<float, 3>;
} // fault_detector
