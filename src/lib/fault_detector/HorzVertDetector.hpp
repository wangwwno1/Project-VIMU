#pragma once

#include "common.hpp"
#include "Detector.hpp"
#include "CuSumVector.hpp"
#include "EMAVector.hpp"
#include "EMACuSumVector.hpp"

namespace fault_detector {

    template<typename HorzDetector, typename VertDetector>
    class HorzVertDetector {
    public:
        using HorzParam = typename HorzDetector::ParamStruct;
        using VertParam = typename VertDetector::ParamStruct;

        HorzVertDetector() = delete;

        HorzVertDetector(HorzParam *horz_params, VertParam *vert_params) :
                _horz_detector(horz_params), _vert_detector(vert_params) {
            _horz_detector.reset();
            _vert_detector.reset();
        }

        void reset() {
            _horz_detector.reset();
            _vert_detector.reset();
        }

        HorzDetector &getHorizontalDetector() { return _horz_detector; }

        const HorzDetector &getHorizontalDetector() const { return _horz_detector; }

        VertDetector &getVerticalDetector() { return _vert_detector; }

        const VertDetector &getVerticalDetector() const { return _vert_detector; }

        uint32_t state() const { return _horz_detector.state() | _vert_detector.state(); }

    private:
        HorzDetector _horz_detector;
        VertDetector _vert_detector;
    };

    using HorzVertCuSum = HorzVertDetector<CuSumVector2f, CuSumf>;
    using HorzVertEMA = HorzVertDetector<EMAVector2f, EMAf>;
    using HorzVertEMACuSum = HorzVertDetector<EMACuSumVector2f, EMACuSumf>;

} // fault_detector
