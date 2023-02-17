#pragma once

#include "common.hpp"

namespace fault_detector {
    template<typename Type, size_t N = 1>
    class DetectorVector {
    public:
        using VectorN = matrix::Vector<Type, N>;

        virtual ~DetectorVector() = default;

        virtual void reset() = 0;

        virtual bool validate(const VectorN &innov_ratios) = 0;

        bool validate(const Type &innov_ratio) {
            VectorN innov_ratios;
            innov_ratios.setAll(innov_ratio);
            return validate(innov_ratios);
        };

        uint32_t state() const { return _error_mask; }

        virtual const Type test_ratio() const = 0;
        virtual const VectorN test_ratios() const = 0;

    protected:
        uint32_t _error_mask;
    };

    using Detectorf = DetectorVector<float>;
    using DetectorVector2f = DetectorVector<float, 2>;
    using DetectorVector3f = DetectorVector<float, 3>;
}
