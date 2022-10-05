#pragma once

#include "common.hpp"
#include "AbsErrorTimeWindowVector.hpp"

namespace fault_detector {
    template<typename Type, size_t N>
    class DetectorVector;

    template<typename Type, size_t N>
    class AbsErrorTimeWindowVector;

    template<typename Type, size_t N = 1>
    class SquareErrorTimeWindowVector : public AbsErrorTimeWindowVector<Type, N> {
    public:
        using ParentClass = AbsErrorTimeWindowVector<Type, N>;
        using ParamStruct = TimeWindowParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        SquareErrorTimeWindowVector() = delete;

        SquareErrorTimeWindowVector(ParamStruct *params) : ParentClass(params) { reset(); }

        ~SquareErrorTimeWindowVector() = default;

        using ParentClass::reset;

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            // squared_values = x * abs(x)
            const VectorN squared_values = innov_ratios.emult(innov_ratios.abs());
            return ParentClass::validate(squared_values);
        }

        using ParentClass::error_sum;
        using ParentClass::error_offset;
        using ParentClass::error_offsets;

        using ParentClass::reset_error_offset;
        using ParentClass::test_ratio;
        using ParentClass::test_ratios;
    };

    using SquareErrorTimeWindowVectorf = SquareErrorTimeWindowVector<float>;
    using SquareErrorTimeWindowVector2f = SquareErrorTimeWindowVector<float, 2>;
    using SquareErrorTimeWindowVector3f = SquareErrorTimeWindowVector<float, 3>;

} // fault_detector

