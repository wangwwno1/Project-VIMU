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
        using ParamStruct = TimeWindowParams<Type>;
        using typename DetectorVector<Type, N>::VectorN;


        SquareErrorTimeWindowVector() = delete;

        SquareErrorTimeWindowVector(ParamStruct *params) : _param(params) { reset(); }

        ~SquareErrorTimeWindowVector() = default;

        using AbsErrorTimeWindowVector<Type, N>::reset;

        using DetectorVector<Type, N>::validate;

        bool validate(const VectorN &innov_ratios) {
            // squared_values = x * abs(x)
            const VectorN squared_values = innov_ratios.emult(innov_ratios.abs());
            return AbsErrorTimeWindowVector<Type, N>::validate(squared_values);
        }

        using AbsErrorTimeWindowVector<Type, N>::error_sum;
        using AbsErrorTimeWindowVector<Type, N>::error_offset;
        using AbsErrorTimeWindowVector<Type, N>::error_offsets;

        using AbsErrorTimeWindowVector<Type, N>::reset_error_offset;
        using AbsErrorTimeWindowVector<Type, N>::test_ratio;
        using AbsErrorTimeWindowVector<Type, N>::test_ratios;
    };

    using SquareErrorTimeWindowVectorf = SquareErrorTimeWindowVector<float>;
    using SquareErrorTimeWindowVector2f = SquareErrorTimeWindowVector<float, 2>;
    using SquareErrorTimeWindowVector3f = SquareErrorTimeWindowVector<float, 3>;

} // fault_detector

