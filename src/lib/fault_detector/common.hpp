#pragma once

#include <matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

namespace fault_detector
{
    static constexpr uint32_t ERROR_FLAG_NO_ERROR = (0x00000000U);
    static constexpr uint32_t ERROR_FLAG_SUM_UCL_EXCEED = (0x00000001U);
    static constexpr uint32_t ERROR_FLAG_SUM_LCL_EXCEED = (0x00000001U << 1);
    static constexpr uint32_t ERROR_FLAG_VAL_UCL_EXCEED = (0x00000001U << 2);
    static constexpr uint32_t ERROR_FLAG_VAL_LCL_EXCEED = (0x00000001U << 3);

    template<typename Type>
    struct CuSumParams{
        Type control_limit     = Type(0.);  ///< The Control limit, expressed in standard deviations when quadratic_ratio = 0.
        Type mean_shift        = Type(1.);  ///< Minimum mean shift to detect, expressed in standard deviations when quadratic_ratio = 0.
    };

    template<typename Type>
    struct EMAParams {
        Type control_limit     = Type(0.);        ///< The Control limit, expressed in standard deviations.
        Type alpha             = Type(1.);        ///< Decay weight for the previous state. High value - more weight for recent state.
        Type cap               = Type(0.);        ///< Clip bound of deviation, expressed in standard deviations.
    };

    template<typename Type>
    struct EMACuSumParams {
        CuSumParams<Type> cusum_params{};
        EMAParams<Type>   ema_params{};
    };
}
