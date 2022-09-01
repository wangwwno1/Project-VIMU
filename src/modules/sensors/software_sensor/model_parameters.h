//
// Created by Robert Wang on 2022/8/9.
//

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace lsm_params {
    using matrix::SquareMatrix3f;
    // Linear State Model
    // y(t)   = C * x(t) + D * u(t)
    // x(t+1) = A * x(t) + B * u(t)

    // Model Parameters for Position Linear State Model
    // TODO Fill the matrix with trained parameters.
    static const SquareMatrix3f POS_MODEL_A((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f POS_MODEL_B((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f POS_MODEL_C((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f POS_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    // Model Parameters for Velocity Linear State Model
    // TODO Fill the matrix with trained parameters.
    static const SquareMatrix3f VEL_MODEL_A((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f VEL_MODEL_B((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f VEL_MODEL_C((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f VEL_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    // Model Parameters for Attitude Linear State Model
    // TODO Fill the matrix with trained parameters.
    static const SquareMatrix3f ATT_MODEL_A((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f ATT_MODEL_B((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f ATT_MODEL_C((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f ATT_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });
    
    // Model Parameters for Rate Linear State Model
    // TODO Fill the matrix with trained parameters.
    static const SquareMatrix3f RATE_MODEL_A((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f RATE_MODEL_B((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f RATE_MODEL_C((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

    static const SquareMatrix3f RATE_MODEL_D((float[9]) {
            0., 0., 0.,
            0., 0., 0.,
            0., 0., 0.
    });

}