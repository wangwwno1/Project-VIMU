//
// Created by Robert Wang on 2022/8/9.
//

#pragma once

#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

namespace lsm {
    using matrix::Vector3f;
    using matrix::SquareMatrix3f;

    class LinearStateModel {
    public:
        LinearStateModel();;

        LinearStateModel(const SquareMatrix3f &A, const SquareMatrix3f &B,
                         const SquareMatrix3f &C, const SquareMatrix3f &D);

        ~LinearStateModel() = default;

        void reset_state();

        void update();

        void update(const Vector3f &target);

        Vector3f getState() const;

        Vector3f getOutputState() const;

        Vector3f getTargetState() const;

        void setState(const Vector3f &state);

        void setTargetState(const Vector3f &target);

        void setModelParam(const SquareMatrix3f &A, const SquareMatrix3f &B,
                           const SquareMatrix3f &C, const SquareMatrix3f &D);

    private:
        void setIfNotNan(Vector3f &internal_state, const Vector3f &setpoint);

        SquareMatrix3f _param_A;
        SquareMatrix3f _param_B;
        SquareMatrix3f _param_C;
        SquareMatrix3f _param_D;
        Vector3f _state;
        Vector3f _output_state;
        Vector3f _target;
    };

} // lsm


