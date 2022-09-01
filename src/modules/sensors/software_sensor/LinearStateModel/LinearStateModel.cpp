//
// Created by Robert Wang on 2022/8/9.
//

#include "LinearStateModel.h"

namespace lsm {
    LinearStateModel::LinearStateModel() {
        _param_A.zero();
        _param_B.zero();
        _param_C.zero();
        _param_D.zero();
        reset_state();
    }

    LinearStateModel::LinearStateModel(const SquareMatrix3f &A, const SquareMatrix3f &B,
                                       const SquareMatrix3f &C, const SquareMatrix3f &D) :
            _param_A(A), _param_B(B), _param_C(C), _param_D(D) {
        reset_state();
    }

    void LinearStateModel::reset_state() {
        _state.zero();
        _output_state.zero();
        _target.zero();
    }

    void LinearStateModel::update() {
        _output_state = _param_C * _state + _param_D * _target;
        _state = _param_A * _state + _param_B * _target;
    }

    void LinearStateModel::update(const Vector3f &target) {
        setTargetState(target);
        update();
    }

    void LinearStateModel::setModelParam(const SquareMatrix3f &A, const SquareMatrix3f &B,
                                         const SquareMatrix3f &C, const SquareMatrix3f &D) {
        _param_A = A;
        _param_B = B;
        _param_C = C;
        _param_D = D;
    }

    void LinearStateModel::setState(const Vector3f &state) {
        setIfNotNan(_state, state);
    }

    void LinearStateModel::setTargetState(const Vector3f &target) {
        setIfNotNan(_target, target);
    }

    void LinearStateModel::setIfNotNan(Vector3f &internal_state, const Vector3f &setpoint) {
        for (uint8_t i = 0; i < 3; ++i) {
            if (PX4_ISFINITE(setpoint(i))) {
                // Replace internal state with new setpoint
                internal_state(i) = setpoint(i);
            }

            // setpoint is NAN, nothing to do
        }
    }

    Vector3f LinearStateModel::getState() const { return _state; }

    Vector3f LinearStateModel::getOutputState() const { return _output_state; }

    Vector3f LinearStateModel::getTargetState() const { return _target; }



} // lsm