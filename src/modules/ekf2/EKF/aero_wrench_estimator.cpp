//
// Created by Robert Wang on 2022/3/22.
//

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::updateAerodynamicWrench() {

    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),
                                  _state.vel(1) - _state.wind_vel(1),
                                  _state.vel(2));
    const Vector3f rel_wind_body = _state.quat_nominal.rotateVectorInverse(rel_wind_earth);
    const float rel_wind_speed = rel_wind_body.norm();

    if (!PX4_ISFINITE(rel_wind_speed)) {
        ECL_ERR("Encounter nan in rel_wind_speed.");
        return;
    }

    _rel_wind_body = rel_wind_body;

    updateDragForce(rel_wind_speed);
    updateDragTorque(rel_wind_speed);

}

void Ekf::updateDragForce(const float rel_wind_speed) {
    const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

    // correct rotor momentum drag for increase in required rotor mass flow with altitude
    // obtained from momentum disc theory
    const float mcoef_corrected = fmaxf(_params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C), 0.f);

    // drag model parameters
    const bool using_bcoef_x = _params.bcoef_x > 1.0f;
    const bool using_bcoef_y = _params.bcoef_y > 1.0f;
    const bool using_mcoef   = _params.mcoef   > 0.001f;

    if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
        return;
    }

    Vector2f bcoef_inv{0.f, 0.f};

    if (using_bcoef_x) {
        bcoef_inv(0) = 1.0f / _params.bcoef_x;
    }

    if (using_bcoef_y) {
        bcoef_inv(1) = 1.0f / _params.bcoef_y;
    }

    if (using_bcoef_x && using_bcoef_y) {
        // Interpolate between the X and Y bluff body drag coefficients using current relative velocity
        // This creates an elliptic drag distribution around the XY plane
        bcoef_inv(0) = Vector2f(bcoef_inv.emult(_rel_wind_body.xy()) / _rel_wind_body.xy().norm()).norm();
        bcoef_inv(1) = bcoef_inv(0);
    }

    // Calculate drag acceleration
    for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
        // Drag is modelled as an arbitrary combination of bluff body drag that proportional to
        // equivalent airspeed squared, and rotor momentum drag that is proportional to true airspeed
        // parallel to the rotor disc and mass flow through the rotor disc.
        float pred_acc = 0.0f; // predicted drag acceleration

        if (axis_index == 0) {
            if (using_mcoef && using_bcoef_x) {
                // Use a combination of bluff body and propeller momentum drag
                pred_acc = -0.5f * bcoef_inv(0) * rho * _rel_wind_body(0) * rel_wind_speed - _rel_wind_body(0) * mcoef_corrected;

            } else if (using_mcoef) {
                // Use propeller momentum drag only
                pred_acc = -_rel_wind_body(0) * mcoef_corrected;

            } else if (using_bcoef_x) {
                // Use bluff body drag only
                pred_acc = -0.5f * bcoef_inv(0) * rho * _rel_wind_body(0) * rel_wind_speed;

            } else {
                // skip this axis
            }

        } else if (axis_index == 1) {
            if (using_mcoef && using_bcoef_y) {
                // Use a combination of bluff body and propeller momentum drag
                pred_acc = -0.5f * bcoef_inv(1) * rho * _rel_wind_body(1) * rel_wind_speed - _rel_wind_body(1) * mcoef_corrected;

            } else if (using_mcoef) {
                // Use propeller momentum drag only
                pred_acc = -_rel_wind_body(1) * mcoef_corrected;

            } else if (using_bcoef_y) {
                // Use bluff body drag only
                pred_acc = -0.5f * bcoef_inv(1) * rho * _rel_wind_body(1) * rel_wind_speed;

            } else {
                // nothing more to do
            }
        } else if (axis_index == 2) {
            // Do not account z-axis drag for now - airflow in z-axis has more influence in the motor model
        }

        if (PX4_ISFINITE(pred_acc)) {
            _drag_acceleration(axis_index) = pred_acc;
        }
    }
}

void Ekf::updateDragTorque(const float rel_wind_speed) {
    const bool using_lcoef_roll     = fabsf(_params.lcoef_roll)     > 0.001f;
    const bool using_lcoef_pitch    = fabsf(_params.lcoef_pitch)    > 0.001f;
    const bool using_qcoef_roll     = fabsf(_params.qcoef_roll)     > 0.001f;
    const bool using_qcoef_pitch    = fabsf(_params.qcoef_pitch)    > 0.001f;

    if (!using_lcoef_roll && !using_lcoef_pitch && !using_qcoef_roll && !using_qcoef_pitch) {
        return;
    }

    // Calculate drag angular acceleration
    for (uint8_t axis_index = 0; axis_index <= 2; axis_index++) {
        float pred_acc = 0.0f;

        if (axis_index == 0) {
            if (using_lcoef_roll && using_qcoef_roll) {
                pred_acc = _params.qcoef_roll * _rel_wind_body(1) * rel_wind_speed + _rel_wind_body(1) * _params.lcoef_roll;

            } else if (using_lcoef_roll) {
                pred_acc = _rel_wind_body(1) * _params.lcoef_roll;

            } else if (using_qcoef_roll) {
                pred_acc = _params.qcoef_roll * _rel_wind_body(1) * rel_wind_speed;

            } else {
                // skip this axis
            }

        } else if (axis_index == 1) {
            if (using_lcoef_pitch && using_qcoef_pitch) {
                pred_acc = _params.qcoef_pitch * _rel_wind_body(0) * rel_wind_speed + _rel_wind_body(0) * _params.lcoef_roll;

            } else if (using_lcoef_pitch) {
                pred_acc = _rel_wind_body(0) * _params.lcoef_roll;

            } else if (using_qcoef_pitch) {
                pred_acc = _params.qcoef_pitch * _rel_wind_body(0) * rel_wind_speed;

            } else {
                // nothing more to do
            }
        } else if (axis_index == 2) {
            // Do not account z-axis drag torque for now
        }

        if (PX4_ISFINITE(pred_acc)) {
            _drag_angular_acceleration(axis_index) = pred_acc;
        }
    }
}
