//
// Created by Robert Wang on 2022/3/22.
//

#include "ekf.h"
#include <mathlib/mathlib.h>

void Ekf::updateAerodynamicWrench() {
    // drag model parameters
    const bool using_bcoef_x = _params.bcoef_x > 1.0f;
    const bool using_bcoef_y = _params.bcoef_y > 1.0f;
    const bool using_mcoef   = _params.mcoef   > 0.001f;

    if (!using_bcoef_x && !using_bcoef_y && !using_mcoef) {
        return;
    }

    const float rho = fmaxf(_air_density, 0.1f); // air density (kg/m**3)

    // correct rotor momentum drag for increase in required rotor mass flow with altitude
    // obtained from momentum disc theory
    const float mcoef_corrrected = fmaxf(_params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C), 0.f);

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3f rel_wind_earth(_state.vel(0) - _state.wind_vel(0),
                                  _state.vel(1) - _state.wind_vel(1),
                                  _state.vel(2));
    const Vector3f rel_wind_body = _state.quat_nominal.rotateVectorInverse(rel_wind_earth);
    const float rel_wind_speed = rel_wind_body.norm();

    Vector2f bcoef_inv;

    if (using_bcoef_x) {
        bcoef_inv(0) = 1.0f / _params.bcoef_x;
    }

    if (using_bcoef_y) {
        bcoef_inv(1) = 1.0f / _params.bcoef_y;
    }

    if (using_bcoef_x && using_bcoef_y) {

        // Interpolate between the X and Y bluff body drag coefficients using current relative velocity
        // This creates an elliptic drag distribution around the XY plane
        bcoef_inv(0) = Vector2f(bcoef_inv.emult(rel_wind_body.xy()) / rel_wind_body.xy().norm()).norm();
        bcoef_inv(1) = bcoef_inv(0);
    }

    // Calculate the drag factor relative to rel_wind_body
    // and leave the sign to the last.
    Vector3f drag_coef{0.f, 0.f, 0.f};
    if (using_bcoef_x || using_bcoef_y) {
        if (using_bcoef_x) {
            drag_coef(0) += 0.5f * bcoef_inv(0) * rho * rel_wind_speed;
        }
        if (using_bcoef_y) {
            drag_coef(1) += 0.5f * bcoef_inv(1) * rho * rel_wind_speed;
        }
    }

    if (using_mcoef) {
        drag_coef += mcoef_corrrected;
    }

    _drag_acceleration = - rel_wind_body.emult(drag_coef);
    _drag_angular_acceleration.zero();      // TODO Angular Acceleration calculation
}