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
    const float mcoef_corrrected = _params.mcoef * sqrtf(rho / CONSTANTS_AIR_DENSITY_SEA_LEVEL_15C);

    // get latest velocity in earth frame
    const float &vn = _output_new.vel(0);
    const float &ve = _output_new.vel(1);
    const float &vd = _output_new.vel(2);

    // get latest wind velocity in earth frame  // FIXME with wind_vel estimate
    const float &vwn = _wind_estimate(0);
    const float &vwe = _wind_estimate(1);
    const float &vwd = _wind_estimate(2);

    // predicted specific forces
    // calculate relative wind velocity in earth frame and rotate into body frame
    const Vector3f rel_wind_earth(vn - vwn, ve - vwe, vd - vwd);
    const Dcmf earth_to_body = quatToInverseRotMat(_output_new.quat_nominal);
    const Vector3f rel_wind_body = earth_to_body * rel_wind_earth;

    // Calculate the drag factor relative to rel_wind_body
    // and leave the sign to the last.
    Vector3f drag_coef{0.f, 0.f, 0.f};
    if (using_bcoef_x || using_bcoef_y) {
        const float total_airspeed = rel_wind_body.norm();
        if (using_bcoef_x) {
            const float bcoef_inv = 1.0f / _params.bcoef_x;
            drag_coef(0) += 0.5f * bcoef_inv * rho * total_airspeed;
        }
        if (using_bcoef_y) {
            const float bcoef_inv = 1.0f / _params.bcoef_y;
            drag_coef(1) += 0.5f * bcoef_inv * rho * total_airspeed;
        }
    }

    if (using_mcoef) {
        drag_coef += mcoef_corrrected;
    }

    // TODO Replace drag acceleration to force and torque, use extra parameter?
    _drag_acceleration = - rel_wind_body.emult(drag_coef);
    _drag_angular_acceleration.zero();      // TODO Angular Acceleration calculation
}