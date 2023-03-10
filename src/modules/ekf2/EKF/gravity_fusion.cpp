/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name ECL nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file gravity_fusion.cpp
 * Fuse observations from the gravity vector to constrain roll
 * and pitch (a la complementary filter).
 *
 * @author Daniel M. Sahu <danielmohansahu@gmail.com>
 */

#include "ekf.h"
#include "python/ekf_derivation/generated/compute_gravity_innov_var_and_k_and_h.h"

#include <mathlib/mathlib.h>

void Ekf::controlGravityFusion()
{
    if (!_params.fuse_gravity || !_gravity_buffer) {
        return;
    }

    _control_status.flags.gravity_vector = false;
    _innov_check_fail_status.flags.reject_gravity = false;

    imuSample imu;
    if (_gravity_buffer->pop_first_older_than(_imu_sample_delayed.time_us, &imu)) {
        if (imu.delta_vel_dt < 1e-4f) {
            // Guard against short time sample
            return;
        }

        const float dt = imu.delta_vel_dt;
        const float dt_inv = 1.f / dt;

        const float alpha = math::constrain((dt / _params.acc_bias_learn_tc), 0.0f, 1.0f);
        const float beta = 1.0f - alpha;
        _grav_accel_vec_filt = alpha * dt_inv * imu.delta_vel + beta * _grav_accel_vec_filt;

        // fuse gravity observation if our overall acceleration isn't too big
        const float gravity_scale = _grav_accel_vec_filt.norm() / CONSTANTS_ONE_G;
//        _control_status.flags.gravity_vector = (((gravity_scale >= 0.9f && gravity_scale <= 1.1f)) || _control_status.flags.vehicle_at_rest)
//                                               && !isHorizontalAidingActive();
        // todo tighten the scale bound?
        _control_status.flags.gravity_vector = (((gravity_scale >= 0.9f && gravity_scale <= 1.1f)) || _control_status.flags.vehicle_at_rest);

    }

    if (_control_status.flags.gravity_vector) {
        // get raw accelerometer reading at delayed horizon and expected measurement noise (gaussian)
        const Vector3f accel_bias = _use_reference_imu ? _real_acc_bias : getAccelBias();
        const Vector3f measurement = imu.delta_vel / imu.delta_vel_dt - accel_bias;
        const float measurement_var = sq(_params.gravity_noise);

        // calculate kalman gains and innovation variances
        Vector24f Kx, Ky, Kz; // Kalman gain vectors
        sym::ComputeGravityInnovVarAndKAndH(
                getStateAtFusionHorizonAsVector(), P, measurement, measurement_var, FLT_EPSILON,
                &_grav_innov, &_grav_innov_var, &Kx, &Ky, &Kz);

        // NOTE: PX4 1.13.2 don't have aid src topic, so we work around it with variables
        float innovation_gate = 1.f;
        for (size_t i = 0; i < (sizeof(_grav_test_ratio) / sizeof(_grav_test_ratio(0))); i++) {
            if (PX4_ISFINITE(_grav_innov(i))
                && PX4_ISFINITE(_grav_innov_var(i))
                && (_grav_innov_var(i) > 0.f)
                    ) {
                _grav_test_ratio(i) = sq(_grav_innov(i)) / (sq(innovation_gate) * _grav_innov_var(i));

                if (_grav_test_ratio(i) > 1.f) {
                    _innov_check_fail_status.flags.reject_gravity = true;
                }

            } else {
                _grav_test_ratio(i) = INFINITY;
                _innov_check_fail_status.flags.reject_gravity = true;
            }
        }

        if (!_innov_check_fail_status.flags.reject_gravity) {
            // perform fusion for each axis
            if (measurementUpdate(Kx, _grav_innov_var(0), _grav_innov(0))) {
                if (measurementUpdate(Ky, _grav_innov_var(1), _grav_innov(1))) {
                    measurementUpdate(Kz, _grav_innov_var(2), _grav_innov(2));
                }
            }
        }
    }

}