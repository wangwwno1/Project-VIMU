/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * 3. Neither the name PX4 nor the names of its contributors may be
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
 * Enable enhanced innovation validator for EKF.
 *
 * The innovation validator is always enable for VIMU-EKF, set this flag true to enable it for other EKF instances.
 *
 * @boolean
 * @group Innovation Validator
 */
PARAM_DEFINE_INT32(IV_APPLY_TO_EKF, 1);

/**
 * Control limit for CuScore validation of the gps horizontal position innovation, expressed in standard deviations. Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the gps position innovation, expressed in standard deviations.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the gps velocity innovation, expressed in standard deviations, set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the gps velocity innovation, expressed in standard deviations.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the ev horizontal position innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_EV_P_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the ev horizontal position innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_EV_P_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the ev horizontal velocity innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_EV_V_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the ev horizontal velocity innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_EV_V_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the barometer height innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_BARO_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the barometer height innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_BARO_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the range finder height innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_RNG_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the range finder height innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_RNG_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the heading innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_HDG_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the heading innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_HDG_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the 3D magnetometer innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_MAG_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the 3D magnetometer innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_MAG_MSHIFT, 1.f);

/**
 * Control limit for CuScore validation of the auxiliary horizontal velocity innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_AUX_V_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the auxiliary horizontal velocity innovation.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero, or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_AUX_V_MSHIFT, 1.f);
