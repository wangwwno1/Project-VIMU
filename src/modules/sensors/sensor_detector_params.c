//
// Created by Robert Wang on 2022/9/1.
//

/**
 * Set true to record status log at high frequency, for debug only.
 *
 * @group Innovation Validator
 * @boolean
 */
PARAM_DEFINE_INT32(IV_DEBUG_LOG, 0);

/**
 * Set the delay that force the detector raise alarm after the attack is initiated, for debug only.
 *
 * Will override the default sensor detection mechanism if greater than zero.
 *
 * @group Innovation Validator
 */
PARAM_DEFINE_INT32(IV_TTD_DELAY_MS, 0);

/**
 * Sensor Injection Attack Control Mask.
 *
 * Set bits in the following positions to set which detector is applied with fixed TTD. Set to zero to disable.
 *
 * @group Innovation Validator
 * @min 0
 * @max 15
 * @bit 0 Gyroscope
 * @bit 1 Accelerometer
 * @bit 2 Gps position
 * @bit 3 Gps velocity
 */
PARAM_DEFINE_INT32(IV_DELAY_MASK, 0);

/**
 * Control limit for CuScore validation of the gps position innovation, expressed in standard deviations. Set zero to inhibit validation.
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
 * Control limit for CuScore validation of magnetometer innovation.
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
 * Minimum detectable mean shift for CuScore validation of magnetometer innovation.
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
 * Accelerometer noise for innovation validation, it is equal to the standard deviation of the residual between the virtual and real sensor.
 *
 * @group Innovation Validator
 * @min 0.01
 * @max 1.0
 * @unit m/s^2
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IV_ACC_NOISE, 3.5e-1f);

/**
 * Control limit for CuScore validation of the accelerometer delta velocity error.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero,
 * or variances if the ratio is set to one.
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the accelerometer delta velocity error.
 *
 * Expressed in standard deviations when the respective quadratic ratio is equal to zero.
 * or variances if the ratio is set to one.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_MSHIFT, 1.f);

/**
 * Rate gyro noise for innovation validation, it is equal to the standard deviation of the residual between virtual gyro and real gyro.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 0.2
 * @unit rad/s
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_NOISE, 0.1f);

/**
 * Control limit for CuScore validation of the gyro delta angle error, expressed in standard deviation.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_CSUM_H, 0.f);

/**
 * Minimum detectable mean shift for CuScore validation of the gyro delta angle error, expressed in standard deviation.
 *
 *
 * @group Innovation Validator
 * @min 0.0001
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_MSHIFT, 1.f);
