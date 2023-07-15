//
// Created by Robert Wang on 2022/9/1.
//

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
 * Control limit for Exponential Moving Average (EMA) validation of the gps position innovation, expressed in standard deviation.
 *
 * The formula is EMA(T) = min(max(val, -CAP), +CAP) * Alpha + (1-Alpha) * EMA(T-1)
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_EMA_H, 0.f);

/**
 * Alpha for Exponential Moving Average (EMA) validation of the gps position innovation.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 1.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_ALPHA, 1.f);

/**
 * Clip value for Exponential Moving Average (EMA) validation of the gps position innovation, expressed in standard deviation.
 *
 * This constrain the maximum value that will input into EMA detector.
 * Set zero to inhibit clip, for inhibit validation please refer to the control limit description.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_EMA_CAP, 0.f);

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
 * @reboot_required true
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
 * @reboot_required true
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
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_MSHIFT, 1.f);


/**
 * Control limit for Exponential Moving Average (EMA) validation of the acceleration error, expressed in standard deviation.
 *
 * The formula is EMA(T) = min(max(val, -CAP), +CAP) * Alpha + (1-Alpha) * EMA(T-1)
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_EMA_H, 0.f);

/**
 * Alpha for Exponential Moving Average (EMA) validation of the acceleration error, expressed in standard deviation.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 1.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_ALPHA, 1.f);

/**
 * Clip value for Exponential Moving Average (EMA) validation of the acceleration error, expressed in standard deviation.
 *
 * This constrain the maximum value that will input into EMA detector
 * Set zero to inhibit clip, for inhibit validation please refer to the control limit description.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IV_ACC_EMA_CAP, 0.f);

/**
 * Rate gyro noise for innovation validation, it is equal to the standard deviation of the residual between virtual gyro and real gyro.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 0.2
 * @unit rad/s
 * @reboot_required true
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

/**
 * Control limit for Exponential Moving Average (EMA) validation of the gyro delta angle error, expressed in standard deviation.
 *
 * The formula is EMA(T) = min(max(val, -CAP), +CAP) * Alpha + (1-Alpha) * EMA(T-1)
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_EMA_H, 0.f);

/**
 * Alpha for Exponential Moving Average (EMA) validation of the gyro delta angle error, expressed in standard deviation.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 1.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_ALPHA, 1.f);

/**
 * Clip value for Exponential Moving Average (EMA) validation of the gyro delta angle error, expressed in standard deviation.
 *
 * This constrain the maximum value that will input into EMA detector
 * Set zero to inhibit clip, for inhibit validation please refer to the control limit description.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IV_GYR_EMA_CAP, 0.f);
