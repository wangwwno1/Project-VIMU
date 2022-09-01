//
// Created by Robert Wang on 2022/9/1.
//


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
 * Control limit for Exponential Moving Average (EMA) validation of the gps velocity innovation, expressed in standard deviation.
 *
 * The formula is EMA(T) = min(max(val, -CAP), +CAP) * Alpha + (1-Alpha) * EMA(T-1)
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_EMA_H, 0.f);

/**
 * Alpha for Exponential Moving Average (EMA) validation of the gps velocity innovation.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 1.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_ALPHA, 1.f);

/**
 * Clip value for Exponential Moving Average (EMA) validation of the gps velocity innovation, expressed in standard deviation.
 *
 * This constrain the maximum value that will input into EMA detector.
 * Set zero to inhibit clip, for inhibit validation please refer to the control limit description.
 *
 * @group Innovation Validator
 * @min 0.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_EMA_CAP, 0.f);

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
