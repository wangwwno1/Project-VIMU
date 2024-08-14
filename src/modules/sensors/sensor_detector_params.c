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
 * Control limit for Time Window (TWIN) validation of the gps position innovation, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_P_TWIN_H, 0.f);


/**
 * Minimum samples required before reset gps position TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GPS_P_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GPS_P_CD_CNT, 1);

/**
 * Control limit for Time Window (TWIN) validation of the gps velocity innovation, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_V_TWIN_H, 0.f);

/**
 * Minimum samples required before reset gps velocity TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GPS_V_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GPS_V_CD_CNT, 1);

/**
 * Control limit for Time Window (TWIN) validation of the barometer rate innovation, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_BARO_TWIN_H, 0.f);

/**
 * Minimum samples required before reset barometer TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_BARO_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_BARO_CD_CNT, 1);

/**
 * Control limit for Time Window (TWIN) validation of the magnetometer rate innovation, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_MAG_TWIN_H, 0.f);

/**
 * Minimum samples required before reset magnetometer TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_MAG_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_MAG_CD_CNT, 1);

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
 * Control limit for Time Window (TWIN) validation for acceleration, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_ACC_TWIN_H, 0.f);

/**
 * Minimum samples required before reset accelerometer TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_ACC_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_ACC_CD_CNT, 1);

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
 * Control limit for Time Window (TWIN) validation of the gyro rate innovation, expressed in squared of standard deviations.
 *
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @reboot_required true
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GYR_TWIN_H, 0.f);

/**
 * Minimum samples required before reset gyro TWIN detector.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GYR_RST_CNT, 1);

/**
 * Minimum consecutive normal samples before declare the sensor is normal.
 *
 * @group Innovation Validator
 * @min 1
 * @reboot_required true
 */
PARAM_DEFINE_INT32(IV_GYR_CD_CNT, 1);
