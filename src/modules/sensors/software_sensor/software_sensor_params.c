//
// Created by Robert Wang on 2022/7/7.
//

/**
 * Enable Linear State Model Prediction
 *
 * @group Innovation Validator
 * @boolean
 */
PARAM_DEFINE_INT32(IV_ENABLE_LSM, 0);

/**
 * Force the usage of software sensor, use for debug only.
 *
 * Set bits in the following positions to set which software sensor is applied. Set to zero to disable.
 *
 * @group Innovation Validator
 * @min 0
 * @max 31
 * @bit 0 Gyroscope
 * @bit 1 Accelerometer (Not implemented)
 * @bit 2 Gps
 * @bit 3 Barometer
 * @bit 4 Magnetometer (Not implemented)
 */
PARAM_DEFINE_INT32(IV_SOFTSENS_MASK, 0);

/**
 * Time Window Length in ms for Linear State Model
 *
 * @min 1
 * @max 5000
 * @reboot_required true
 * @unit ms
 */
PARAM_DEFINE_INT32(IV_LSM_T_WINDOW, 575);
