//
// Created by Robert Wang on 2022/7/7.
//

/**
 * The max delay induced by the AuxEKF Buffer
 *
 * This buffer can store gyroscope measurements up to the delay to prevent potential stealthy attacks.
 *
 * @group Vehicle Model
 * @min 1000
 * @max 1000000
 * @reboot_required true
 * @unit us
 */
PARAM_DEFINE_INT32(IV_IMU_DELAY_US, 500000);

/**
 * The integration interval of AuxEKF
 *
 * Accumulate torque samples to this interval before measurement update.
 *
 * @group Vehicle Model
 * @min 1000
 * @max 100000
 * @reboot_required true
 * @unit us
 */
PARAM_DEFINE_INT32(VIMU_PREDICT_US, 50000);

/**
* Boolean determining if gyroscope measurements should fused.
*
* A value of 1 indicates that fusion is active
*
* @group Vehicle Model
* @boolean
*/
PARAM_DEFINE_INT32(VIMU_FUSE_GYRO, 1);