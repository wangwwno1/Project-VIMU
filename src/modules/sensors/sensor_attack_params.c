//
// Created by Robert Wang on 2022/7/6.
//

/**
 * Apply attack after the attack flag is enabled for this period of time.
 *
 * Use for coordinating attacks between modules.
 * NOTE: After the attack flag is enabled, DO NOT change other parameter for 2 seconds. Otherwise the countdown would be inaccurate.
 *
 * @group Sensor Attack
 * @min 5000
 * @max 60000
 */
PARAM_DEFINE_INT32(ATK_COUNTDOWN_MS, 5000);

/**
 * Sensor Injection Attack Control Mask.
 *
 * Set bits in the following positions to set which attack is applied. Set to zero to disable.
 *
 * @group Sensor Attack
 * @min 0
 * @max 128
 * @bit 0 Gyroscope spoofing (Apply to both Vehicle IMU and rate control)
 * @bit 1 Accelerometer spoofing
 * @bit 2 Gps position spoofing
 * @bit 3 Gps velocity spoofing
 * @bit 4 Barometer height jamming
 * @bit 5 Magnetometer jamming
 * @bit 6 GPS attack with both velocity and position
 */
PARAM_DEFINE_INT32(ATK_APPLY_TYPE, 0);

/**
 * Enable stealthy attack option.
 *
 * Set bits in the following positions to set which detector is countered. Set to zero to disable.
 * When enabled, override existing attack mechanism if related detector is also enabled.
 *
 * @group Sensor Attack
 * @min 0
 * @max 7
 * @bit 0 Cumulative Sum Detector
 * @bit 1 Exponential Moving Average (used by Gyroscope and GPS velocity)
 * @bit 2 Time-Windowed Detector
 */
PARAM_DEFINE_INT32(ATK_STEALTH_TYPE, 0);

/**
 * Sensor Injection Attack Control Mask for IMU.
 *
 * Set bits in the following positions to set which imu is affected. Set to zero to disable.
 *
 * @group Sensor Attack
 * @min 0
 * @max 15
 * @bit 0 Apply attack to IMU0
 * @bit 1 Apply attack to IMU1
 * @bit 2 Apply attack to IMU2
 * @bit 3 Apply attack to IMU3
 */
PARAM_DEFINE_INT32(ATK_MULTI_IMU, 0);

/**
 * Sensor Denied of Service Attack Control Mask for Magnetometer.
 *
 * Set bits in the following positions to set which mag is affected. Set to zero to disable.
 *
 * @group Sensor Attack
 * @min 0
 * @max 15
 * @bit 0 Apply attack to MAG0
 * @bit 1 Apply attack to MAG1
 * @bit 2 Apply attack to MAG2
 * @bit 3 Apply attack to MAG3
 */
PARAM_DEFINE_INT32(ATK_MULTI_MAG, 0);

/**
 * Sensor Denied of Service Attack Control Mask for Barometer.
 *
 * Set bits in the following positions to set which barometer is affected. Set to zero to disable.
 *
 * @group Sensor Attack
 * @min 0
 * @max 15
 * @bit 0 Apply attack to BARO0
 * @bit 1 Apply attack to BARO1
 * @bit 2 Apply attack to BARO2
 * @bit 3 Apply attack to BARO3
 */
PARAM_DEFINE_INT32(ATK_MULTI_BARO, 0);

// Parameters related to GPS Position Spoofing
/**
 * GPS Position Spoofing Attack class selector
 *
 * This parameter determine the type of attack. 0 - Bias, 1 - Linear, 2 - Exponential
 *
 * @group Sensors
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(ATK_GPS_P_CLS, 0);

/**
 * GPS Position Spoofing Deviation - Initial Values
 *
 * Set the initial value of deviation, based on the total deviation length.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.01
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_P_IV, 0.01f);

/**
 * GPS Position Spoofing Deviation - Rate of Rise
 *
 * Set the rise rate of deviation per second, for detail definition, see the implementation in sensor_attack.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.001
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_P_RATE, 1.00f);

/**
 * GPS Position Spoofing Deviation - Max Length
 *
 * Set the maximize length of deviation vector.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_P_CAP, 1.0f);


/**
 * GPS Position Spoofing Deviation - Heading
 *
 * Set the horizontal direction of deviation vector, use to convert deviation length to XY-axis.
 *
 *
 * @group Sensors
 * @unit deg
 * @min 0.0
 * @max 360.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ATK_GPS_P_HDG, 0.0f);

/**
 * GPS Position Spoofing Deviation - Pitch Angle
 *
 * Set the pitch angle of target deviation vector.
 *
 * @group Sensors
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ATK_GPS_P_PITCH, 0.0f);

// Parameters related to GPS Velocity Spoofing
/**
 * GPS Velocity Spoofing Attack class selector
 *
 * This parameter determine the type of attack. 0 - Bias, 1 - Linear, 2 - Exponential
 *
 * @group Sensors
 * @min 0
 * @max 2
 */
PARAM_DEFINE_INT32(ATK_GPS_V_CLS, 0);

/**
 * GPS Velocity Spoofing Deviation - Initial Values
 *
 * Set the initial value of deviation, based on the total deviation length.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.01
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_V_IV, 0.01f);

/**
 * GPS Velocity Spoofing Deviation - Rate of Rise
 *
 * Set the rise rate of deviation per second, for detail definition, see the implementation in sensor_attack.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.001
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_V_RATE, 1.00f);

/**
 * GPS Velocity Spoofing Deviation - Max Length
 *
 * Set the maximize length of deviation vector.
 *
 *
 * @group Sensors
 * @unit m
 * @min 0.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GPS_V_CAP, 1.0f);


/**
 * GPS Velocity Spoofing Deviation - Heading
 *
 * Set the horizontal direction of deviation vector, use to convert deviation length to XY-axis.
 *
 *
 * @group Sensors
 * @unit deg
 * @min 0.0
 * @max 360.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ATK_GPS_V_HDG, 0.0f);

/**
 * GPS Velocity Spoofing Deviation - Pitch Angle
 *
 * Set the pitch angle of target deviation vector.
 *
 * @group Sensors
 * @unit deg
 * @min -180.0
 * @max 180.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(ATK_GPS_V_PITCH, 0.0f);

/**
 * Gyroscope Spoofing Deviation - Roll Bias
 *
 * Set the bias of deviation at Roll Axis.
 *
 *
 * @group Sensors
 * @unit rad/s
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_GYR_BIAS, 0.00f);

/**
 * Accelerometer Spoofing Deviation - North Bias
 *
 * Set the bias of deviation at North Axis.
 *
 * @group Sensors
 * @unit m/s^2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(ATK_ACC_BIAS, 0.00f);

