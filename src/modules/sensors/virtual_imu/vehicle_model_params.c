/****************************************************************************
 *
 *   Copyright (c) 2015-2016 Estimation and Control Library (ECL). All rights reserved.
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
 * Mass
 *
 * @unit kg
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_MASS, 1.f);

/**
 * Inertia matrix, XX component
 *
 * @unit kg m^2
 * @decimal 5
 * @min 0.00001
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XX, 0.01f);

/**
 * Inertia matrix, YY component
 *
 * @unit kg m^2
 * @decimal 5
 * @min 0.00001
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_YY, 0.01f);

/**
 * Inertia matrix, ZZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @min 0.00001
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_ZZ, 0.01f);

/**
 * Inertia matrix, XY component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XY, 0.f);

/**
 * Inertia matrix, XZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_XZ, 0.f);

/**
 * Inertia matrix, YZ component
 *
 * @unit kg m^2
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_INERTIA_YZ, 0.f);

/**
 *
 *
 * @group Vehicle Model
 * @min 0.01
 * @decimal 5
 * @increment 0.00001
 */
PARAM_DEFINE_FLOAT(VM_THR_FACTOR, 4.0f);

/**
 * Motor spin-up time constant
 *
 * @unit s
 * @decimal 5
 * @min 0
 * @increment 0.00001
 * @group Vehicle Model
 * @reboot_required true
 */
PARAM_DEFINE_FLOAT(VM_MOTOR_TAU, 0.005f);

/**
 *
 *
 * @group Vehicle Model
 * @min 0.001
 * @decimal 5
 * @increment 0.00001
 */
PARAM_DEFINE_FLOAT(VM_DRAG_FACTOR, 0.05f);

/**
 * Process noise for vehicle model angular acceleration
 *
 * @unit rad/s^2
 * @decimal 5
 * @min 0.00001
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_ANG_ACC_NOISE, 0.075f);

/**
 * X-axis length Scale used for motor position matrix
 *
 * @unit m
 * @decimal 3
 * @min 0.001
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_LEN_SCALE_X, 1.0f);

/**
 * Y-axis length Scale used for motor position matrix
 *
 * @unit m
 * @decimal 3
 * @min 0.001
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_LEN_SCALE_Y, 1.0f);

/**
 * Z-axis length Scale used for motor position matrix
 *
 * @unit m
 * @decimal 3
 * @min 0.001
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_LEN_SCALE_Z, 1.0f);

/**
 * X-axis Center of Gravity Offset to Body Frame
 *
 * @unit m
 * @decimal 3
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_COG_OFF_X, 0.0f);

/**
 * Y-axis Center of Gravity Offset to Body Frame
 *
 * @unit m
 * @decimal 3
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_COG_OFF_Y, 0.0f);

/**
 * Z-axis Center of Gravity Offset to Body Frame
 *
 * @unit m
 * @decimal 3
 * @increment 0.001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_COG_OFF_Z, 0.0f);


