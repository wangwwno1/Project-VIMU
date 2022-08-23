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
 * Control limit for Exponential Moving Average (EMA) validation of the gps velocity innovation, expressed in standard deviation.
 *
 * The formula is EMA(T) = min(max(val, -CAP), +CAP) * Alpha + (1-Alpha) * EMA(T-1)
 * Set zero to inhibit validation.
 *
 * @group Innovation Validator
 * @min 0.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_EMA_H, 0.f);

/**
 * Alpha for Exponential Moving Average (EMA) validation of the gps velocity innovation.
 *
 * @group Innovation Validator
 * @min 0.0001
 * @max 1.0
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(IV_GPS_ALPHA, 1.f);

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
PARAM_DEFINE_FLOAT(IV_GPS_EMA_CAP, 0.f);
