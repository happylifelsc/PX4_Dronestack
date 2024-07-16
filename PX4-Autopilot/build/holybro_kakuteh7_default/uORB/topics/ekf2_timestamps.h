/****************************************************************************
 *
 *   Copyright (C) 2013-2022 PX4 Development Team. All rights reserved.
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

/* Auto-generated by genmsg_cpp from file /home/cpsl/PX4-Autopilot/msg/Ekf2Timestamps.msg */


#pragma once


#include <uORB/uORB.h>


#ifndef __cplusplus
#define EKF2_TIMESTAMPS_RELATIVE_TIMESTAMP_INVALID 32767

#endif


#ifdef __cplusplus
struct __EXPORT ekf2_timestamps_s {
#else
struct ekf2_timestamps_s {
#endif
	uint64_t timestamp;
	int16_t airspeed_timestamp_rel;
	int16_t distance_sensor_timestamp_rel;
	int16_t optical_flow_timestamp_rel;
	int16_t vehicle_air_data_timestamp_rel;
	int16_t vehicle_magnetometer_timestamp_rel;
	int16_t visual_odometry_timestamp_rel;
	uint8_t _padding0[4]; // required for logger


#ifdef __cplusplus
	static constexpr int16_t RELATIVE_TIMESTAMP_INVALID = 32767;

#endif
};

#ifdef __cplusplus
namespace px4 {
	namespace msg {
		using Ekf2Timestamps = ekf2_timestamps_s;
	} // namespace msg
} // namespace px4
#endif

/* register this as object request broker structure */
ORB_DECLARE(ekf2_timestamps);


#ifdef __cplusplus
void print_message(const orb_metadata *meta, const ekf2_timestamps_s& message);
#endif