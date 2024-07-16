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


// auto-generated file

#pragma once

#include <ucdr/microcdr.h>
#include <string.h>
#include <uORB/topics/resilient_state_estimate.h>


static inline constexpr int ucdr_topic_size_resilient_state_estimate()
{
	return 185;
}

bool ucdr_serialize_resilient_state_estimate(const resilient_state_estimate_s& topic, ucdrBuffer& buf, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	const uint64_t timestamp_adjusted = topic.timestamp + time_offset;
	memcpy(buf.iterator, &timestamp_adjusted, sizeof(topic.timestamp));
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.rotor_speed) == 16, "size mismatch");
	memcpy(buf.iterator, &topic.rotor_speed, sizeof(topic.rotor_speed));
	buf.iterator += sizeof(topic.rotor_speed);
	buf.offset += sizeof(topic.rotor_speed);
	static_assert(sizeof(topic.torque_ground_truth) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.torque_ground_truth, sizeof(topic.torque_ground_truth));
	buf.iterator += sizeof(topic.torque_ground_truth);
	buf.offset += sizeof(topic.torque_ground_truth);
	static_assert(sizeof(topic.thrust_ground_truth) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.thrust_ground_truth, sizeof(topic.thrust_ground_truth));
	buf.iterator += sizeof(topic.thrust_ground_truth);
	buf.offset += sizeof(topic.thrust_ground_truth);
	static_assert(sizeof(topic.raw_angular_vel) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.raw_angular_vel, sizeof(topic.raw_angular_vel));
	buf.iterator += sizeof(topic.raw_angular_vel);
	buf.offset += sizeof(topic.raw_angular_vel);
	static_assert(sizeof(topic.raw_acc) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.raw_acc, sizeof(topic.raw_acc));
	buf.iterator += sizeof(topic.raw_acc);
	buf.offset += sizeof(topic.raw_acc);
	static_assert(sizeof(topic.imu_angular_vel) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.imu_angular_vel, sizeof(topic.imu_angular_vel));
	buf.iterator += sizeof(topic.imu_angular_vel);
	buf.offset += sizeof(topic.imu_angular_vel);
	static_assert(sizeof(topic.imu_acc) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.imu_acc, sizeof(topic.imu_acc));
	buf.iterator += sizeof(topic.imu_acc);
	buf.offset += sizeof(topic.imu_acc);
	static_assert(sizeof(topic.attitude_standard) == 16, "size mismatch");
	memcpy(buf.iterator, &topic.attitude_standard, sizeof(topic.attitude_standard));
	buf.iterator += sizeof(topic.attitude_standard);
	buf.offset += sizeof(topic.attitude_standard);
	static_assert(sizeof(topic.torque_estimate) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.torque_estimate, sizeof(topic.torque_estimate));
	buf.iterator += sizeof(topic.torque_estimate);
	buf.offset += sizeof(topic.torque_estimate);
	static_assert(sizeof(topic.thrust_estimate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.thrust_estimate, sizeof(topic.thrust_estimate));
	buf.iterator += sizeof(topic.thrust_estimate);
	buf.offset += sizeof(topic.thrust_estimate);
	static_assert(sizeof(topic.attitude_estimate) == 16, "size mismatch");
	memcpy(buf.iterator, &topic.attitude_estimate, sizeof(topic.attitude_estimate));
	buf.iterator += sizeof(topic.attitude_estimate);
	buf.offset += sizeof(topic.attitude_estimate);
	static_assert(sizeof(topic.angular_vel_estimate) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.angular_vel_estimate, sizeof(topic.angular_vel_estimate));
	buf.iterator += sizeof(topic.angular_vel_estimate);
	buf.offset += sizeof(topic.angular_vel_estimate);
	static_assert(sizeof(topic.anomaly_score) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.anomaly_score, sizeof(topic.anomaly_score));
	buf.iterator += sizeof(topic.anomaly_score);
	buf.offset += sizeof(topic.anomaly_score);
	static_assert(sizeof(topic.anomaly_detection_rate) == 4, "size mismatch");
	memcpy(buf.iterator, &topic.anomaly_detection_rate, sizeof(topic.anomaly_detection_rate));
	buf.iterator += sizeof(topic.anomaly_detection_rate);
	buf.offset += sizeof(topic.anomaly_detection_rate);
	static_assert(sizeof(topic.anomaly_flag) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.anomaly_flag, sizeof(topic.anomaly_flag));
	buf.iterator += sizeof(topic.anomaly_flag);
	buf.offset += sizeof(topic.anomaly_flag);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.imu_acc_offset) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.imu_acc_offset, sizeof(topic.imu_acc_offset));
	buf.iterator += sizeof(topic.imu_acc_offset);
	buf.offset += sizeof(topic.imu_acc_offset);
	static_assert(sizeof(topic.imu_gyro_offset) == 12, "size mismatch");
	memcpy(buf.iterator, &topic.imu_gyro_offset, sizeof(topic.imu_gyro_offset));
	buf.iterator += sizeof(topic.imu_gyro_offset);
	buf.offset += sizeof(topic.imu_gyro_offset);
	static_assert(sizeof(topic.attack_flag) == 1, "size mismatch");
	memcpy(buf.iterator, &topic.attack_flag, sizeof(topic.attack_flag));
	buf.iterator += sizeof(topic.attack_flag);
	buf.offset += sizeof(topic.attack_flag);
	return true;
}

bool ucdr_deserialize_resilient_state_estimate(ucdrBuffer& buf, resilient_state_estimate_s& topic, int64_t time_offset = 0)
{
	static_assert(sizeof(topic.timestamp) == 8, "size mismatch");
	memcpy(&topic.timestamp, buf.iterator, sizeof(topic.timestamp));
	if (topic.timestamp == 0) topic.timestamp = hrt_absolute_time();
	else topic.timestamp = math::min(topic.timestamp - time_offset, hrt_absolute_time());
	buf.iterator += sizeof(topic.timestamp);
	buf.offset += sizeof(topic.timestamp);
	static_assert(sizeof(topic.rotor_speed) == 16, "size mismatch");
	memcpy(&topic.rotor_speed, buf.iterator, sizeof(topic.rotor_speed));
	buf.iterator += sizeof(topic.rotor_speed);
	buf.offset += sizeof(topic.rotor_speed);
	static_assert(sizeof(topic.torque_ground_truth) == 12, "size mismatch");
	memcpy(&topic.torque_ground_truth, buf.iterator, sizeof(topic.torque_ground_truth));
	buf.iterator += sizeof(topic.torque_ground_truth);
	buf.offset += sizeof(topic.torque_ground_truth);
	static_assert(sizeof(topic.thrust_ground_truth) == 4, "size mismatch");
	memcpy(&topic.thrust_ground_truth, buf.iterator, sizeof(topic.thrust_ground_truth));
	buf.iterator += sizeof(topic.thrust_ground_truth);
	buf.offset += sizeof(topic.thrust_ground_truth);
	static_assert(sizeof(topic.raw_angular_vel) == 12, "size mismatch");
	memcpy(&topic.raw_angular_vel, buf.iterator, sizeof(topic.raw_angular_vel));
	buf.iterator += sizeof(topic.raw_angular_vel);
	buf.offset += sizeof(topic.raw_angular_vel);
	static_assert(sizeof(topic.raw_acc) == 12, "size mismatch");
	memcpy(&topic.raw_acc, buf.iterator, sizeof(topic.raw_acc));
	buf.iterator += sizeof(topic.raw_acc);
	buf.offset += sizeof(topic.raw_acc);
	static_assert(sizeof(topic.imu_angular_vel) == 12, "size mismatch");
	memcpy(&topic.imu_angular_vel, buf.iterator, sizeof(topic.imu_angular_vel));
	buf.iterator += sizeof(topic.imu_angular_vel);
	buf.offset += sizeof(topic.imu_angular_vel);
	static_assert(sizeof(topic.imu_acc) == 12, "size mismatch");
	memcpy(&topic.imu_acc, buf.iterator, sizeof(topic.imu_acc));
	buf.iterator += sizeof(topic.imu_acc);
	buf.offset += sizeof(topic.imu_acc);
	static_assert(sizeof(topic.attitude_standard) == 16, "size mismatch");
	memcpy(&topic.attitude_standard, buf.iterator, sizeof(topic.attitude_standard));
	buf.iterator += sizeof(topic.attitude_standard);
	buf.offset += sizeof(topic.attitude_standard);
	static_assert(sizeof(topic.torque_estimate) == 12, "size mismatch");
	memcpy(&topic.torque_estimate, buf.iterator, sizeof(topic.torque_estimate));
	buf.iterator += sizeof(topic.torque_estimate);
	buf.offset += sizeof(topic.torque_estimate);
	static_assert(sizeof(topic.thrust_estimate) == 4, "size mismatch");
	memcpy(&topic.thrust_estimate, buf.iterator, sizeof(topic.thrust_estimate));
	buf.iterator += sizeof(topic.thrust_estimate);
	buf.offset += sizeof(topic.thrust_estimate);
	static_assert(sizeof(topic.attitude_estimate) == 16, "size mismatch");
	memcpy(&topic.attitude_estimate, buf.iterator, sizeof(topic.attitude_estimate));
	buf.iterator += sizeof(topic.attitude_estimate);
	buf.offset += sizeof(topic.attitude_estimate);
	static_assert(sizeof(topic.angular_vel_estimate) == 12, "size mismatch");
	memcpy(&topic.angular_vel_estimate, buf.iterator, sizeof(topic.angular_vel_estimate));
	buf.iterator += sizeof(topic.angular_vel_estimate);
	buf.offset += sizeof(topic.angular_vel_estimate);
	static_assert(sizeof(topic.anomaly_score) == 4, "size mismatch");
	memcpy(&topic.anomaly_score, buf.iterator, sizeof(topic.anomaly_score));
	buf.iterator += sizeof(topic.anomaly_score);
	buf.offset += sizeof(topic.anomaly_score);
	static_assert(sizeof(topic.anomaly_detection_rate) == 4, "size mismatch");
	memcpy(&topic.anomaly_detection_rate, buf.iterator, sizeof(topic.anomaly_detection_rate));
	buf.iterator += sizeof(topic.anomaly_detection_rate);
	buf.offset += sizeof(topic.anomaly_detection_rate);
	static_assert(sizeof(topic.anomaly_flag) == 1, "size mismatch");
	memcpy(&topic.anomaly_flag, buf.iterator, sizeof(topic.anomaly_flag));
	buf.iterator += sizeof(topic.anomaly_flag);
	buf.offset += sizeof(topic.anomaly_flag);
	buf.iterator += 3; // padding
	buf.offset += 3; // padding
	static_assert(sizeof(topic.imu_acc_offset) == 12, "size mismatch");
	memcpy(&topic.imu_acc_offset, buf.iterator, sizeof(topic.imu_acc_offset));
	buf.iterator += sizeof(topic.imu_acc_offset);
	buf.offset += sizeof(topic.imu_acc_offset);
	static_assert(sizeof(topic.imu_gyro_offset) == 12, "size mismatch");
	memcpy(&topic.imu_gyro_offset, buf.iterator, sizeof(topic.imu_gyro_offset));
	buf.iterator += sizeof(topic.imu_gyro_offset);
	buf.offset += sizeof(topic.imu_gyro_offset);
	static_assert(sizeof(topic.attack_flag) == 1, "size mismatch");
	memcpy(&topic.attack_flag, buf.iterator, sizeof(topic.attack_flag));
	buf.iterator += sizeof(topic.attack_flag);
	buf.offset += sizeof(topic.attack_flag);
	return true;
}
