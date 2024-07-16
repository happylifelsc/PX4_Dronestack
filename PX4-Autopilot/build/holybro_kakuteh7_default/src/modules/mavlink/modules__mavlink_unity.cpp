/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink.c
 * Define MAVLink specific parameters
 *
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <px4_platform_common/px4_config.h>
#include <unistd.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "mavlink_bridge_header.h"
#include <parameters/param.h>

mavlink_system_t mavlink_system = {
	1,
	1
}; // System ID, 1-255, Component/Subsystem ID, 1-255
/****************************************************************************
 *
 *   Copyright (c) 2017-2021 PX4 Development Team. All rights reserved.
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
 * @file mavlink_command_sender.cpp
 * Mavlink commands sender with support for retransmission.
 *
 * @author Julian Oes <julian@oes.ch>
 */

#include "mavlink_command_sender.h"
#include <px4_platform_common/log.h>

#define CMD_DEBUG(FMT, ...) PX4_LOG_NAMED_COND("cmd sender", _debug_enabled, FMT, ##__VA_ARGS__)

MavlinkCommandSender *MavlinkCommandSender::_instance = nullptr;
px4_sem_t MavlinkCommandSender::_lock;

void MavlinkCommandSender::initialize()
{
	px4_sem_init(&_lock, 1, 1);

	if (_instance == nullptr) {
		_instance = new MavlinkCommandSender();
	}
}

MavlinkCommandSender &MavlinkCommandSender::instance()
{
	return *_instance;
}

MavlinkCommandSender::~MavlinkCommandSender()
{
	px4_sem_destroy(&_lock);
}

int MavlinkCommandSender::handle_vehicle_command(const vehicle_command_s &command, mavlink_channel_t channel)
{
	// commands > uint16 are PX4 internal only
	if (command.command >= vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START) {
		return 0;
	}

	lock();
	CMD_DEBUG("new command: %" PRIu32 " (channel: %d)", command.command, channel);

	mavlink_command_long_t msg = {};
	msg.target_system = command.target_system;
	msg.target_component = command.target_component;
	msg.command = command.command;
	msg.confirmation = command.confirmation;
	msg.param1 = command.param1;
	msg.param2 = command.param2;
	msg.param3 = command.param3;
	msg.param4 = command.param4;
	msg.param5 = command.param5;
	msg.param6 = command.param6;
	msg.param7 = command.param7;
	mavlink_msg_command_long_send_struct(channel, &msg);

	bool already_existing = false;
	_commands.reset_to_start();

	while (command_item_s *item = _commands.get_next()) {
		if (item->timestamp_us == command.timestamp) {

			// We should activate the channel by setting num_sent_per_channel from -1 to 0.
			item->num_sent_per_channel[channel] = 0;
			already_existing = true;
			break;
		}
	}

	if (!already_existing) {

		command_item_s new_item;
		new_item.command = msg;
		new_item.timestamp_us = command.timestamp;
		new_item.num_sent_per_channel[channel] = 0;
		new_item.last_time_sent_us = hrt_absolute_time();
		_commands.put(new_item);
	}

	unlock();
	return 0;
}

void MavlinkCommandSender::handle_mavlink_command_ack(const mavlink_command_ack_t &ack,
		uint8_t from_sysid, uint8_t from_compid, uint8_t channel)
{
	CMD_DEBUG("handling result %" PRIu8 " for command %" PRIu16 " (from %" PRIu8 ":%" PRIu8 ")",
		  ack.result, ack.command, from_sysid, from_compid);
	lock();

	_commands.reset_to_start();

	while (command_item_s *item = _commands.get_next()) {
		// Check if the incoming ack matches any of the commands that we have sent.
		if (item->command.command == ack.command &&
		    (item->command.target_system == 0 || from_sysid == item->command.target_system) &&
		    (item->command.target_component == 0 || from_compid == item->command.target_component) &&
		    item->num_sent_per_channel[channel] != -1) {
			item->num_sent_per_channel[channel] = -2;	// mark this as acknowledged
			break;
		}
	}

	unlock();
}

void MavlinkCommandSender::check_timeout(mavlink_channel_t channel)
{
	lock();

	_commands.reset_to_start();

	while (command_item_s *item = _commands.get_next()) {
		if (hrt_elapsed_time(&item->last_time_sent_us) <= TIMEOUT_US) {
			// We keep waiting for the timeout.
			continue;
		}

		// Loop through num_sent_per_channel and check if any channel has receives an ack for this command
		// (indicated by the value -2). We avoid removing the command at the time of receiving the ack
		// as some channels might be lagging behind and will end up putting the same command into the buffer.
		bool dropped_command = false;

		for (unsigned i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i) {
			if (item->num_sent_per_channel[i] == -2) {
				_commands.drop_current();
				dropped_command = true;
				break;
			}
		}

		if (dropped_command) {
			continue;
		}

		// The goal of this is to retry from all channels. Therefore, we keep
		// track of the retry count for each channel.
		//
		// When the first channel does a retry, the timeout is reset.
		// (e.g. all channel have done 2 retries, then channel 0 is called
		// and does retry number 3, and also resets the timeout timestamp).

		// First, we need to determine what the current max and min retry level
		// are because we can only level up, if all have caught up.
		// If num_sent_per_channel is at -1, the channel is inactive.
		int8_t max_sent = 0;
		int8_t min_sent = INT8_MAX;

		for (unsigned i = 0; i < MAVLINK_COMM_NUM_BUFFERS; ++i) {
			if (item->num_sent_per_channel[i] > max_sent) {
				max_sent = item->num_sent_per_channel[i];
			}

			if ((item->num_sent_per_channel[i] != -1) &&
			    (item->num_sent_per_channel[i] < min_sent)) {
				min_sent = item->num_sent_per_channel[i];
			}
		}

		if (item->num_sent_per_channel[channel] < max_sent && item->num_sent_per_channel[channel] != -1) {
			// We are behind and need to do a retransmission.
			item->command.confirmation = ++item->num_sent_per_channel[channel];
			mavlink_msg_command_long_send_struct(channel, &item->command);

			CMD_DEBUG("command %" PRIu16 " sent (not first, retries: %" PRIu8 "/%" PRIi8 ", channel: %d)",
				  item->command.command,
				  item->num_sent_per_channel[channel],
				  max_sent,
				  channel);

		} else if (item->num_sent_per_channel[channel] == max_sent &&
			   min_sent == max_sent) {

			// If the next retry would be above the needed retries anyway, we can
			// drop the item, and continue with other items.
			if (item->num_sent_per_channel[channel] + 1 > RETRIES) {
				CMD_DEBUG("command %" PRIu16 " dropped", item->command.command);
				_commands.drop_current();
				continue;
			}

			// We are the first of a new retransmission series.
			item->command.confirmation = ++item->num_sent_per_channel[channel];
			mavlink_msg_command_long_send_struct(channel, &item->command);
			// Therefore, we are the ones setting the timestamp of this retry round.
			item->last_time_sent_us = hrt_absolute_time();

			CMD_DEBUG("command %" PRIu16 " sent (first, retries: %" PRId8 "/%" PRId8 ", channel: %d)",
				  item->command.command,
				  item->num_sent_per_channel[channel],
				  max_sent,
				  channel);

		} else {
			// We are already ahead, so this should not happen.
			// If it ever does, just ignore it. It will timeout eventually.
			continue;
		}
	}

	unlock();
}
/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

#include "mavlink_events.h"
#include "mavlink_main.h"

#include <px4_log.h>
#include <errno.h>

namespace events
{

EventBuffer::EventBuffer(int capacity)
	: _capacity(capacity)
{
	pthread_mutex_init(&_mutex, nullptr);
}

EventBuffer::~EventBuffer()
{
	delete[](_events);
	pthread_mutex_destroy(&_mutex);
}

int EventBuffer::init()
{
	if (_events) { return 0; }

	_events = new Event[_capacity];

	if (!_events) {
		return -ENOMEM;
	}

	return 0;
}

void EventBuffer::insert_event(const Event &event)
{
	pthread_mutex_lock(&_mutex);
	_events[_next] = event;
	_next = (_next + 1) % _capacity;

	if (_size < _capacity) {
		++_size;
	}

	_latest_sequence.store(event.sequence);
	pthread_mutex_unlock(&_mutex);
}

uint16_t EventBuffer::get_oldest_sequence_after(uint16_t sequence) const
{
	pthread_mutex_lock(&_mutex);
	uint16_t sequence_ret = _latest_sequence.load();
	uint16_t min_diff = UINT16_MAX;

	for (int i = 0; i < _size; ++i) {
		uint16_t event_seq = _events[i].sequence;
		uint16_t diff = event_seq - sequence;

		// this handles wrap-arounds correctly
		if (event_seq != sequence && diff < min_diff) {
			min_diff = diff;
			sequence_ret = event_seq;
		}
	}

	pthread_mutex_unlock(&_mutex);
	return sequence_ret;
}
bool EventBuffer::get_event(uint16_t sequence, Event &event) const
{
	pthread_mutex_lock(&_mutex);

	for (int count = 0; count < _size; ++count) {
		int index = (_next - 1 - count + _size) % _size;

		if (_events[index].sequence == sequence) {
			event = _events[index];
			pthread_mutex_unlock(&_mutex);
			return true;
		}
	}

	pthread_mutex_unlock(&_mutex);
	return false;
}

int EventBuffer::size() const
{
	pthread_mutex_lock(&_mutex);
	int size = _size;
	pthread_mutex_unlock(&_mutex);
	return size;
}

SendProtocol::SendProtocol(EventBuffer &buffer, Mavlink &mavlink)
	: _buffer(buffer), _latest_sequence(buffer.get_latest_sequence()), _mavlink(mavlink)
{
}

void SendProtocol::update(const hrt_abstime &now)
{
	// check for new events in the buffer
	uint16_t buffer_sequence = _buffer.get_latest_sequence();
	int num_drops = 0;

	if (_first_event && buffer_sequence != _latest_sequence) {
		// If events are published before the first mavlink instance starts, they would be flagged as dropped
		// events as the buffer initializes the sequence to the global events::initial_event_sequence,
		// therefore reset the sequence.
		_latest_sequence = buffer_sequence - 1;
		_first_event = false;
		PX4_DEBUG("Setting initial sequence to %i", _latest_sequence);
		// Send sequence - only needed for SITL to ensure we send the sequence reset flag initially
		send_current_sequence(now, true);
	}

	while (_latest_sequence != buffer_sequence) {
		// only send if enough tx buffer space available
		if (_mavlink.get_free_tx_buf() < MAVLINK_MSG_ID_EVENT_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
			break;
		}

		PX4_DEBUG("Changed seq: %i, latest: %i (mavlink instance: %i)", buffer_sequence, _latest_sequence,
			  _mavlink.get_instance_id());
		++_latest_sequence;
		Event e;

		if (_buffer.get_event(_latest_sequence, e)) {
			send_event(e);

		} else {
			if (num_drops == 0) { // avoid console spamming
				// This happens if either an event dropped in uORB or update() is not called fast enough
				PX4_WARN("Event dropped (%i, %i)", (int)_latest_sequence, buffer_sequence);
			}

			++num_drops;
		}
	}

	if (num_drops > 1) {
		PX4_WARN("Dropped %i events (seq=%i)", num_drops, _latest_sequence);
	}

	if (now - _last_current_sequence_sent > current_sequence_interval) {
		send_current_sequence(now);
	}
}

void SendProtocol::handle_request_event(const mavlink_message_t &msg) const
{
	mavlink_request_event_t request_event;
	mavlink_msg_request_event_decode(&msg, &request_event);
	Event e;

	const uint16_t end_sequence = request_event.last_sequence + 1;

	for (uint16_t sequence = request_event.first_sequence; sequence != end_sequence; ++sequence) {
		if (_buffer.get_event(sequence, e)) {
			PX4_DEBUG("sending requested event %i", sequence);
			send_event(e);

		} else {
			mavlink_response_event_error_t event_error{};
			event_error.target_system = msg.sysid;
			event_error.target_component = msg.compid;
			event_error.sequence = sequence;
			event_error.sequence_oldest_available = _buffer.get_oldest_sequence_after(sequence);
			event_error.reason = MAV_EVENT_ERROR_REASON_UNAVAILABLE;
			PX4_DEBUG("Event unavailable (seq=%i oldest=%i)", sequence, event_error.sequence_oldest_available);
			mavlink_msg_response_event_error_send_struct(_mavlink.get_channel(), &event_error);
		}
	}
}

void SendProtocol::send_event(const Event &event) const
{
	mavlink_event_t event_msg{};
	event_msg.event_time_boot_ms = event.timestamp_ms;
	event_msg.destination_component = MAV_COMP_ID_ALL;
	event_msg.destination_system = 0;
	event_msg.id = event.id;
	event_msg.sequence = event.sequence;
	event_msg.log_levels = event.log_levels;
	static_assert(sizeof(event_msg.arguments) >= sizeof(event.arguments), "MAVLink message arguments buffer too small");
	memcpy(&event_msg.arguments, event.arguments, sizeof(event.arguments));
	mavlink_msg_event_send_struct(_mavlink.get_channel(), &event_msg);

}

void SendProtocol::on_gcs_connected()
{
	send_current_sequence(hrt_absolute_time());
}

void SendProtocol::send_current_sequence(const hrt_abstime &now, bool force_reset)
{
	// only send if enough tx buffer space available
	if (_mavlink.get_free_tx_buf() < MAVLINK_MSG_ID_CURRENT_EVENT_SEQUENCE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
		return;
	}

	_last_current_sequence_sent = now;
	mavlink_current_event_sequence_t current_event_seq;
	current_event_seq.sequence = _buffer.get_latest_sequence();
	current_event_seq.flags = (_buffer.size() == 0 || force_reset) ? MAV_EVENT_CURRENT_SEQUENCE_FLAGS_RESET : 0;
	mavlink_msg_current_event_sequence_send_struct(_mavlink.get_channel(), &current_event_seq);
}

} /* namespace events */
/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

/// @file mavlink_ftp.cpp
///	@author px4dev, Don Gagne <don@thegagnes.com>

#include <crc32.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>

#include "mavlink_ftp.h"
#include "mavlink_tests/mavlink_ftp_test.h"

#ifndef MAVLINK_FTP_UNIT_TEST
#include "mavlink_main.h"
#else
#include <mavlink.h>
#endif

using namespace time_literals;

constexpr const char MavlinkFTP::_root_dir[];

MavlinkFTP::MavlinkFTP(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	// initialize session
	_session_info.fd = -1;
}

MavlinkFTP::~MavlinkFTP()
{
	delete[] _work_buffer1;
	delete[] _work_buffer2;
}

unsigned
MavlinkFTP::get_size()
{
	if (_session_info.stream_download) {
		return MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

	} else {
		return 0;
	}
}

#ifdef MAVLINK_FTP_UNIT_TEST
void
MavlinkFTP::set_unittest_worker(ReceiveMessageFunc_t rcvMsgFunc, void *worker_data)
{
	_utRcvMsgFunc = rcvMsgFunc;
	_worker_data = worker_data;
}
#endif

uint8_t
MavlinkFTP::_getServerSystemId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverSystemId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_system_id();
#endif
}

uint8_t
MavlinkFTP::_getServerComponentId()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverComponentId;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_component_id();
#endif
}

uint8_t
MavlinkFTP::_getServerChannel()
{
#ifdef MAVLINK_FTP_UNIT_TEST
	// We use fake ids when unit testing
	return MavlinkFtpTest::serverChannel;
#else
	// Not unit testing, use the real thing
	return _mavlink->get_channel();
#endif
}

void
MavlinkFTP::handle_message(const mavlink_message_t *msg)
{
	//warnx("MavlinkFTP::handle_message %d %d", buf_size_1, buf_size_2);

	if (msg->msgid == MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL) {
		mavlink_file_transfer_protocol_t ftp_request;
		mavlink_msg_file_transfer_protocol_decode(msg, &ftp_request);

		PX4_DEBUG("FTP: received ftp protocol message target_system: %d target_component: %d, seq: %d",
			  ftp_request.target_system, ftp_request.target_component, msg->seq);

		if ((ftp_request.target_system == _getServerSystemId() || ftp_request.target_system == 0) &&
		    (ftp_request.target_component == _getServerComponentId() || ftp_request.target_component == 0)) {
			_process_request(&ftp_request, msg->sysid, msg->compid);
		}
	}
}

/// @brief Processes an FTP message
void
MavlinkFTP::_process_request(
	mavlink_file_transfer_protocol_t *ftp_req,
	uint8_t target_system_id,
	uint8_t target_comp_id)
{
	bool stream_send = false;
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	ErrorCode errorCode = kErrNone;

	if (!_ensure_buffers_exist()) {
		PX4_ERR("Failed to allocate buffers");
		errorCode = kErrFailErrno;
		_our_errno = ENOMEM;
		goto out;
	}

	// basic sanity checks; must validate length before use
	if (payload->size > kMaxDataLength) {
		errorCode = kErrInvalidDataSize;
		PX4_WARN("invalid data size: %d", payload->size);
		goto out;
	}

	// check the sequence number: if this is a resent request, resend the last response
	if (_last_reply_valid) {
		mavlink_file_transfer_protocol_t *last_reply = reinterpret_cast<mavlink_file_transfer_protocol_t *>(_last_reply);
		PayloadHeader *last_payload = reinterpret_cast<PayloadHeader *>(&last_reply->payload[0]);

		if (payload->seq_number + 1 == last_payload->seq_number) {
			// this is the same request as the one we replied to last. It means the (n)ack got lost, and the GCS
			// resent the request
#ifdef MAVLINK_FTP_UNIT_TEST
			_utRcvMsgFunc(last_reply, _worker_data);
#else
			mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), last_reply);
#endif
			return;
		}
	}


	PX4_DEBUG("ftp: channel %" PRIu8 " opc %" PRIu8 " size %" PRIu8 " offset %" PRIu32,
		  _getServerChannel(), payload->opcode, payload->size, payload->offset);

	switch (payload->opcode) {
	case kCmdNone:
		break;

	case kCmdTerminateSession:
		errorCode = _workTerminate(payload);
		break;

	case kCmdResetSessions:
		errorCode = _workReset(payload);
		break;

	case kCmdListDirectory:
		errorCode = _workList(payload);
		break;

	case kCmdOpenFileRO:
		errorCode = _workOpen(payload, O_RDONLY);
		break;

	case kCmdCreateFile:
		errorCode = _workOpen(payload, O_CREAT | O_TRUNC | O_WRONLY);
		break;

	case kCmdOpenFileWO:
		errorCode = _workOpen(payload, O_CREAT | O_WRONLY);
		break;

	case kCmdReadFile:
		errorCode = _workRead(payload);
		break;

	case kCmdBurstReadFile:
		errorCode = _workBurst(payload, target_system_id, target_comp_id);
		stream_send = true;
		break;

	case kCmdWriteFile:
		errorCode = _workWrite(payload);
		break;

	case kCmdRemoveFile:
		errorCode = _workRemoveFile(payload);
		break;

	case kCmdRename:
		errorCode = _workRename(payload);
		break;

	case kCmdTruncateFile:
		errorCode = _workTruncateFile(payload);
		break;

	case kCmdCreateDirectory:
		errorCode = _workCreateDirectory(payload);
		break;

	case kCmdRemoveDirectory:
		errorCode = _workRemoveDirectory(payload);
		break;

	case kCmdCalcFileCRC32:
		errorCode = _workCalcFileCRC32(payload);
		break;

	default:
		errorCode = kErrUnknownCommand;
		break;
	}

out:
	payload->seq_number++;

	// handle success vs. error
	if (errorCode == kErrNone) {
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspAck;

	} else {
		PX4_DEBUG("errorCode: %d, errno: %d / %s", errorCode, _our_errno, strerror(_our_errno));
		payload->req_opcode = payload->opcode;
		payload->opcode = kRspNak;
		payload->size = 1;

		if (_our_errno == EEXIST) {
			errorCode = kErrFailFileExists;

		} else if (_our_errno == ENOENT && errorCode == kErrFailErrno) {
			errorCode = kErrFileNotFound;
		}

		payload->data[0] = errorCode;

		if (errorCode == kErrFailErrno) {
			payload->size = 2;
			payload->data[1] = _our_errno;
		}
	}

	_last_reply_valid = false;

	// Stream download replies are sent through mavlink stream mechanism. Unless we need to Nack.
	if (!stream_send || errorCode != kErrNone) {
		// respond to the request
		ftp_req->target_system = target_system_id;
		ftp_req->target_network = 0;
		ftp_req->target_component = target_comp_id;
		_reply(ftp_req);
	}
}

bool MavlinkFTP::_ensure_buffers_exist()
{
	_last_work_buffer_access = hrt_absolute_time();

	if (!_work_buffer1) {
		_work_buffer1 = new char[_work_buffer1_len];
	}

	if (!_work_buffer2) {
		_work_buffer2 = new char[_work_buffer2_len];
	}

	return _work_buffer1 && _work_buffer2;
}

/// @brief Sends the specified FTP response message out through mavlink
void
MavlinkFTP::_reply(mavlink_file_transfer_protocol_t *ftp_req)
{
	PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_req->payload[0]);

	// keep a copy of the last sent response ((n)ack), so that if it gets lost and the GCS resends the request,
	// we can simply resend the response.
	// we only keep small responses to reduce RAM usage and avoid large memcpy's. The larger responses are all data
	// retrievals without side-effects, meaning it's ok to reexecute them if a response gets lost
	if (payload->size <= sizeof(uint32_t)) {
		_last_reply_valid = true;
		memcpy(_last_reply, ftp_req, sizeof(_last_reply));
	}

	// clear any not used payload data to correctly trim mavlink ftp message reply
	memset(&payload->data[payload->size], 0, kMaxDataLength - payload->size);

	PX4_DEBUG("FTP: %s seq_number: %" PRIu16, payload->opcode == kRspAck ? "Ack" : "Nak", payload->seq_number);

#ifdef MAVLINK_FTP_UNIT_TEST
	// Unit test hook is set, call that instead
	_utRcvMsgFunc(ftp_req, _worker_data);
#else
	mavlink_msg_file_transfer_protocol_send_struct(_mavlink->get_channel(), ftp_req);
#endif

}

/// @brief Responds to a List command
MavlinkFTP::ErrorCode
MavlinkFTP::_workList(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	ErrorCode errorCode = kErrNone;
	unsigned offset = 0;

	PX4_DEBUG("opendir: %s", _work_buffer1);

	DIR *dp = opendir(_work_buffer1);

	if (dp == nullptr) {
		_our_errno = errno;
		PX4_DEBUG("Dir open failed %s: %s", _work_buffer1, strerror(_our_errno));
		return kErrFileNotFound;
	}

	PX4_DEBUG("FTP: list %s offset %" PRIu32, _work_buffer1, payload->offset);

	struct dirent *result = nullptr;

	// move to the requested offset
	int requested_offset = payload->offset;

	PX4_DEBUG("readdir with offset: %d", requested_offset);

	while (requested_offset-- > 0 && readdir(dp)) {}

	for (;;) {
		errno = 0;
		result = readdir(dp);

		// read the directory entry
		if (result == nullptr) {
			_our_errno = errno;

			if (_our_errno) {
				PX4_WARN("readdir failed: %s", strerror(_our_errno));
				payload->data[offset++] = kDirentSkip;
				*((char *)&payload->data[offset]) = '\0';
				offset++;
				payload->size = offset;
				closedir(dp);

				return errorCode;
			}

			// FIXME: does this ever happen? I would assume readdir always sets errno.
			// no more entries?
			if (payload->offset != 0 && offset == 0) {
				// User is requesting subsequent dir entries but there were none. This means the user asked
				// to seek past EOF.
				errorCode = kErrEOF;
			}

			// Otherwise we are just at the last directory entry, so we leave the errorCode at kErrorNone to signal that
			break;
		}

		uint32_t fileSize = 0;
		char direntType;

		// Determine the directory entry type
		switch (result->d_type) {
#ifdef __PX4_NUTTX

		case DTYPE_FILE: {
#else

		case DT_REG: {
#endif
				// For files we get the file size as well
				direntType = kDirentFile;
				int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s/%s", _work_buffer1, result->d_name);
				bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

				if (buf_is_ok) {
					struct stat st;

					if (stat(_work_buffer2, &st) == 0) {
						fileSize = st.st_size;
					}
				}

				break;
			}

#ifdef __PX4_NUTTX

		case DTYPE_DIRECTORY:
#else
		case DT_DIR:
#endif
			if (strcmp(result->d_name, ".") == 0 || strcmp(result->d_name, "..") == 0) {
				// Don't bother sending these back
				direntType = kDirentSkip;

			} else {
				direntType = kDirentDir;
			}

			break;

		default:
			// We only send back file and diretory entries, skip everything else
			direntType = kDirentSkip;
		}

		if (direntType == kDirentSkip) {
			// Skip send only dirent identifier
			_work_buffer2[0] = '\0';

		} else if (direntType == kDirentFile) {
			// Files send filename and file length
			int ret = snprintf(_work_buffer2, _work_buffer2_len, "%s\t%" PRIu32, result->d_name, fileSize);
			bool buf_is_ok = ((ret > 0) && (ret < _work_buffer2_len));

			if (!buf_is_ok) {
				_work_buffer2[_work_buffer2_len - 1] = '\0';
			}

		} else {
			// Everything else just sends name
			strncpy(_work_buffer2, result->d_name, _work_buffer2_len);
			_work_buffer2[_work_buffer2_len - 1] = '\0';
		}

		size_t nameLen = strlen(_work_buffer2);

		// Do we have room for the name, the one char directory identifier and the null terminator?
		if ((offset + nameLen + 2) > kMaxDataLength) {
			break;
		}

		// Move the data into the buffer
		payload->data[offset++] = direntType;
		strcpy((char *)&payload->data[offset], _work_buffer2);
		PX4_DEBUG("FTP: list %s %s", _work_buffer1, (char *)&payload->data[offset - 1]);
		offset += nameLen + 1;
	}

	closedir(dp);
	payload->size = offset;

	return errorCode;
}

/// @brief Responds to an Open command
MavlinkFTP::ErrorCode
MavlinkFTP::_workOpen(PayloadHeader *payload, int oflag)
{
	if (_session_info.fd >= 0) {
		PX4_ERR("FTP: Open failed - out of sessions\n");
		return kErrNoSessionsAvailable;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);

	PX4_DEBUG("FTP: open '%s'", _work_buffer1);

	uint32_t fileSize = 0;
	struct stat st;

	PX4_DEBUG("stat: %s", _work_buffer1);

	if (stat(_work_buffer1, &st) != 0) {
		// fail only if requested open for read
		if (oflag & O_RDONLY) {
			_our_errno = errno;
			PX4_ERR("stat failed read: %s", strerror(_our_errno));
			return kErrFailErrno;

		} else {
			st.st_size = 0;
		}
	}

	fileSize = st.st_size;

	PX4_DEBUG("open: %s", _work_buffer1);
	// Set mode to 666 incase oflag has O_CREAT
	int fd = ::open(_work_buffer1, oflag, PX4_O_MODE_666);

	if (fd < 0) {
		_our_errno = errno;
		PX4_ERR("open failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	_session_info.fd = fd;
	_session_info.file_size = fileSize;
	_session_info.stream_download = false;

	payload->session = 0;
	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &fileSize, payload->size);

	return kErrNone;
}

/// @brief Responds to a Read command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRead(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	PX4_DEBUG("FTP: read offset:%ld" PRIu32, payload->offset);

	// We have to test seek past EOF ourselves, lseek will allow seek past EOF
	if (payload->offset >= _session_info.file_size) {
		PX4_WARN("request past EOF");
		return kErrEOF;
	}

	PX4_DEBUG("lseek with offset: %ld", payload->offset);

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		_our_errno = errno;
		PX4_ERR("seek fail: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	int bytes_read = ::read(_session_info.fd, &payload->data[0], payload->size);

	if (bytes_read < 0) {
		// Negative return indicates error other than eof
		_our_errno = errno;
		PX4_ERR("read fail %d, %s", bytes_read, strerror(_our_errno));
		return kErrFailErrno;
	}

	payload->size = bytes_read;

	return kErrNone;
}

/// @brief Responds to a Stream command
MavlinkFTP::ErrorCode
MavlinkFTP::_workBurst(PayloadHeader *payload, uint8_t target_system_id, uint8_t target_component_id)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		PX4_DEBUG("_workBurst: no session or no fd");
		return kErrInvalidSession;
	}

	PX4_DEBUG("FTP: burst offset:%" PRIu32, payload->offset);
	// Setup for streaming sends
	_session_info.stream_download = true;
	_session_info.stream_offset = payload->offset;
	_session_info.stream_chunk_transmitted = 0;
	_session_info.stream_seq_number = payload->seq_number + 1;
	_session_info.stream_target_system_id = target_system_id;
	_session_info.stream_target_component_id = target_component_id;

	return kErrNone;
}

/// @brief Responds to a Write command
MavlinkFTP::ErrorCode
MavlinkFTP::_workWrite(PayloadHeader *payload)
{
	if (payload->session != 0 && _session_info.fd < 0) {
		PX4_DEBUG("_workWrite: no session or no fd");
		return kErrInvalidSession;
	}

	if (!_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
		// Unable to see to the specified location
		PX4_ERR("seek fail");
		return kErrFailErrno;
	}

	PX4_DEBUG("write %d bytes", payload->size);
	int bytes_written = ::write(_session_info.fd, &payload->data[0], payload->size);

	if (bytes_written < 0) {
		// Negative return indicates error other than eof
		_our_errno = errno;
		PX4_ERR("write fail %d, %s", bytes_written, strerror(_our_errno));
		return kErrFailErrno;
	}

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &bytes_written, payload->size);

	return kErrNone;
}

/// @brief Responds to a RemoveFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (!_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("unlink %s", _work_buffer1);

	if (unlink(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("unlink failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a TruncateFile command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTruncateFile(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';
	payload->size = 0;

	if (!_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

#ifdef __PX4_NUTTX

	// emulate truncate(_work_buffer1, payload->offset) by
	// copying to temp and overwrite with O_TRUNC flag (NuttX does not support truncate()).
	const char temp_file[] = PX4_STORAGEDIR"/.trunc.tmp";

	struct stat st;

	PX4_DEBUG("stat: %s", _work_buffer1);

	if (stat(_work_buffer1, &st) != 0) {
		_our_errno = errno;
		PX4_ERR("stat failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}

	if (!S_ISREG(st.st_mode)) {
		_our_errno = EISDIR;
		return kErrFailErrno;
	}

	// check perms allow us to write (not romfs)
	if (!(st.st_mode & (S_IWUSR | S_IWGRP | S_IWOTH))) {
		_our_errno = EROFS;
		return kErrFailErrno;
	}

	if (payload->offset == (unsigned)st.st_size) {
		// nothing to do
		return kErrNone;

	} else if (payload->offset == 0) {
		// 1: truncate all data
		int fd = ::open(_work_buffer1, O_TRUNC | O_WRONLY);

		if (fd < 0) {
			return kErrFailErrno;
		}

		::close(fd);
		return kErrNone;

	} else if (payload->offset > (unsigned)st.st_size) {
		// 2: extend file

		PX4_DEBUG("extend file: %s", _work_buffer1);

		int fd = ::open(_work_buffer1, O_WRONLY);

		if (fd < 0) {
			_our_errno = errno;
			PX4_ERR("open failed: %s", strerror(_our_errno));
			return kErrFailErrno;
		}

		if (lseek(fd, payload->offset - 1, SEEK_SET) < 0) {
			_our_errno = errno;
			PX4_ERR("seek failed: %s", strerror(_our_errno));
			::close(fd);
			return kErrFailErrno;
		}

		PX4_DEBUG("write 1");
		bool ok = 1 == ::write(fd, "", 1);

		if (!ok) {
			_our_errno = errno;
			PX4_ERR("write 1 failed: %s", strerror(_our_errno));
		}

		::close(fd);

		return (ok) ? kErrNone : kErrFailErrno;

	} else {
		// 3: truncate
		PX4_DEBUG("truncate file %s", _work_buffer1);

		if (_copy_file(_work_buffer1, temp_file, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (_copy_file(temp_file, _work_buffer1, payload->offset) != 0) {
			return kErrFailErrno;
		}

		if (::unlink(temp_file) != 0) {
			_our_errno = errno;
			PX4_ERR("unlink failed: %s", strerror(_our_errno));
			return kErrFailErrno;
		}

		return kErrNone;
	}

#else
	int ret = truncate(_work_buffer1, payload->offset);

	if (ret == 0) {
		return kErrNone;
	}

	return kErrFailErrno;
#endif /* __PX4_NUTTX */
}

/// @brief Responds to a Terminate command
MavlinkFTP::ErrorCode
MavlinkFTP::_workTerminate(PayloadHeader *payload)
{
	if (payload->session != 0 || _session_info.fd < 0) {
		return kErrInvalidSession;
	}

	PX4_DEBUG("work terminate: close");
	::close(_session_info.fd);
	_session_info.fd = -1;
	_session_info.stream_download = false;

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Reset command
MavlinkFTP::ErrorCode
MavlinkFTP::_workReset(PayloadHeader *payload)
{
	PX4_DEBUG("work reset: close");

	if (_session_info.fd != -1) {
		::close(_session_info.fd);
		_session_info.fd = -1;
		_session_info.stream_download = false;
	}

	payload->size = 0;

	return kErrNone;
}

/// @brief Responds to a Rename command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRename(PayloadHeader *payload)
{
	char *ptr = _data_as_cstring(payload);
	size_t oldpath_sz = strlen(ptr);

	if (oldpath_sz == payload->size) {
		// no newpath
		errno = EINVAL;
		return kErrFailErrno;
	}

	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, ptr, _work_buffer1_len - _root_dir_len);
	_work_buffer1[_work_buffer1_len - 1] = '\0'; // ensure termination

	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, ptr + oldpath_sz + 1, _work_buffer2_len - _root_dir_len);
	_work_buffer2[_work_buffer2_len - 1] = '\0'; // ensure termination

	if (!_validatePathIsWritable(_work_buffer2)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("rename from %s to %s", _work_buffer1, _work_buffer2);

	if (rename(_work_buffer1, _work_buffer2) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("rename failed: %d %s", _our_errno, strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a RemoveDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workRemoveDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (!_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("remove dir %s", _work_buffer1);

	if (rmdir(_work_buffer1) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_DEBUG("remove dir failed: %d %s", _our_errno, strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a CreateDirectory command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCreateDirectory(PayloadHeader *payload)
{
	strncpy(_work_buffer1, _root_dir, _work_buffer1_len);
	strncpy(_work_buffer1 + _root_dir_len, _data_as_cstring(payload), _work_buffer1_len - _root_dir_len);
	// ensure termination
	_work_buffer1[_work_buffer1_len - 1] = '\0';

	if (!_validatePathIsWritable(_work_buffer1)) {
		return kErrFailFileProtected;
	}

	PX4_DEBUG("create dir %s", _work_buffer1);

	if (mkdir(_work_buffer1, S_IRWXU | S_IRWXG | S_IRWXO) == 0) {
		payload->size = 0;
		return kErrNone;

	} else {
		_our_errno = errno;
		PX4_ERR("create dir failed: %s", strerror(_our_errno));
		return kErrFailErrno;
	}
}

/// @brief Responds to a CalcFileCRC32 command
MavlinkFTP::ErrorCode
MavlinkFTP::_workCalcFileCRC32(PayloadHeader *payload)
{
	uint32_t checksum = 0;
	ssize_t bytes_read;
	strncpy(_work_buffer2, _root_dir, _work_buffer2_len);
	strncpy(_work_buffer2 + _root_dir_len, _data_as_cstring(payload), _work_buffer2_len - _root_dir_len);
	// ensure termination
	_work_buffer2[_work_buffer2_len - 1] = '\0';

	int fd = ::open(_work_buffer2, O_RDONLY);

	if (fd < 0) {
		return kErrFailErrno;
	}

	do {
		bytes_read = ::read(fd, _work_buffer2, _work_buffer2_len);

		if (bytes_read < 0) {
			_our_errno = errno;
			::close(fd);
			return kErrFailErrno;
		}

		checksum = crc32part((uint8_t *)_work_buffer2, bytes_read, checksum);
	} while (bytes_read == _work_buffer2_len);

	::close(fd);

	payload->size = sizeof(uint32_t);
	std::memcpy(payload->data, &checksum, payload->size);
	return kErrNone;
}

/// @brief Guarantees that the payload data is null terminated.
///     @return Returns a pointer to the payload data as a char *
char *
MavlinkFTP::_data_as_cstring(PayloadHeader *payload)
{
	// guarantee nul termination
	if (payload->size < kMaxDataLength) {
		payload->data[payload->size] = '\0';

	} else {
		payload->data[kMaxDataLength - 1] = '\0';
	}

	// and return data
	return (char *) & (payload->data[0]);
}

/// @brief Copy file (with limited space)
int
MavlinkFTP::_copy_file(const char *src_path, const char *dst_path, size_t length)
{
	PX4_DEBUG("copy file from %s to %s", src_path, dst_path);

	int src_fd = -1, dst_fd = -1;

	src_fd = ::open(src_path, O_RDONLY);

	if (src_fd < 0) {
		return -1;
	}

	dst_fd = ::open(dst_path, O_CREAT | O_TRUNC | O_WRONLY
// POSIX requires the permissions to be supplied if O_CREAT passed
#ifdef __PX4_POSIX
			, 0666
#endif
		       );

	if (dst_fd < 0) {
		_our_errno = errno;
		::close(src_fd);
		return -1;
	}

	while (length > 0) {
		ssize_t bytes_read, bytes_written;
		size_t blen = (length > _work_buffer2_len) ? _work_buffer2_len : length;

		bytes_read = ::read(src_fd, _work_buffer2, blen);

		if (bytes_read == 0) {
			// EOF
			break;

		} else if (bytes_read < 0) {
			_our_errno = errno;
			PX4_ERR("cp: read");
			break;
		}

		bytes_written = ::write(dst_fd, _work_buffer2, bytes_read);

		if (bytes_written != bytes_read) {
			_our_errno = errno;
			PX4_ERR("cp: short write");
			break;
		}

		length -= bytes_written;
	}

	::close(src_fd);
	::close(dst_fd);

	return (length > 0) ? -1 : 0;
}

void MavlinkFTP::send()
{

	if (_work_buffer1 || _work_buffer2) {
		// free the work buffers if they are not used for a while
		if (hrt_elapsed_time(&_last_work_buffer_access) > 2_s) {
			if (_work_buffer1) {
				delete[] _work_buffer1;
				_work_buffer1 = nullptr;
			}

			if (_work_buffer2) {
				delete[] _work_buffer2;
				_work_buffer2 = nullptr;
			}
		}

	} else if (_session_info.fd != -1) {
		// close session without activity
		if (hrt_elapsed_time(&_last_work_buffer_access) > 10_s) {
			::close(_session_info.fd);
			_session_info.fd = -1;
			_session_info.stream_download = false;
			_last_reply_valid = false;
			PX4_WARN("Session was closed without activity");
		}
	}

	// Anything to stream?
	if (!_session_info.stream_download) {
		return;
	}

#ifndef MAVLINK_FTP_UNIT_TEST
	// Skip send if not enough room
	unsigned max_bytes_to_send = _mavlink->get_free_tx_buf();
	PX4_DEBUG("MavlinkFTP::send max_bytes_to_send(%u) get_free_tx_buf(%u)", max_bytes_to_send, _mavlink->get_free_tx_buf());

	if (max_bytes_to_send < get_size()) {
		return;
	}

#endif

	// Send stream packets until buffer is full

	bool more_data;

	do {
		more_data = false;

		ErrorCode error_code = kErrNone;

		mavlink_file_transfer_protocol_t ftp_msg;
		PayloadHeader *payload = reinterpret_cast<PayloadHeader *>(&ftp_msg.payload[0]);

		payload->seq_number = _session_info.stream_seq_number;
		payload->session = 0;
		payload->opcode = kRspAck;
		payload->req_opcode = kCmdBurstReadFile;
		payload->offset = _session_info.stream_offset;
		_session_info.stream_seq_number++;

		PX4_DEBUG("stream send: offset %" PRIu32, _session_info.stream_offset);

		// We have to test seek past EOF ourselves, lseek will allow seek past EOF
		if (_session_info.stream_offset >= _session_info.file_size) {
			error_code = kErrEOF;
			PX4_DEBUG("stream download: sending Nak EOF");
		}

		if (error_code == kErrNone) {
			if (lseek(_session_info.fd, payload->offset, SEEK_SET) < 0) {
				error_code = kErrFailErrno;
				PX4_WARN("stream download: seek fail");
			}
		}

		if (error_code == kErrNone) {
			int bytes_read = ::read(_session_info.fd, &payload->data[0], kMaxDataLength);

			if (bytes_read < 0) {
				// Negative return indicates error other than eof
				error_code = kErrFailErrno;
				PX4_WARN("stream download: read fail");

			} else {
				payload->size = bytes_read;
				_session_info.stream_offset += bytes_read;
				_session_info.stream_chunk_transmitted += bytes_read;
			}
		}

		if (error_code != kErrNone) {
			payload->opcode = kRspNak;
			payload->size = 1;
			uint8_t *pData = &payload->data[0];
			*pData = error_code; // Straight reference to data[0] is causing bogus gcc array subscript error

			if (error_code == kErrFailErrno) {
				payload->size = 2;
				payload->data[1] = _our_errno;
			}

			_session_info.stream_download = false;

		} else {
#ifndef MAVLINK_FTP_UNIT_TEST

			if (max_bytes_to_send < (get_size() * 2)) {
				more_data = false;

				/* perform transfers in 35K chunks - this is determined empirical */
				if (_session_info.stream_chunk_transmitted > 35000) {
					payload->burst_complete = true;
					_session_info.stream_download = false;
					_session_info.stream_chunk_transmitted = 0;
				}

			} else {
#endif
				more_data = true;
				payload->burst_complete = false;
#ifndef MAVLINK_FTP_UNIT_TEST
				max_bytes_to_send -= get_size();
			}

#endif
		}

		ftp_msg.target_system = _session_info.stream_target_system_id;
		ftp_msg.target_network = 0;
		ftp_msg.target_component = _session_info.stream_target_component_id;
		_reply(&ftp_msg);
	} while (more_data);
}

bool MavlinkFTP::_validatePathIsWritable(const char *path)
{
#ifdef __PX4_NUTTX

	// Don't allow writes to system paths as they are in RAM
	// Ideally we'd canonicalize the path (with 'realpath'), but it might not exist, so realpath() would fail.
	// The next simpler thing is to check there's no reference to a parent dir.
	if (strncmp(path, "/fs/microsd/", 12) != 0 || strstr(path, "/../") != nullptr) {
		PX4_ERR("Disallowing write to %s", path);
		return false;
	}

#endif
	return true;
}
/****************************************************************************
 *
 *   Copyright (c) 2014-2021 PX4 Development Team. All rights reserved.
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

/// @file mavlink_log_handler.h
/// @author px4dev, Gus Grubba <mavlink@grubba.com>

#include "mavlink_log_handler.h"
#include "mavlink_main.h"
#include <sys/stat.h>
#include <time.h>
#include <systemlib/err.h>

#define MOUNTPOINT PX4_STORAGEDIR

static const char *kLogRoot    = MOUNTPOINT "/log";
static const char *kLogData    = MOUNTPOINT "/logdata.txt";
static const char *kTmpData    = MOUNTPOINT "/$log$.txt";

#ifdef __PX4_NUTTX
#define PX4LOG_REGULAR_FILE DTYPE_FILE
#define PX4LOG_DIRECTORY    DTYPE_DIRECTORY
#else
#define PX4LOG_REGULAR_FILE DT_REG
#define PX4LOG_DIRECTORY    DT_DIR
#endif

//#define MAVLINK_LOG_HANDLER_VERBOSE

#ifdef MAVLINK_LOG_HANDLER_VERBOSE
#define PX4LOG_WARN(fmt, ...) warnx(fmt, ##__VA_ARGS__)
#else
#define PX4LOG_WARN(fmt, ...)
#endif

//-------------------------------------------------------------------
static bool
stat_file(const char *file, time_t *date = nullptr, uint32_t *size = nullptr)
{
	struct stat st;

	if (stat(file, &st) == 0) {
		if (date) { *date = st.st_mtime; }

		if (size) { *size = st.st_size; }

		return true;
	}

	return false;
}

//-------------------------------------------------------------------
MavlinkLogHandler::MavlinkLogHandler(Mavlink *mavlink)
	: _mavlink(mavlink)
{

}
MavlinkLogHandler::~MavlinkLogHandler()
{
	_close_and_unlink_files();
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
		_log_request_list(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
		_log_request_data(msg);
		break;

	case MAVLINK_MSG_ID_LOG_ERASE:
		_log_request_erase(msg);
		break;

	case MAVLINK_MSG_ID_LOG_REQUEST_END:
		_log_request_end(msg);
		break;
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::send()
{
	//-- An arbitrary count of max bytes in one go (one of the two below but never both)
#define MAX_BYTES_SEND 256 * 1024
	size_t count = 0;

	//-- Log Entries
	while (_current_status == LogHandlerState::Listing
	       && _mavlink->get_free_tx_buf() > MAVLINK_MSG_ID_LOG_ENTRY_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	       && count < MAX_BYTES_SEND) {
		count += _log_send_listing();
	}

	//-- Log Data
	while (_current_status == LogHandlerState::SendingData
	       && _mavlink->get_free_tx_buf() > MAVLINK_MSG_ID_LOG_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	       && count < MAX_BYTES_SEND) {
		count += _log_send_data();
	}
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_list(const mavlink_message_t *msg)
{
	mavlink_log_request_list_t request;
	mavlink_msg_log_request_list_decode(msg, &request);

	//-- Check for re-requests (data loss) or new request
	if (_current_status != LogHandlerState::Inactive) {
		//-- Is this a new request?
		if (request.start == 0) {
			_current_status = LogHandlerState::Inactive;
			_close_and_unlink_files();

		} else {
			_current_status = LogHandlerState::Idle;

		}
	}

	if (_current_status == LogHandlerState::Inactive) {
		//-- Prepare new request

		_reset_list_helper();
		_init_list_helper();
		_current_status = LogHandlerState::Idle;
	}

	if (_log_count) {
		//-- Define (and clamp) range
		_next_entry = request.start < _log_count ? request.start :
			      _log_count - 1;
		_last_entry = request.end   < _log_count ? request.end :
			      _log_count - 1;
	}

	PX4LOG_WARN("\nMavlinkLogHandler::_log_request_list: start: %d last: %d count: %d",
		    _next_entry,
		    _last_entry,
		    _log_count);
	//-- Enable streaming
	_current_status = LogHandlerState::Listing;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_data(const mavlink_message_t *msg)
{
	//-- If we haven't listed, we can't do much
	if (_current_status == LogHandlerState::Inactive) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Log request with no list requested.");
		return;
	}

	mavlink_log_request_data_t request;
	mavlink_msg_log_request_data_decode(msg, &request);

	//-- Does the requested log exist?
	if (request.id >= _log_count) {
		PX4LOG_WARN("MavlinkLogHandler::_log_request_data Requested log %" PRIu16 " but we only have %u.", request.id,
			    _log_count);
		return;
	}

	//-- If we were sending log entries, stop it
	_current_status = LogHandlerState::Idle;

	if (_current_log_index != request.id) {
		//-- Init send log dataset
		_current_log_filename[0] = 0;
		_current_log_index = request.id;
		uint32_t time_utc = 0;

		if (!_get_entry(_current_log_index, _current_log_size, time_utc,
				_current_log_filename, sizeof(_current_log_filename))) {
			PX4LOG_WARN("LogListHelper::get_entry failed.");
			return;
		}

		_open_for_transmit();
	}

	_current_log_data_offset = request.ofs;

	if (_current_log_data_offset >= _current_log_size) {
		_current_log_data_remaining = 0;

	} else {
		_current_log_data_remaining = _current_log_size - request.ofs;
	}

	if (_current_log_data_remaining > request.count) {
		_current_log_data_remaining = request.count;
	}

	//-- Enable streaming
	_current_status = LogHandlerState::SendingData;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_erase(const mavlink_message_t * /*msg*/)
{
	/*
	mavlink_log_erase_t request;
	mavlink_msg_log_erase_decode(msg, &request);
	*/
	_current_status = LogHandlerState::Inactive;
	_close_and_unlink_files();

	//-- Delete all logs
	_delete_all(kLogRoot);
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_log_request_end(const mavlink_message_t * /*msg*/)
{
	PX4LOG_WARN("MavlinkLogHandler::_log_request_end");

	_current_status = LogHandlerState::Inactive;
	_close_and_unlink_files();
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_listing()
{
	mavlink_log_entry_t response;
	uint32_t size, date;
	_get_entry(_next_entry, size, date);
	response.size         = size;
	response.time_utc     = date;
	response.id           = _next_entry;
	response.num_logs     = _log_count;
	response.last_log_num = _last_entry;
	mavlink_msg_log_entry_send_struct(_mavlink->get_channel(), &response);

	//-- If we're done listing, flag it.
	if (_next_entry == _last_entry) {
		_current_status = LogHandlerState::Idle;

	} else {
		_next_entry++;
	}

	PX4LOG_WARN("MavlinkLogHandler::_log_send_listing id: %" PRIu16 " count: %" PRIu16 " last: %" PRIu16 " size: %" PRIu32
		    " date: %" PRIu32 " status: %" PRIu32,
		    response.id,
		    response.num_logs,
		    response.last_log_num,
		    response.size,
		    response.time_utc,
		    (uint32_t)_current_status);
	return sizeof(response);
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_log_send_data()
{
	mavlink_log_data_t response;
	memset(&response, 0, sizeof(response));
	uint32_t len = _current_log_data_remaining;

	if (len > sizeof(response.data)) {
		len = sizeof(response.data);
	}

	size_t read_size = _get_log_data(len, response.data);
	response.ofs     = _current_log_data_offset;
	response.id      = _current_log_index;
	response.count   = read_size;
	mavlink_msg_log_data_send_struct(_mavlink->get_channel(), &response);
	_current_log_data_offset    += read_size;
	_current_log_data_remaining -= read_size;

	if (read_size < sizeof(response.data) || _current_log_data_remaining == 0) {
		_current_status = LogHandlerState::Idle;
	}

	return sizeof(response);
}

//-------------------------------------------------------------------
void MavlinkLogHandler::_close_and_unlink_files()
{
	if (_current_log_filep) {
		::fclose(_current_log_filep);
		_reset_list_helper();
	}

	// Remove log data files (if any)
	unlink(kLogData);
	unlink(kTmpData);
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_entry(int idx, uint32_t &size, uint32_t &date, char *filename, int filename_len)
{
	//-- Find log file in log list file created during init()
	size = 0;
	date = 0;
	bool result = false;
	//-- Open list of log files
	FILE *f = ::fopen(kLogData, "r");

	if (f) {
		//--- Find requested entry
		char line[160];
		int count = 0;

		while (fgets(line, sizeof(line), f)) {
			//-- Found our "index"
			if (count++ == idx) {
				char file[160];

				if (sscanf(line, "%" PRIu32 " %" PRIu32 " %s", &date, &size, file) == 3) {
					if (filename && filename_len > 0) {
						strncpy(filename, file, filename_len);
						filename[filename_len - 1] = 0; // ensure null-termination
					}

					result = true;
					break;
				}
			}
		}

		fclose(f);
	}

	return result;
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_open_for_transmit()
{
	if (_current_log_filep) {
		::fclose(_current_log_filep);
		_current_log_filep = nullptr;
	}

	_current_log_filep = ::fopen(_current_log_filename, "rb");

	if (!_current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::open_for_transmit Could not open %s", _current_log_filename);
		return false;
	}

	return true;
}

//-------------------------------------------------------------------
size_t
MavlinkLogHandler::_get_log_data(uint8_t len, uint8_t *buffer)
{
	if (!_current_log_filename[0]) {
		return 0;
	}

	if (!_current_log_filep) {
		PX4LOG_WARN("MavlinkLogHandler::get_log_data file not open %s", _current_log_filename);
		return 0;
	}

	long int offset = _current_log_data_offset - ftell(_current_log_filep);

	if (offset && fseek(_current_log_filep, offset, SEEK_CUR)) {
		fclose(_current_log_filep);
		_current_log_filep = nullptr;
		PX4LOG_WARN("MavlinkLogHandler::get_log_data Seek error in %s", _current_log_filename);
		return 0;
	}

	size_t result = fread(buffer, 1, len, _current_log_filep);
	return result;
}


void
MavlinkLogHandler::_reset_list_helper()
{
	_next_entry = 0;
	_last_entry = 0;
	_log_count = 0;
	_current_log_index = UINT16_MAX;
	_current_log_size = 0;
	_current_log_data_offset = 0;
	_current_log_data_remaining = 0;
	_current_log_filep = nullptr;
}

void
MavlinkLogHandler::_init_list_helper()
{
	/*

		When this helper is created, it scans the log directory
		and collects all log files found into one file for easy,
		subsequent access.
	*/

	_current_log_filename[0] = 0;

	// Remove old log data file (if any)
	unlink(kLogData);
	// Open log directory
	DIR *dp = opendir(kLogRoot);

	if (dp == nullptr) {
		// No log directory. Nothing to do.
		return;
	}

	// Create work file
	FILE *f = ::fopen(kTmpData, "w");

	if (!f) {
		PX4LOG_WARN("MavlinkLogHandler::init Error creating %s", kTmpData);
		closedir(dp);
		return;
	}

	// Scan directory and collect log files
	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		if (result->d_type == PX4LOG_DIRECTORY) {
			time_t tt = 0;
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", kLogRoot, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (_get_session_date(log_path, result->d_name, tt)) {
					_scan_logs(f, log_path, tt);
				}
			}
		}
	}

	closedir(dp);
	fclose(f);

	// Rename temp file to data file
	if (rename(kTmpData, kLogData)) {
		PX4LOG_WARN("MavlinkLogHandler::init Error renaming %s", kTmpData);
		_log_count = 0;
	}
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_session_date(const char *path, const char *dir, time_t &date)
{
	if (strlen(dir) > 4) {
		// Always try to get file time first
		if (stat_file(path, &date)) {
			// Try to prevent taking date if it's around 1970 (use the logic below instead)
			if (date > 60 * 60 * 24) {
				return true;
			}
		}

		// Convert "sess000" to 00:00 Jan 1 1970 (day per session)
		if (strncmp(dir, "sess", 4) == 0) {
			unsigned u;

			if (sscanf(&dir[4], "%u", &u) == 1) {
				date = u * 60 * 60 * 24;
				return true;
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_scan_logs(FILE *f, const char *dir, time_t &date)
{
	DIR *dp = opendir(dir);

	if (dp) {
		struct dirent *result = nullptr;

		while ((result = readdir(dp))) {
			if (result->d_type == PX4LOG_REGULAR_FILE) {
				time_t  ldate = date;
				uint32_t size = 0;
				char log_file_path[128];
				int ret = snprintf(log_file_path, sizeof(log_file_path), "%s/%s", dir, result->d_name);
				bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_file_path));

				if (path_is_ok) {
					if (_get_log_time_size(log_file_path, result->d_name, ldate, size)) {
						//-- Write result->out to list file
						fprintf(f, "%u %u %s\n", (unsigned)ldate, (unsigned)size, log_file_path);
						_log_count++;
					}
				}
			}
		}

		closedir(dp);
	}
}

//-------------------------------------------------------------------
bool
MavlinkLogHandler::_get_log_time_size(const char *path, const char *file, time_t &date, uint32_t &size)
{
	if (file && file[0]) {
		if (strstr(file, ".px4log") || strstr(file, ".ulg")) {
			// Always try to get file time first
			if (stat_file(path, &date, &size)) {
				// Try to prevent taking date if it's around 1970 (use the logic below instead)
				if (date > 60 * 60 * 24) {
					return true;
				}
			}

			// Convert "log000" to 00:00 (minute per flight in session)
			if (strncmp(file, "log", 3) == 0) {
				unsigned u;

				if (sscanf(&file[3], "%u", &u) == 1) {
					date += (u * 60);

					if (stat_file(path, nullptr, &size)) {
						return true;
					}
				}
			}
		}
	}

	return false;
}

//-------------------------------------------------------------------
void
MavlinkLogHandler::_delete_all(const char *dir)
{
	//-- Open log directory
	DIR *dp = opendir(dir);

	if (dp == nullptr) {
		return;
	}

	struct dirent *result = nullptr;

	while ((result = readdir(dp))) {
		// no more entries?
		if (result == nullptr) {
			break;
		}

		if (result->d_type == PX4LOG_DIRECTORY && result->d_name[0] != '.') {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				_delete_all(log_path); //Recursive call. TODO: consider add protection

				if (rmdir(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error removing %s", log_path);
				}
			}
		}

		if (result->d_type == PX4LOG_REGULAR_FILE) {
			char log_path[128];
			int ret = snprintf(log_path, sizeof(log_path), "%s/%s", dir, result->d_name);
			bool path_is_ok = (ret > 0) && (ret < (int)sizeof(log_path));

			if (path_is_ok) {
				if (unlink(log_path)) {
					PX4LOG_WARN("MavlinkLogHandler::delete_all Error deleting %s", log_path);
				}
			}
		}
	}

	closedir(dp);
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
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
 * @file mavlink_main.cpp
 * MAVLink 1.0 protocol implementation.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <termios.h>

#ifdef CONFIG_NET
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netutils/netlib.h>
#endif

#include <containers/LockGuard.hpp>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/systemlib/mavlink_log.h>
#include <lib/version/version.h>

#include <px4_platform_common/events.h>

#include <uORB/topics/event.h>
#include "mavlink_receiver.h"
#include "mavlink_main.h"

// Guard against MAVLink misconfiguration
#ifndef MAVLINK_CRC_EXTRA
#error MAVLINK_CRC_EXTRA has to be defined on PX4 systems
#endif

// Guard against flow control misconfiguration
#if defined (CRTSCTS) && defined (__PX4_NUTTX) && (CRTSCTS != (CRTS_IFLOW | CCTS_OFLOW))
#error The non-standard CRTSCTS define is incorrect. Fix this in the OS or replace with (CRTS_IFLOW | CCTS_OFLOW)
#endif

#ifdef CONFIG_NET
#define MAVLINK_NET_ADDED_STACK PX4_STACK_ADJUSTED(350)
#else
#define MAVLINK_NET_ADDED_STACK 0
#endif

#define FLOW_CONTROL_DISABLE_THRESHOLD 40              ///< picked so that some messages still would fit it.
#define MAX_DATA_RATE                  10000000        ///< max data rate in bytes/s
#define MAIN_LOOP_DELAY                10000           ///< 100 Hz @ 1000 bytes/s data rate

static pthread_mutex_t mavlink_module_mutex = PTHREAD_MUTEX_INITIALIZER;
events::EventBuffer *Mavlink::_event_buffer = nullptr;

Mavlink *mavlink_module_instances[MAVLINK_COMM_NUM_BUFFERS] {};

void mavlink_send_uart_bytes(mavlink_channel_t chan, const uint8_t *ch, int length) { mavlink_module_instances[chan]->send_bytes(ch, length); }
void mavlink_start_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_start(length); }
void mavlink_end_uart_send(mavlink_channel_t chan, int length) { mavlink_module_instances[chan]->send_finish(); }
mavlink_status_t *mavlink_get_channel_status(uint8_t channel) { return mavlink_module_instances[channel]->get_status(); }
mavlink_message_t *mavlink_get_channel_buffer(uint8_t channel) { return mavlink_module_instances[channel]->get_buffer(); }

static void usage();

hrt_abstime Mavlink::_first_start_time = {0};

bool Mavlink::_boot_complete = false;

Mavlink::Mavlink() :
	ModuleParams(nullptr),
	_receiver(this)
{
	// initialise parameter cache
	mavlink_update_parameters();

	// save the current system- and component ID because we don't allow them to change during operation
	int sys_id = _param_mav_sys_id.get();

	if (sys_id > 0 && sys_id < 255) {
		mavlink_system.sysid = sys_id;
	}

	int comp_id = _param_mav_comp_id.get();

	if (comp_id > 0 && comp_id < 255) {
		mavlink_system.compid = comp_id;
	}

	if (_first_start_time == 0) {
		_first_start_time = hrt_absolute_time();
	}

	// ensure topic exists, otherwise we might lose first queued commands
	if (orb_exists(ORB_ID(vehicle_command), 0) == PX4_ERROR) {
		orb_advertise_queue(ORB_ID(vehicle_command), nullptr, vehicle_command_s::ORB_QUEUE_LENGTH);
	}

	_vehicle_command_sub.subscribe();

	if (orb_exists(ORB_ID(event), 0) == PX4_ERROR) {
		orb_advertise_queue(ORB_ID(event), nullptr, event_s::ORB_QUEUE_LENGTH);
	}

	_event_sub.subscribe();
	_telemetry_status_pub.advertise();
}

Mavlink::~Mavlink()
{
	if (running()) {
		/* task wakes up every 10ms or so at the longest */
		request_stop();

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait at least 1 second (10ms * 10) */
			px4_usleep(10000);

			/* if we have given up, kill it */
			if (++i > 100) {
				PX4_ERR("mavlink didn't stop, killing task %d", _task_id);
				px4_task_delete(_task_id);
				break;
			}
		} while (running());
	}

	if (_instance_id >= 0) {
		mavlink_module_instances[_instance_id] = nullptr;
	}

	// if this instance was responsible for checking events then select a new mavlink instance
	if (check_events()) {
		check_events_disable();

		// select next available instance
		for (Mavlink *inst : mavlink_module_instances) {
			if (inst) {
				inst->check_events_enable();
				break;
			}
		}
	}

	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
	perf_free(_send_byte_error_perf);
	perf_free(_forwarding_error_perf);
}

void
Mavlink::mavlink_update_parameters()
{
	updateParams();

	int32_t proto = _param_mav_proto_ver.get();

	if (_protocol_version_switch != proto) {
		_protocol_version_switch = proto;
		set_proto_version(proto);
	}

	if (_param_mav_type.get() < 0 || _param_mav_type.get() >= MAV_TYPE_ENUM_END) {
		_param_mav_type.set(0);
		_param_mav_type.commit_no_notification();
		PX4_ERR("MAV_TYPE parameter invalid, resetting to 0.");
	}
}

bool Mavlink::set_channel()
{
	/* set channel according to instance id */
	switch (_instance_id) {
	case 0:
		_channel = MAVLINK_COMM_0;
		return true;

	case 1:
		_channel = MAVLINK_COMM_1;
		return true;

	case 2:
		_channel = MAVLINK_COMM_2;
		return true;

	case 3:
		_channel = MAVLINK_COMM_3;
		return true;
#ifdef MAVLINK_COMM_4

	case 4:
		_channel = MAVLINK_COMM_4;
		return true;
#endif
#ifdef MAVLINK_COMM_5

	case 5:
		_channel = MAVLINK_COMM_5;
		return true;
#endif
#ifdef MAVLINK_COMM_6

	case 6:
		_channel = MAVLINK_COMM_6;
		return true;
#endif

	default:
		PX4_ERR("instance ID %d is out of range", _instance_id);
		break;
	}

	return false;
}

bool
Mavlink::set_instance_id()
{
	LockGuard lg{mavlink_module_mutex};

	// instance count
	size_t inst_count = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			inst_count++;
		}
	}

	// if this is the first instance use it to check events
	if (inst_count == 0) {
		check_events_enable();
	}

	for (int instance_id = 0; instance_id < MAVLINK_COMM_NUM_BUFFERS; instance_id++) {
		if (mavlink_module_instances[instance_id] == nullptr) {
			mavlink_module_instances[instance_id] = this;
			_instance_id = instance_id;
			return true;
		}
	}

	return false;
}

void
Mavlink::set_proto_version(unsigned version)
{
	if ((version == 1 || version == 0) &&
	    ((_protocol_version_switch == 0) || (_protocol_version_switch == 1))) {
		get_status()->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
		_protocol_version = 1;

	} else if (version == 2 &&
		   ((_protocol_version_switch == 0) || (_protocol_version_switch == 2))) {
		get_status()->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
		_protocol_version = 2;
	}
}

int
Mavlink::instance_count()
{
	LockGuard lg{mavlink_module_mutex};
	size_t inst_index = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			inst_index++;
		}
	}

	return inst_index;
}

Mavlink *
Mavlink::get_instance_for_device(const char *device_name)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->_protocol == Protocol::SERIAL) && (strcmp(inst->_device_name, device_name) == 0)) {
			return inst;
		}
	}

	return nullptr;
}

#ifdef MAVLINK_UDP
Mavlink *
Mavlink::get_instance_for_network_port(unsigned long port)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->_protocol == Protocol::UDP) && (inst->_network_port == port)) {
			return inst;
		}
	}

	return nullptr;
}
#endif // MAVLINK_UDP

int
Mavlink::destroy_all_instances()
{
	PX4_INFO("waiting for instances to stop");

	unsigned iterations = 0;

	while (iterations < 1000) {
		int running = 0;

		pthread_mutex_lock(&mavlink_module_mutex);

		for (Mavlink *inst_to_del : mavlink_module_instances) {
			if (inst_to_del != nullptr) {
				if (inst_to_del->running()) {
					running++;

					// set flag to stop thread and wait for all threads to finish
					inst_to_del->request_stop();
				}
			}
		}

		pthread_mutex_unlock(&mavlink_module_mutex);

		if (running == 0) {
			break;

		} else if (iterations > 1000) {
			PX4_ERR("Couldn't stop all mavlink instances.");
			return PX4_ERROR;

		} else {
			iterations++;
			printf(".");
			fflush(stdout);
			px4_usleep(10000);
		}
	}

	LockGuard lg{mavlink_module_mutex};

	// we know all threads have exited, so it's safe to delete objects.
	for (Mavlink *inst_to_del : mavlink_module_instances) {
		delete inst_to_del;
	}

	delete _event_buffer;
	_event_buffer = nullptr;

	PX4_INFO("all instances stopped");
	return OK;
}

int
Mavlink::get_status_all_instances(bool show_streams_status)
{
	LockGuard lg{mavlink_module_mutex};
	unsigned iterations = 0;

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst != nullptr) {
			printf("\ninstance #%u:\n", iterations);

			if (show_streams_status) {
				inst->display_status_streams();

			} else {
				inst->display_status();
			}

			iterations++;
		}
	}

	/* return an error if there are no instances */
	return (iterations == 0);
}

bool
Mavlink::serial_instance_exists(const char *device_name, Mavlink *self)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		/* don't compare with itself and with non serial instances*/
		if (inst && (inst != self) && (inst->get_protocol() == Protocol::SERIAL) && !strcmp(device_name, inst->_device_name)) {
			return true;
		}
	}

	return false;
}

bool
Mavlink::component_was_seen(int system_id, int component_id, Mavlink *self)
{
	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst != self) && (inst->_receiver.component_was_seen(system_id, component_id))) {
			return true;
		}
	}

	return false;
}

void
Mavlink::forward_message(const mavlink_message_t *msg, Mavlink *self)
{
	const mavlink_msg_entry_t *meta = mavlink_get_msg_entry(msg->msgid);

	int target_system_id = 0;
	int target_component_id = 0;

	// might be nullptr if message is unknown
	if (meta) {
		// Extract target system and target component if set
		if (meta->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM) {
			target_system_id = static_cast<uint8_t>((_MAV_PAYLOAD(msg))[meta->target_system_ofs]);
		}

		if (meta->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT) {
			target_component_id = static_cast<uint8_t>((_MAV_PAYLOAD(msg))[meta->target_component_ofs]);
		}
	}

	// If it's a message only for us, we keep it
	if (target_system_id == self->get_system_id() && target_component_id == self->get_component_id()) {
		return;
	}

	// We don't forward heartbeats unless it's specifically enabled.
	if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT && !self->forward_heartbeats_enabled()) {
		return;
	}

	LockGuard lg{mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst != self) && (inst->get_forwarding_on())) {
			// Pass message only if target component was seen before
			if (inst->_receiver.component_was_seen(target_system_id, target_component_id)) {
				inst->pass_message(msg);
			}
		}
	}
}

int
Mavlink::mavlink_open_uart(const int baud, const char *uart_name, const FLOW_CONTROL_MODE flow_control)
{
#ifndef B460800
#define B460800 460800
#endif

#ifndef B500000
#define B500000 500000
#endif

#ifndef B921600
#define B921600 921600
#endif

#ifndef B1000000
#define B1000000 1000000
#endif

	/* process baud rate */
	int speed;

	switch (baud) {
	case 0:      speed = B0;      break;

	case 50:     speed = B50;     break;

	case 75:     speed = B75;     break;

	case 110:    speed = B110;    break;

	case 134:    speed = B134;    break;

	case 150:    speed = B150;    break;

	case 200:    speed = B200;    break;

	case 300:    speed = B300;    break;

	case 600:    speed = B600;    break;

	case 1200:   speed = B1200;   break;

	case 1800:   speed = B1800;   break;

	case 2400:   speed = B2400;   break;

	case 4800:   speed = B4800;   break;

	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	case 230400: speed = B230400; break;

	case 460800: speed = B460800; break;

	case 500000: speed = B500000; break;

	case 921600: speed = B921600; break;

	case 1000000: speed = B1000000; break;

#ifdef B1500000

	case 1500000: speed = B1500000; break;
#endif

#ifdef B2000000

	case 2000000: speed = B2000000; break;
#endif

#ifdef B3000000

	case 3000000: speed = B3000000; break;
#endif

	default:
		PX4_ERR("Unsupported baudrate: %d\n\tsupported examples:\n\t9600, 19200, 38400, 57600\t\n115200\n230400\n460800\n500000\n921600\n1000000\n",
			baud);
		return -EINVAL;
	}

	/* open uart */
	_uart_fd = ::open(uart_name, O_RDWR | O_NOCTTY);

	/*
	 * Return here in the iridium mode since the iridium driver does not
	 * support the subsequent function calls.
	*/
	if (_uart_fd < 0 || _mode == MAVLINK_MODE_IRIDIUM) {
		return _uart_fd;
	}

	/* Try to set baud rate */
	struct termios uart_config;
	int termios_state;

	/* Initialize the uart config */
	if ((termios_state = tcgetattr(_uart_fd, &uart_config)) < 0) {
		PX4_ERR("ERR GET CONF %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

	/* Clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	/* Set baud rate */
	if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
		PX4_ERR("ERR SET BAUD %s: %d\n", uart_name, termios_state);
		::close(_uart_fd);
		return -1;
	}

#if defined(__PX4_LINUX) || defined(__PX4_DARWIN) || defined(__PX4_CYGWIN)
	/* Put in raw mode */
	cfmakeraw(&uart_config);
#endif

	if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
		PX4_WARN("ERR SET CONF %s\n", uart_name);
		::close(_uart_fd);
		return -1;
	}

	/* setup hardware flow control */
	if (setup_flow_control(flow_control) && (flow_control != FLOW_CONTROL_AUTO)) {
		PX4_WARN("hardware flow control not supported");
	}

	return _uart_fd;
}

int
Mavlink::setup_flow_control(enum FLOW_CONTROL_MODE mode)
{
	struct termios uart_config;

	int ret = tcgetattr(_uart_fd, &uart_config);

	if (mode != FLOW_CONTROL_OFF) {
		uart_config.c_cflag |= CRTSCTS;

	} else {
		uart_config.c_cflag &= ~CRTSCTS;

	}

	ret = tcsetattr(_uart_fd, TCSANOW, &uart_config);

	if (!ret) {
		_flow_control_mode = mode;
	}

	return ret;
}

int
Mavlink::set_hil_enabled(bool hil_enabled)
{
	int ret = OK;

	/* enable HIL (only on links with sufficient bandwidth) */
	if (hil_enabled && !_hil_enabled && _datarate > 5000) {
		_hil_enabled = true;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 200.0f);

		if (_param_sys_hitl.get() == 2) {		// Simulation in Hardware enabled ?
			configure_stream("HIL_STATE_QUATERNION", 25.0f); // ground truth to display the SIH

		} else {
			configure_stream("HIL_STATE_QUATERNION", 0.0f);
		}
	}

	/* disable HIL */
	if (!hil_enabled && _hil_enabled) {
		_hil_enabled = false;
		ret = configure_stream("HIL_ACTUATOR_CONTROLS", 0.0f);

		configure_stream("HIL_STATE_QUATERNION", 0.0f);
	}

	return ret;
}

unsigned
Mavlink::get_free_tx_buf()
{
	/*
	 * Check if the OS buffer is full and disable HW
	 * flow control if it continues to be full
	 */
	int buf_free = 0;

#if defined(MAVLINK_UDP)

	// if we are using network sockets, return max length of one packet
	if (get_protocol() == Protocol::UDP) {
# if defined(__PX4_POSIX)
		return  1500 * 10; // Speed up FTP transfers
# else
		return  1500;
# endif /* defined(__PX4_POSIX) */

	} else
#endif // MAVLINK_UDP
	{

#if defined(__PX4_NUTTX)
		(void) ioctl(_uart_fd, FIONSPACE, (unsigned long)&buf_free);
#else
		// No FIONSPACE on Linux todo:use SIOCOUTQ  and queue size to emulate FIONSPACE
		//Linux cp210x does not support TIOCOUTQ
		buf_free = MAVLINK_MAX_PACKET_LEN;
#endif

		if (_flow_control_mode == FLOW_CONTROL_AUTO && buf_free < FLOW_CONTROL_DISABLE_THRESHOLD) {
			/* Disable hardware flow control in FLOW_CONTROL_AUTO mode:
			 * if no successful write since a defined time
			 * and if the last try was not the last successful write
			 */
			if (_last_write_try_time != 0 &&
			    hrt_elapsed_time(&_last_write_success_time) > 500_ms &&
			    _last_write_success_time != _last_write_try_time) {

				setup_flow_control(FLOW_CONTROL_OFF);
			}
		}
	}

	return buf_free;
}

void Mavlink::send_start(int length)
{
	pthread_mutex_lock(&_send_mutex);
	_last_write_try_time = hrt_absolute_time();

	// check if there is space in the buffer
	if (length > (int)get_free_tx_buf()) {
		// not enough space in buffer to send
		count_txerrbytes(length);

		_tstatus.tx_buffer_overruns++;

		// prevent writes
		_tx_buffer_low = true;

	} else {
		_tx_buffer_low = false;
	}
}

void Mavlink::send_finish()
{
	if (_tx_buffer_low || (_buf_fill == 0)) {
		pthread_mutex_unlock(&_send_mutex);
		return;
	}

	int ret = -1;

	// send message to UART
	if (get_protocol() == Protocol::SERIAL) {
		ret = ::write(_uart_fd, _buf, _buf_fill);
	}

#if defined(MAVLINK_UDP)

	else if (get_protocol() == Protocol::UDP) {

# if defined(CONFIG_NET)

		if (_src_addr_initialized) {
# endif // CONFIG_NET
			ret = sendto(_socket_fd, _buf, _buf_fill, 0, (struct sockaddr *)&_src_addr, sizeof(_src_addr));
# if defined(CONFIG_NET)
		}

# endif // CONFIG_NET

		if ((_mode != MAVLINK_MODE_ONBOARD) && broadcast_enabled() &&
		    (!get_client_source_initialized() || !is_gcs_connected())) {

			if (!_broadcast_address_found) {
				find_broadcast_address();
			}

			if (_broadcast_address_found && _buf_fill > 0) {

				int bret = sendto(_socket_fd, _buf, _buf_fill, 0, (struct sockaddr *)&_bcast_addr, sizeof(_bcast_addr));

				if (bret <= 0) {
					if (!_broadcast_failed_warned) {
						PX4_ERR("sending broadcast failed, errno: %d: %s", errno, strerror(errno));
						_broadcast_failed_warned = true;
					}

				} else {
					_broadcast_failed_warned = false;
				}
			}
		}
	}

#endif // MAVLINK_UDP

	if (ret == (int)_buf_fill) {
		_tstatus.tx_message_count++;
		count_txbytes(_buf_fill);
		_last_write_success_time = _last_write_try_time;

	} else {
		count_txerrbytes(_buf_fill);
	}

	_buf_fill = 0;

	pthread_mutex_unlock(&_send_mutex);
}

void Mavlink::send_bytes(const uint8_t *buf, unsigned packet_len)
{
	if (!_tx_buffer_low) {
		if (_buf_fill + packet_len < sizeof(_buf)) {
			memcpy(&_buf[_buf_fill], buf, packet_len);
			_buf_fill += packet_len;

		} else {
			perf_count(_send_byte_error_perf);
		}
	}
}

#ifdef MAVLINK_UDP
void Mavlink::find_broadcast_address()
{
	struct ifconf ifconf;
	int ret;

#if defined(__APPLE__) && defined(__MACH__) || defined(__CYGWIN__)
	// On Mac, we can't determine the required buffer
	// size in advance, so we just use what tends to work.
	ifconf.ifc_len = 1024;
#else
	// On Linux, we can determine the required size of the
	// buffer first by providing NULL to ifc_req.
	ifconf.ifc_req = nullptr;
	ifconf.ifc_len = 0;

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_WARN("getting required buffer size failed");
		return;
	}

#endif

	PX4_DEBUG("need to allocate %d bytes", ifconf.ifc_len);

	// Allocate buffer.
	ifconf.ifc_req = (struct ifreq *)(new uint8_t[ifconf.ifc_len]);

	if (ifconf.ifc_req == nullptr) {
		PX4_ERR("Could not allocate ifconf buffer");
		return;
	}

	memset(ifconf.ifc_req, 0, ifconf.ifc_len);

	ret = ioctl(_socket_fd, SIOCGIFCONF, &ifconf);

	if (ret != 0) {
		PX4_ERR("getting network config failed");
		delete[] ifconf.ifc_req;
		return;
	}

	int offset = 0;
	// Later used to point to next network interface in buffer.
	struct ifreq *cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

	// The ugly `for` construct is used because it allows to use
	// `continue` and `break`.
	for (;
	     offset < (int)ifconf.ifc_len;
#if defined(__APPLE__) && defined(__MACH__)
	     // On Mac, to get to next entry in buffer, jump by the size of
	     // the interface name size plus whatever is greater, either the
	     // sizeof sockaddr or ifr_addr.sa_len.
	     offset += IF_NAMESIZE
		       + (sizeof(struct sockaddr) > cur_ifreq->ifr_addr.sa_len ?
			  sizeof(struct sockaddr) : cur_ifreq->ifr_addr.sa_len)
#else
	     // On Linux, it's much easier to traverse the buffer, every entry
	     // has the constant length.
	     offset += sizeof(struct ifreq)
#endif
	    ) {
		// Point to next network interface in buffer.
		cur_ifreq = (struct ifreq *) & (((uint8_t *)ifconf.ifc_req)[offset]);

		PX4_DEBUG("looking at %s", cur_ifreq->ifr_name);

		// ignore loopback network
		if (strcmp(cur_ifreq->ifr_name, "lo") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo0") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo1") == 0 ||
		    strcmp(cur_ifreq->ifr_name, "lo2") == 0) {
			PX4_DEBUG("skipping loopback");
			continue;
		}

		struct in_addr &sin_addr = ((struct sockaddr_in *)&cur_ifreq->ifr_addr)->sin_addr;

		// Accept network interfaces to local network only. This means it's an IP starting with:
		// 192./172./10.
		// Also see https://tools.ietf.org/html/rfc1918#section-3

		uint8_t first_byte = sin_addr.s_addr & 0xFF;

		if (first_byte != 192 && first_byte != 172 && first_byte != 10) {
			continue;
		}

		if (!_broadcast_address_found) {
			const struct in_addr netmask_addr = query_netmask_addr(_socket_fd, *cur_ifreq);
			const struct in_addr broadcast_addr = compute_broadcast_addr(sin_addr, netmask_addr);

			if (_interface_name && strstr(cur_ifreq->ifr_name, _interface_name) == nullptr) { continue; }

			PX4_INFO("using network interface %s, IP: %s", cur_ifreq->ifr_name, inet_ntoa(sin_addr));
			PX4_INFO("with netmask: %s", inet_ntoa(netmask_addr));
			PX4_INFO("and broadcast IP: %s", inet_ntoa(broadcast_addr));

			_bcast_addr.sin_family = AF_INET;
			_bcast_addr.sin_addr = broadcast_addr;

			_broadcast_address_found = true;

		} else {
			PX4_DEBUG("ignoring additional network interface %s, IP:  %s",
				  cur_ifreq->ifr_name, inet_ntoa(sin_addr));
		}
	}

	if (_broadcast_address_found) {
		_bcast_addr.sin_port = htons(_remote_port);

		int broadcast_opt = 1;

		if (setsockopt(_socket_fd, SOL_SOCKET, SO_BROADCAST, &broadcast_opt, sizeof(broadcast_opt)) < 0) {
			PX4_WARN("setting broadcast permission failed");
		}

		_broadcast_address_not_found_warned = false;

	} else {
		if (!_broadcast_address_not_found_warned) {
			PX4_WARN("no broadcasting address found");
			_broadcast_address_not_found_warned = true;
		}
	}

	delete[] ifconf.ifc_req;
}

const in_addr Mavlink::query_netmask_addr(const int socket_fd, const ifreq &ifreq)
{
	struct ifreq netmask_ifreq {};
	strncpy(netmask_ifreq.ifr_name, ifreq.ifr_name, IF_NAMESIZE);
	ioctl(socket_fd, SIOCGIFNETMASK, &netmask_ifreq);

	return ((struct sockaddr_in *)&netmask_ifreq.ifr_addr)->sin_addr;
}

const in_addr Mavlink::compute_broadcast_addr(const in_addr &host_addr, const in_addr &netmask_addr)
{
	struct in_addr broadcast_addr;
	broadcast_addr.s_addr = ~netmask_addr.s_addr | host_addr.s_addr;

	return broadcast_addr;
}

void Mavlink::init_udp()
{
	PX4_DEBUG("Setting up UDP with port %hu", _network_port);

	_myaddr.sin_family = AF_INET;
	_myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	_myaddr.sin_port = htons(_network_port);

	if ((_socket_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		PX4_WARN("create socket failed: %s", strerror(errno));
		return;
	}

	if (bind(_socket_fd, (struct sockaddr *)&_myaddr, sizeof(_myaddr)) < 0) {
		PX4_WARN("bind failed: %s", strerror(errno));
		return;
	}

	/* set default target address, but not for onboard mode (will be set on first received packet) */
	if (!_src_addr_initialized) {
		_src_addr.sin_family = AF_INET;
		inet_aton("127.0.0.1", &_src_addr.sin_addr);
	}

	_src_addr.sin_port = htons(_remote_port);
}
#endif // MAVLINK_UDP

void
Mavlink::handle_message(const mavlink_message_t *msg)
{
	/*
	 *  NOTE: this is called from the receiver thread
	 */

	if (get_forwarding_on()) {
		/* forward any messages to other mavlink instances */
		Mavlink::forward_message(msg, this);
	}

	// Special case for gimbals that need to forward GIMBAL_DEVICE_ATTITUDE_STATUS.
	else if (msg->msgid == MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS) {
		Mavlink::forward_message(msg, this);
	}
}

void
Mavlink::send_statustext_info(const char *string)
{
	mavlink_log_info(&_mavlink_log_pub, "%s", string);
}

void
Mavlink::send_statustext_critical(const char *string)
{
	mavlink_log_critical(&_mavlink_log_pub, "%s", string);
}

void
Mavlink::send_statustext_emergency(const char *string)
{
	mavlink_log_emergency(&_mavlink_log_pub, "%s", string);
}

bool
Mavlink::send_autopilot_capabilities()
{
	uORB::Subscription status_sub{ORB_ID(vehicle_status)};
	vehicle_status_s status;

	if (status_sub.copy(&status)) {
		mavlink_autopilot_version_t msg{};

		msg.capabilities = MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_COMMAND_INT;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_BYTEWISE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FTP;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MAVLINK2;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_FENCE;
		msg.capabilities |= MAV_PROTOCOL_CAPABILITY_MISSION_RALLY;
		msg.flight_sw_version = px4_firmware_version();
		msg.middleware_sw_version = px4_firmware_version();
		msg.os_sw_version = px4_os_version();
		msg.board_version = px4_board_version();
		/* use only first 5 bytes of git hash for firmware version */
		const uint64_t fw_git_version_binary = px4_firmware_version_binary() & 0xFFFFFFFFFF000000;
		const uint64_t fw_vendor_version = px4_firmware_vendor_version() >> 8;
		constexpr size_t fw_vendor_version_length = 3;
		memcpy(&msg.flight_custom_version, &fw_git_version_binary, sizeof(msg.flight_custom_version));
		memcpy(&msg.flight_custom_version, &fw_vendor_version, fw_vendor_version_length);
		memcpy(&msg.middleware_custom_version, &fw_git_version_binary, sizeof(msg.middleware_custom_version));
		uint64_t os_git_version_binary = px4_os_version_binary();
		memcpy(&msg.os_custom_version, &os_git_version_binary, sizeof(msg.os_custom_version));
#ifdef CONFIG_CDCACM_VENDORID
		msg.vendor_id = CONFIG_CDCACM_VENDORID;
#else
		msg.vendor_id = 0;
#endif
#ifdef CONFIG_CDCACM_PRODUCTID
		msg.product_id = CONFIG_CDCACM_PRODUCTID;
#else
		msg.product_id = 0;
#endif
		uuid_uint32_t uid;
		board_get_uuid32(uid);
		msg.uid = (((uint64_t)uid[PX4_CPU_UUID_WORD32_UNIQUE_M]) << 32) | uid[PX4_CPU_UUID_WORD32_UNIQUE_H];

#ifndef BOARD_HAS_NO_UUID
		px4_guid_t px4_guid;
		board_get_px4_guid(px4_guid);
		static_assert(sizeof(px4_guid_t) == sizeof(msg.uid2), "GUID byte length mismatch");
		memcpy(&msg.uid2, &px4_guid, sizeof(msg.uid2));
#endif /* BOARD_HAS_NO_UUID */

#ifdef CONFIG_ARCH_BOARD_PX4_SITL
		// To avoid that multiple SITL instances have the same UUID, we add the mavlink
		// system ID. We subtract 1, so that the first UUID remains unchanged given the
		// default system ID is 1.
		//
		// Note that the UUID show in `ver` will still be the same for all instances.
		msg.uid += mavlink_system.sysid - 1;
		msg.uid2[0] += mavlink_system.sysid - 1;
#endif /* CONFIG_ARCH_BOARD_PX4_SITL */
		mavlink_msg_autopilot_version_send_struct(get_channel(), &msg);
		return true;
	}

	return false;
}

void
Mavlink::send_protocol_version()
{
	mavlink_protocol_version_t msg = {};

	msg.version = _protocol_version * 100;
	msg.min_version = 100;
	msg.max_version = 200;
	uint64_t mavlink_lib_git_version_binary = px4_mavlink_lib_version_binary();
	// TODO add when available
	//memcpy(&msg.spec_version_hash, &mavlink_spec_git_version_binary, sizeof(msg.spec_version_hash));
	memcpy(&msg.library_version_hash, &mavlink_lib_git_version_binary, sizeof(msg.library_version_hash));

	// Switch to MAVLink 2
	int curr_proto_ver = _protocol_version;
	set_proto_version(2);
	// Send response - if it passes through the link its fine to use MAVLink 2
	mavlink_msg_protocol_version_send_struct(get_channel(), &msg);
	// Reset to previous value
	set_proto_version(curr_proto_ver);
}

int
Mavlink::configure_stream(const char *stream_name, const float rate)
{
	PX4_DEBUG("configure_stream(%s, %.3f)", stream_name, (double)rate);

	/* calculate interval in us, -1 means unlimited stream, 0 means disabled */
	int interval = 0;

	if (rate > 0.000001f) {
		interval = (1000000.0f / rate);

	} else if (rate < 0.0f) {
		interval = -1;
	}

	for (const auto &stream : _streams) {
		if (strcmp(stream_name, stream->get_name()) == 0) {
			if (interval != 0) {
				/* set new interval */
				stream->set_interval(interval);

			} else {
				/* delete stream */
				_streams.deleteNode(stream);
				return OK; // must finish with loop after node is deleted
			}

			return OK;
		}
	}

	// search for stream with specified name in supported streams list
	// create new instance if found
	MavlinkStream *stream = create_mavlink_stream(stream_name, this);

	if (stream != nullptr) {
		stream->set_interval(interval);
		_streams.add(stream);

		return OK;
	}

	/* if we reach here, the stream list does not contain the stream */
#if defined(CONSTRAINED_FLASH) // flash constrained target's don't include all streams
	return PX4_OK;
#else
	PX4_WARN("stream %s not found", stream_name);
	return PX4_ERROR;
#endif
}

void
Mavlink::configure_stream_threadsafe(const char *stream_name, const float rate)
{
	/* orb subscription must be done from the main thread,
	 * set _subscribe_to_stream and _subscribe_to_stream_rate fields
	 * which polled in mavlink main loop */
	if (!should_exit()) {
		/* wait for previous subscription completion */
		while (_subscribe_to_stream != nullptr) {
			px4_usleep(MAIN_LOOP_DELAY / 2);
		}

		/* copy stream name */
		unsigned n = strlen(stream_name) + 1;
		char *s = new char[n];
		strcpy(s, stream_name);

		/* set subscription task */
		_subscribe_to_stream_rate = rate;
		_subscribe_to_stream = s;

		/* wait for subscription */
		do {
			px4_usleep(MAIN_LOOP_DELAY / 2);
		} while (_subscribe_to_stream != nullptr);

		delete[] s;
	}
}

void
Mavlink::pass_message(const mavlink_message_t *msg)
{
	/* size is 12 bytes plus variable payload */
	int size = MAVLINK_NUM_NON_PAYLOAD_BYTES + msg->len;
	LockGuard lg{_message_buffer_mutex};

	if (!_message_buffer.push_back(reinterpret_cast<const uint8_t *>(msg), size)) {
		perf_count(_forwarding_error_perf);
	}
}

MavlinkShell *
Mavlink::get_shell()
{
	if (!_mavlink_shell) {
		MavlinkShell *shell = new MavlinkShell();

		if (!shell) {
			PX4_ERR("Failed to allocate a shell");

		} else {
			int ret = shell->start();

			if (ret == 0) {
				_mavlink_shell = shell;

			} else {
				PX4_ERR("Failed to start shell (%i)", ret);
				delete shell;
			}
		}
	}

	return _mavlink_shell;
}

void
Mavlink::close_shell()
{
	if (_mavlink_shell) {
		delete _mavlink_shell;
		_mavlink_shell = nullptr;
	}
}

void
Mavlink::update_rate_mult()
{
	float const_rate = 0.0f;
	float rate = 0.0f;

	/* scale down rates if their theoretical bandwidth is exceeding the link bandwidth */
	for (const auto &stream : _streams) {
		if (stream->const_rate()) {
			const_rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;

		} else {
			rate += (stream->get_interval() > 0) ? stream->get_size_avg() * 1000000.0f / stream->get_interval() : 0;
		}
	}

	float mavlink_ulog_streaming_rate_inv = 1.0f;

	if (_mavlink_ulog) {
		mavlink_ulog_streaming_rate_inv = 1.0f - _mavlink_ulog->current_data_rate();
	}

	/* scale up and down as the link permits */
	float bandwidth_mult = (float)(_datarate * mavlink_ulog_streaming_rate_inv - const_rate) / rate;

	/* if we do not have flow control, limit to the set data rate */
	if (!get_flow_control_enabled()) {
		bandwidth_mult = fminf(1.0f, bandwidth_mult);
	}

	float hardware_mult = 1.0f;
	bool log_radio_timeout = false;

	pthread_mutex_lock(&_radio_status_mutex);

	// scale down if we have a TX err rate suggesting link congestion
	if ((_tstatus.tx_error_rate_avg > 0.f) && !_radio_status_critical) {
		hardware_mult = _tstatus.tx_rate_avg / (_tstatus.tx_rate_avg + _tstatus.tx_error_rate_avg);

	} else if (_radio_status_available) {

		// check for RADIO_STATUS timeout and reset
		if (hrt_elapsed_time(&_rstatus.timestamp) > (_param_mav_radio_timeout.get() * 1_s)) {
			_radio_status_available = false;
			log_radio_timeout = true;

			if (_use_software_mav_throttling) {
				_radio_status_critical = false;
				_radio_status_mult = 1.0f;
			}
		}

		hardware_mult *= _radio_status_mult;
	}

	pthread_mutex_unlock(&_radio_status_mutex);

	if (log_radio_timeout) {
		PX4_ERR("instance %d: RADIO_STATUS timeout", _instance_id);
	}

	/* pick the minimum from bandwidth mult and hardware mult as limit */
	_rate_mult = fminf(bandwidth_mult, hardware_mult);

	/* ensure the rate multiplier never drops below 5% so that something is always sent */
	_rate_mult = math::constrain(_rate_mult, 0.05f, 1.0f);
}

void
Mavlink::update_radio_status(const radio_status_s &radio_status)
{
	pthread_mutex_lock(&_radio_status_mutex);
	_rstatus = radio_status;
	_radio_status_available = true;

	if (_use_software_mav_throttling) {

		/* check hardware limits */
		_radio_status_critical = (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE);

		if (radio_status.txbuf < RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 20% */
			_radio_status_mult *= 0.80f;

		} else if (radio_status.txbuf < RADIO_BUFFER_LOW_PERCENTAGE) {
			/* this indicates link congestion, reduce rate by 2.5% */
			_radio_status_mult *= 0.975f;

		} else if (radio_status.txbuf > RADIO_BUFFER_HALF_PERCENTAGE) {
			/* this indicates spare bandwidth, increase by 2.5% */
			_radio_status_mult *= 1.025f;
		}

		/* Constrain radio status multiplier between 1% and 100% to allow recovery */
		_radio_status_mult = math::constrain(_radio_status_mult, 0.01f, 1.0f);
	}

	pthread_mutex_unlock(&_radio_status_mutex);
}

int
Mavlink::configure_streams_to_default(const char *configure_single_stream)
{
	int ret = 0;
	bool stream_configured = false;

	auto configure_stream_local =
	[&stream_configured, configure_single_stream, &ret, this](const char *stream_name, float rate) {
		if (!configure_single_stream || strcmp(configure_single_stream, stream_name) == 0) {
			int ret_local = configure_stream(stream_name, rate);

			if (ret_local != 0) {
				ret = ret_local;
			}

			stream_configured = true;
		}
	};

	const float unlimited_rate = -1.0f;

	switch (_mode) {
	case MAVLINK_MODE_NORMAL:
		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 1.0f);
		configure_stream_local("ATTITUDE", 15.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 10.0f);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("DISTANCE_SENSOR", 0.5f);
		configure_stream_local("EFI_STATUS", 2.0f);
		configure_stream_local("ESC_INFO", 1.0f);
		configure_stream_local("ESC_STATUS", 1.0f);
		configure_stream_local("ESTIMATOR_STATUS", 0.5f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS2_RAW", 1.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 0.1f);
		configure_stream_local("LOCAL_POSITION_NED", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 1.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 2.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.0f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RAW_RPM", 2.0f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 0.5f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.1f);
		configure_stream_local("WIND_COV", 0.5f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_ONBOARD:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("HIGHRES_IMU", 50.0f);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 100.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("ESC_INFO", 10.0f);
		configure_stream_local("ESC_STATUS", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_QUATERNION", 50.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("EFI_STATUS", 2.0f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 5.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GLOBAL_POSITION_INT", 50.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 10.0f);
		configure_stream_local("RAW_RPM", 5.0f);
		configure_stream_local("RC_CHANNELS", 20.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 10.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("TRAJECTORY_REPRESENTATION_WAYPOINTS", 5.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 10.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 10.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 10.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 10.0f);
		configure_stream_local("DEBUG_VECT", 10.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 10.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_GIMBAL:
		// Note: streams requiring low latency come first
		configure_stream_local("AUTOPILOT_STATE_FOR_GIMBAL_DEVICE", 20.0f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 20.0f);
		break;

	case MAVLINK_MODE_EXTVISION:
		configure_stream_local("HIGHRES_IMU", unlimited_rate);		// for VIO

	// FALLTHROUGH
	case MAVLINK_MODE_EXTVISIONMIN:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 20.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS2_RAW", 1.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.5f);
		configure_stream_local("OPTICAL_FLOW_RAD", 1.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.5f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("TRAJECTORY_REPRESENTATION_WAYPOINTS", 5.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_OSD:
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 25.0f);
		configure_stream_local("ATTITUDE_TARGET", 10.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS_RAW_INT", 1.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 0.1f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("VFR_HUD", 25.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 2.0f);
		break;

	case MAVLINK_MODE_MAGIC:

	/* fallthrough */
	case MAVLINK_MODE_CUSTOM:
		//stream nothing
		break;

	case MAVLINK_MODE_CONFIG: // USB
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("ATTITUDE", 50.0f);
		configure_stream_local("ATTITUDE_QUATERNION", 50.0f);
		configure_stream_local("ATTITUDE_TARGET", 8.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("CAMERA_IMAGE_CAPTURED", unlimited_rate);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("EFI_STATUS", 10.0f);
		configure_stream_local("ESC_INFO", 10.0f);
		configure_stream_local("ESC_STATUS", 10.0f);
		configure_stream_local("ESTIMATOR_STATUS", 5.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 2.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("GPS_STATUS", 1.0f);
		configure_stream_local("HIGHRES_IMU", 50.0f);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("HYGROMETER_SENSOR", 1.0f);
		configure_stream_local("MAG_CAL_REPORT", 1.0f);
		configure_stream_local("MANUAL_CONTROL", 5.0f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 10.0f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 10.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 1.0f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 10.0f);
		configure_stream_local("RAW_RPM", 5.0f);
		configure_stream_local("RC_CHANNELS", 10.0f);
		configure_stream_local("SCALED_IMU", 25.0f);
		configure_stream_local("SCALED_IMU2", 25.0f);
		configure_stream_local("SCALED_IMU3", 25.0f);
		configure_stream_local("SCALED_PRESSURE", 1.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 20.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_1", 20.0f);
		configure_stream_local("SYS_STATUS", 1.0f);
		configure_stream_local("SYSTEM_TIME", 1.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 20.0f);
		configure_stream_local("VIBRATION", 2.5f);
		configure_stream_local("WIND_COV", 10.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 50.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 50.0f);
		configure_stream_local("DEBUG_VECT", 50.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 50.0f);
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_IRIDIUM:
		configure_stream_local("HIGH_LATENCY2", 0.015f);
		break;

	case MAVLINK_MODE_MINIMAL:
		configure_stream_local("ALTITUDE", 0.5f);
		configure_stream_local("ATTITUDE", 10.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 0.1f);
		configure_stream_local("GLOBAL_POSITION_INT", 5.0f);
		configure_stream_local("GPS_RAW_INT", 0.5f);
		configure_stream_local("HOME_POSITION", 0.1f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
		configure_stream_local("RC_CHANNELS", 0.5f);
		configure_stream_local("SYS_STATUS", 0.1f);
		configure_stream_local("VFR_HUD", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("LINK_NODE_STATUS", 1.0f);
#endif // !CONSTRAINED_FLASH

		break;

	case MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH:
		// Note: streams requiring low latency come first
		configure_stream_local("TIMESYNC", 10.0f);
		configure_stream_local("CAMERA_TRIGGER", unlimited_rate);
		configure_stream_local("LOCAL_POSITION_NED", 30.0f);
		configure_stream_local("ATTITUDE", 20.0f);
		configure_stream_local("ALTITUDE", 10.0f);
		configure_stream_local("DISTANCE_SENSOR", 10.0f);
		configure_stream_local("MOUNT_ORIENTATION", 10.0f);
		configure_stream_local("OBSTACLE_DISTANCE", 10.0f);
		configure_stream_local("ODOMETRY", 30.0f);
		configure_stream_local("GIMBAL_DEVICE_ATTITUDE_STATUS", 1.0f);
		configure_stream_local("GIMBAL_MANAGER_STATUS", 0.5f);
		configure_stream_local("GIMBAL_DEVICE_SET_ATTITUDE", 5.0f);
		configure_stream_local("ESC_INFO", 1.0f);
		configure_stream_local("ESC_STATUS", 5.0f);

		configure_stream_local("ADSB_VEHICLE", unlimited_rate);
		configure_stream_local("ATTITUDE_TARGET", 2.0f);
		configure_stream_local("BATTERY_STATUS", 0.5f);
		configure_stream_local("COLLISION", unlimited_rate);
		configure_stream_local("ESTIMATOR_STATUS", 1.0f);
		configure_stream_local("EXTENDED_SYS_STATE", 1.0f);
		configure_stream_local("GLOBAL_POSITION_INT", 10.0f);
		configure_stream_local("GPS_GLOBAL_ORIGIN", 1.0f);
		configure_stream_local("GPS2_RAW", unlimited_rate);
		configure_stream_local("GPS_RAW_INT", unlimited_rate);
		configure_stream_local("HOME_POSITION", 0.5f);
		configure_stream_local("NAV_CONTROLLER_OUTPUT", 1.5f);
		configure_stream_local("OPEN_DRONE_ID_LOCATION", 1.f);
		configure_stream_local("OPEN_DRONE_ID_SYSTEM", 1.f);
		configure_stream_local("OPTICAL_FLOW_RAD", 1.0f);
		configure_stream_local("ORBIT_EXECUTION_STATUS", 5.0f);
		configure_stream_local("PING", 0.1f);
		configure_stream_local("POSITION_TARGET_GLOBAL_INT", 1.5f);
		configure_stream_local("POSITION_TARGET_LOCAL_NED", 1.5f);
		configure_stream_local("RC_CHANNELS", 5.0f);
		configure_stream_local("SERVO_OUTPUT_RAW_0", 1.0f);
		configure_stream_local("SYS_STATUS", 5.0f);
		configure_stream_local("TIME_ESTIMATE_TO_TARGET", 1.0f);
		configure_stream_local("TRAJECTORY_REPRESENTATION_WAYPOINTS", 5.0f);
		configure_stream_local("UTM_GLOBAL_POSITION", 1.0f);
		configure_stream_local("VFR_HUD", 4.0f);
		configure_stream_local("VIBRATION", 0.5f);
		configure_stream_local("WIND_COV", 1.0f);

#if !defined(CONSTRAINED_FLASH)
		configure_stream_local("DEBUG", 1.0f);
		configure_stream_local("DEBUG_FLOAT_ARRAY", 1.0f);
		configure_stream_local("DEBUG_VECT", 1.0f);
		configure_stream_local("NAMED_VALUE_FLOAT", 1.0f);
#endif // !CONSTRAINED_FLASH
		break;

	case MAVLINK_MODE_UAVIONIX:
		configure_stream_local("UAVIONIX_ADSB_OUT_CFG", 0.1f);
		configure_stream_local("UAVIONIX_ADSB_OUT_DYNAMIC", 5.0f);
		break;

	default:
		ret = -1;
		break;
	}

	if (configure_single_stream && !stream_configured && strcmp(configure_single_stream, "HEARTBEAT") != 0) {
		// stream was not found, assume it is disabled by default
		return configure_stream(configure_single_stream, 0.0f);
	}

	return ret;
}

int
Mavlink::task_main(int argc, char *argv[])
{
	// If stdin, stdout and/or stderr file descriptors (0, 1, 2)
	// are not open when mavlink module starts (as might be the case for USB auto-start),
	// use default /dev/null so that these numbers are not used by other other files.
	if (fcntl(0, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_RDONLY);

		if (tmp != 0) {
			dup2(tmp, 0);
			close(tmp);
		}

	}

	if (fcntl(1, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_WRONLY);

		if (tmp != 1) {
			dup2(tmp, 1);
			close(tmp);
		}
	}

	if (fcntl(2, F_GETFD) == -1) {
		int tmp = open("/dev/null", O_WRONLY);

		if (tmp != 2) {
			dup2(tmp, 2);
			close(tmp);
		}
	}

	int ch;
	_baudrate = 57600;
	_datarate = 0;
	_mode = MAVLINK_MODE_COUNT;
	FLOW_CONTROL_MODE _flow_control = FLOW_CONTROL_AUTO;

	_interface_name = nullptr;

	// We don't care about the name and verb at this point.
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	int temp_int_arg;
#endif

	while ((ch = px4_getopt(argc, argv, "b:r:d:n:u:o:m:t:c:fswxzZp", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			if (px4_get_parameter_value(myoptarg, _baudrate) != 0) {
				PX4_ERR("baudrate parsing failed");
				err_flag = true;
			}

			if (_baudrate < 9600 || _baudrate > 3000000) {
				PX4_ERR("invalid baud rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'r':
			if (px4_get_parameter_value(myoptarg, _datarate) != 0) {
				PX4_ERR("datarate parsing failed");
				err_flag = true;
			}

			if (_datarate > MAX_DATA_RATE) {
				PX4_ERR("invalid data rate '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'd':
			_device_name = myoptarg;
			set_protocol(Protocol::SERIAL);

			if (access(_device_name, F_OK) == -1) {
				PX4_ERR("Device %s does not exist", _device_name);
				err_flag = true;
			}

			break;

		case 'n':
			_interface_name = myoptarg;
			break;

#if defined(MAVLINK_UDP)

		case 'u':
			if (px4_get_parameter_value(myoptarg, temp_int_arg) != 0) {
				PX4_ERR("invalid data udp_port");
				err_flag = true;

			} else {
				_network_port = temp_int_arg;
				set_protocol(Protocol::UDP);
			}

			break;

		case 'o':
			if (px4_get_parameter_value(myoptarg, temp_int_arg) != 0) {
				PX4_ERR("invalid remote udp_port");
				err_flag = true;

			} else {
				_remote_port = temp_int_arg;
				set_protocol(Protocol::UDP);
			}

			break;

		case 't':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;

		case 'p':
			_mav_broadcast = BROADCAST_MODE_ON;
			break;

#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)

		// multicast
		case 'c':
			_src_addr.sin_family = AF_INET;

			if (inet_aton(myoptarg, &_src_addr.sin_addr)) {
				_src_addr_initialized = true;
				_mav_broadcast = BROADCAST_MODE_MULTICAST;

			} else {
				PX4_ERR("invalid partner ip '%s'", myoptarg);
				err_flag = true;
			}

			break;
#else

		case 'c':
			PX4_ERR("Multicast option is not supported on this platform");
			err_flag = true;
			break;
#endif
#else

		case 'p':
		case 'u':
		case 'o':
		case 't':
			PX4_ERR("UDP options not supported on this platform");
			err_flag = true;
			break;
#endif

//		case 'e':
//			_mavlink_link_termination_allowed = true;
//			break;

		case 'm': {

				int mode;

				if (px4_get_parameter_value(myoptarg, mode) == 0) {
					if (mode >= 0 && mode < (int)MAVLINK_MODE_COUNT) {
						_mode = (MAVLINK_MODE)mode;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}

				} else {
					if (strcmp(myoptarg, "custom") == 0) {
						_mode = MAVLINK_MODE_CUSTOM;

					} else if (strcmp(myoptarg, "camera") == 0) {
						// left in here for compatibility
						_mode = MAVLINK_MODE_ONBOARD;

					} else if (strcmp(myoptarg, "onboard") == 0) {
						_mode = MAVLINK_MODE_ONBOARD;

					} else if (strcmp(myoptarg, "osd") == 0) {
						_mode = MAVLINK_MODE_OSD;

					} else if (strcmp(myoptarg, "magic") == 0) {
						_mode = MAVLINK_MODE_MAGIC;

					} else if (strcmp(myoptarg, "config") == 0) {
						_mode = MAVLINK_MODE_CONFIG;

					} else if (strcmp(myoptarg, "iridium") == 0) {
						_mode = MAVLINK_MODE_IRIDIUM;
						set_telemetry_status_type(telemetry_status_s::LINK_TYPE_IRIDIUM);

					} else if (strcmp(myoptarg, "minimal") == 0) {
						_mode = MAVLINK_MODE_MINIMAL;

					} else if (strcmp(myoptarg, "extvision") == 0) {
						_mode = MAVLINK_MODE_EXTVISION;

					} else if (strcmp(myoptarg, "extvisionmin") == 0) {
						_mode = MAVLINK_MODE_EXTVISIONMIN;

					} else if (strcmp(myoptarg, "gimbal") == 0) {
						_mode = MAVLINK_MODE_GIMBAL;

					} else if (strcmp(myoptarg, "onboard_low_bandwidth") == 0) {
						_mode = MAVLINK_MODE_ONBOARD_LOW_BANDWIDTH;

					} else if (strcmp(myoptarg, "uavionix") == 0) {
						_mode = MAVLINK_MODE_UAVIONIX;

					} else {
						PX4_ERR("invalid mode");
						err_flag = true;
					}
				}

				break;
			}

		case 'f':
			_forwarding_on = true;
			break;

		case 's':
			_use_software_mav_throttling = true;
			break;

		case 'w':
			_wait_to_transmit = true;
			break;

		case 'x':
			_ftp_on = true;
			break;

		case 'z':
			_flow_control = FLOW_CONTROL_ON;
			break;

		case 'Z':
			_flow_control = FLOW_CONTROL_OFF;
			break;

		default:
			err_flag = true;
			break;
		}
	}

	if (err_flag) {
		usage();
		return PX4_ERROR;
	}

	/* USB serial is indicated by /dev/ttyACMx */
	if (strncmp(_device_name, "/dev/ttyACM", 11) == 0) {
		if (_datarate == 0) {
			_datarate = 100000;
		}

		/* USB has no baudrate, but use a magic number for 'fast' */
		_baudrate = 2000000;

		if (_mode == MAVLINK_MODE_COUNT) {
			_mode = MAVLINK_MODE_CONFIG;
		}

		_ftp_on = true;
		_is_usb_uart = true;

		// Always forward messages to/from the USB instance.
		_forwarding_on = true;

		set_telemetry_status_type(telemetry_status_s::LINK_TYPE_USB);
	}

	if (_mode == MAVLINK_MODE_COUNT) {
		_mode = MAVLINK_MODE_NORMAL;
	}

	if (_datarate == 0) {
		/* convert bits to bytes and use 1/2 of bandwidth by default */
		_datarate = _baudrate / 20;
	}

	if (_datarate > MAX_DATA_RATE) {
		_datarate = MAX_DATA_RATE;
	}

	if (get_protocol() == Protocol::SERIAL) {
		if (Mavlink::serial_instance_exists(_device_name, this)) {
			PX4_ERR("%s already running", _device_name);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on %s @ %dB",
			 mavlink_mode_str(_mode), _datarate, _device_name, _baudrate);

		/* flush stdout in case MAVLink is about to take it over */
		fflush(stdout);
	}

#if defined(MAVLINK_UDP)

	else if (get_protocol() == Protocol::UDP) {
		if (Mavlink::get_instance_for_network_port(_network_port) != nullptr) {
			PX4_ERR("port %hu already occupied", _network_port);
			return PX4_ERROR;
		}

		PX4_INFO("mode: %s, data rate: %d B/s on udp port %hu remote port %hu",
			 mavlink_mode_str(_mode), _datarate, _network_port, _remote_port);
	}

#endif // MAVLINK_UDP

	if (set_instance_id()) {
		if (!set_channel()) {
			PX4_ERR("set channel failed");
			return PX4_ERROR;
		}

		// set thread name
		char thread_name[13];
		snprintf(thread_name, sizeof(thread_name), "mavlink_if%d", get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());

	} else {
		PX4_ERR("no instances available");
		return PX4_ERROR;
	}

	pthread_mutex_init(&_message_buffer_mutex, nullptr);
	pthread_mutex_init(&_send_mutex, nullptr);
	pthread_mutex_init(&_radio_status_mutex, nullptr);

	/* if we are passing on mavlink messages, we need to prepare a buffer for this instance */
	if (get_forwarding_on()) {
		/* initialize message buffer if multiplexing is on.
		 * make space for two messages plus off-by-one space as we use the empty element
		 * marker ring buffer approach.
		 */
		LockGuard lg{_message_buffer_mutex};

		if (!_message_buffer.allocate(2 * sizeof(mavlink_message_t) + 1)) {
			PX4_ERR("msg buf alloc fail");
			return PX4_ERROR;
		}
	}

	/* Activate sending the data by default (for the IRIDIUM mode it will be disabled after the first round of packages is sent)*/
	_transmitting_enabled = true;
	_transmitting_enabled_commanded = true;

	if (_mode == MAVLINK_MODE_IRIDIUM) {
		_transmitting_enabled_commanded = false;
	}

	/* add default streams depending on mode */
	if (_mode != MAVLINK_MODE_IRIDIUM) {

		/* HEARTBEAT is constant rate stream, rate never adjusted */
		configure_stream("HEARTBEAT", 1.0f);

		/* STATUSTEXT stream */
		configure_stream("STATUSTEXT", 20.0f);

		/* COMMAND_LONG stream: use unlimited rate to send all commands */
		configure_stream("COMMAND_LONG");

	}

	if (configure_streams_to_default() != 0) {
		PX4_ERR("configure_streams_to_default() failed");
	}

	/* set main loop delay depending on data rate to minimize CPU overhead */
	_main_loop_delay = (MAIN_LOOP_DELAY * 1000) / _datarate;

	/* hard limit to 1000 Hz at max */
	if (_main_loop_delay < MAVLINK_MIN_INTERVAL) {
		_main_loop_delay = MAVLINK_MIN_INTERVAL;
	}

	/* hard limit to 100 Hz at least */
	if (_main_loop_delay > MAVLINK_MAX_INTERVAL) {
		_main_loop_delay = MAVLINK_MAX_INTERVAL;
	}

	/* open the UART device after setting the instance, as it might block */
	if (get_protocol() == Protocol::SERIAL) {
		_uart_fd = mavlink_open_uart(_baudrate, _device_name, _flow_control);

		if (_uart_fd < 0) {
			PX4_ERR("could not open %s", _device_name);
			return PX4_ERROR;
		}
	}

#if defined(MAVLINK_UDP)

	/* init socket if necessary */
	if (get_protocol() == Protocol::UDP) {
		init_udp();
	}

#endif // MAVLINK_UDP

	_task_id = px4_getpid();

	/* if the protocol is serial, we send the system version blindly */
	if (get_protocol() == Protocol::SERIAL) {
		send_autopilot_capabilities();
	}

	_receiver.start();

	uint16_t event_sequence_offset = 0; // offset to account for skipped events, not sent via MAVLink

	_mavlink_start_time = hrt_absolute_time();

	_task_running.store(true);

	while (!should_exit()) {
		/* main loop */
		px4_usleep(_main_loop_delay);

		if (!should_transmit()) {
			check_requested_subscriptions();
			continue;
		}

		perf_count(_loop_interval_perf);
		perf_begin(_loop_perf);

		const hrt_abstime t = hrt_absolute_time();

		update_rate_mult();

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			mavlink_update_parameters();
		}

		configure_sik_radio();

		if (_vehicle_status_sub.updated()) {
			vehicle_status_s vehicle_status;

			if (_vehicle_status_sub.copy(&vehicle_status)) {
				/* switch HIL mode if required */
				set_hil_enabled(vehicle_status.hil_state == vehicle_status_s::HIL_STATE_ON);

				if (_mode == MAVLINK_MODE_IRIDIUM) {

					if (_transmitting_enabled && vehicle_status.high_latency_data_link_lost &&
					    !_transmitting_enabled_commanded && _first_heartbeat_sent) {

						_transmitting_enabled = false;
						mavlink_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s\t", _device_name);
						events::send<int8_t>(events::ID("mavlink_iridium_disable"), events::Log::Info,
								     "Disabling transmitting with IRIDIUM mavlink on instance {1}", _instance_id);

					} else if (!_transmitting_enabled && !vehicle_status.high_latency_data_link_lost) {
						_transmitting_enabled = true;
						mavlink_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s\t", _device_name);
						events::send<int8_t>(events::ID("mavlink_iridium_enable"), events::Log::Info,
								     "Enabling transmitting with IRIDIUM mavlink on instance {1}", _instance_id);
					}
				}
			}
		}


		// MAVLINK_MODE_IRIDIUM: handle VEHICLE_CMD_CONTROL_HIGH_LATENCY
		if (_mode == MAVLINK_MODE_IRIDIUM) {
			int vehicle_command_updates = 0;

			while (_vehicle_command_sub.updated() && (vehicle_command_updates < vehicle_command_s::ORB_QUEUE_LENGTH)) {
				vehicle_command_updates++;
				const unsigned last_generation = _vehicle_command_sub.get_last_generation();
				vehicle_command_s vehicle_cmd;

				if (_vehicle_command_sub.update(&vehicle_cmd)) {
					if (_vehicle_command_sub.get_last_generation() != last_generation + 1) {
						PX4_ERR("vehicle_command lost, generation %u -> %u", last_generation, _vehicle_command_sub.get_last_generation());
					}

					if ((vehicle_cmd.command == vehicle_command_s::VEHICLE_CMD_CONTROL_HIGH_LATENCY) &&
					    _mode == MAVLINK_MODE_IRIDIUM) {

						if (vehicle_cmd.param1 > 0.5f) {
							if (!_transmitting_enabled) {
								mavlink_log_info(&_mavlink_log_pub, "Enable transmitting with IRIDIUM mavlink on device %s by command\t",
										 _device_name);
								events::send<int8_t>(events::ID("mavlink_iridium_enable_cmd"), events::Log::Info,
										     "Enabling transmitting with IRIDIUM mavlink on instance {1} by command", _instance_id);
							}

							_transmitting_enabled = true;
							_transmitting_enabled_commanded = true;

						} else {
							if (_transmitting_enabled) {
								mavlink_log_info(&_mavlink_log_pub, "Disable transmitting with IRIDIUM mavlink on device %s by command\t",
										 _device_name);
								events::send<int8_t>(events::ID("mavlink_iridium_disable_cmd"), events::Log::Info,
										     "Disabling transmitting with IRIDIUM mavlink on instance {1} by command", _instance_id);
							}

							_transmitting_enabled = false;
							_transmitting_enabled_commanded = false;
						}

						// send positive command ack
						vehicle_command_ack_s command_ack{};
						command_ack.command = vehicle_cmd.command;
						command_ack.result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
						command_ack.from_external = !vehicle_cmd.from_external;
						command_ack.target_system = vehicle_cmd.source_system;
						command_ack.target_component = vehicle_cmd.source_component;
						command_ack.timestamp = vehicle_cmd.timestamp;
						_vehicle_command_ack_pub.publish(command_ack);
					}
				}
			}
		}

		/* send command ACK */
		bool cmd_logging_start_acknowledgement = false;
		bool cmd_logging_stop_acknowledgement = false;

		if (_vehicle_command_ack_sub.updated()) {
			static constexpr size_t COMMAND_ACK_TOTAL_LEN = MAVLINK_MSG_ID_COMMAND_ACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;

			while ((get_free_tx_buf() >= COMMAND_ACK_TOTAL_LEN) && _vehicle_command_ack_sub.updated()) {
				vehicle_command_ack_s command_ack;
				const unsigned last_generation = _vehicle_command_ack_sub.get_last_generation();

				if (_vehicle_command_ack_sub.update(&command_ack)) {
					if (_vehicle_command_ack_sub.get_last_generation() != last_generation + 1) {
						PX4_ERR("vehicle_command_ack lost, generation %u -> %u", last_generation,
							_vehicle_command_ack_sub.get_last_generation());
					}

					const bool is_target_known = _receiver.component_was_seen(command_ack.target_system, command_ack.target_component);

					if (!command_ack.from_external
					    && command_ack.command < vehicle_command_s::VEHICLE_CMD_PX4_INTERNAL_START
					    && is_target_known) {

						mavlink_command_ack_t msg{};
						msg.result = command_ack.result;
						msg.command = command_ack.command;
						msg.progress = command_ack.result_param1;
						msg.result_param2 = command_ack.result_param2;
						msg.target_system = command_ack.target_system;
						msg.target_component = command_ack.target_component;

						mavlink_msg_command_ack_send_struct(get_channel(), &msg);

						if (command_ack.command == vehicle_command_s::VEHICLE_CMD_LOGGING_START) {
							cmd_logging_start_acknowledgement = true;

						} else if (command_ack.command == vehicle_command_s::VEHICLE_CMD_LOGGING_STOP
							   && command_ack.result == vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED) {
							cmd_logging_stop_acknowledgement = true;
						}
					}
				}
			}
		}

		// For legacy gimbals using the mavlink gimbal v1 protocol, we need to send out commands.
		// We don't care about acks for these though.
		if (_gimbal_v1_command_sub.updated()) {
			vehicle_command_s cmd;
			_gimbal_v1_command_sub.copy(&cmd);

			// FIXME: filter for target system/component

			mavlink_command_long_t msg{};
			msg.param1 = cmd.param1;
			msg.param2 = cmd.param2;
			msg.param3 = cmd.param3;
			msg.param4 = cmd.param4;
			msg.param5 = cmd.param5;
			msg.param6 = cmd.param6;
			msg.param7 = cmd.param7;
			msg.command = cmd.command;
			msg.target_system = cmd.target_system;
			msg.target_component = cmd.target_component;
			msg.confirmation = 0;

			mavlink_msg_command_long_send_struct(get_channel(), &msg);
		}

		/* check for shell output */
		if (_mavlink_shell && _mavlink_shell->available() > 0) {
			if (get_free_tx_buf() >= MAVLINK_MSG_ID_SERIAL_CONTROL_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES) {
				mavlink_serial_control_t msg;
				msg.baudrate = 0;
				msg.flags = SERIAL_CONTROL_FLAG_REPLY;
				msg.timeout = 0;
				msg.device = SERIAL_CONTROL_DEV_SHELL;
				msg.count = _mavlink_shell->read(msg.data, sizeof(msg.data));
				msg.target_system = _mavlink_shell->targetSysid();
				msg.target_component = _mavlink_shell->targetCompid();
				mavlink_msg_serial_control_send_struct(get_channel(), &msg);
			}
		}

		check_requested_subscriptions();

		/* update streams */
		for (const auto &stream : _streams) {
			stream->update(t);

			if (!_first_heartbeat_sent) {
				if (_mode == MAVLINK_MODE_IRIDIUM) {
					if (stream->get_id() == MAVLINK_MSG_ID_HIGH_LATENCY2) {
						_first_heartbeat_sent = stream->first_message_sent();
					}

				} else {
					if (stream->get_id() == MAVLINK_MSG_ID_HEARTBEAT) {
						_first_heartbeat_sent = stream->first_message_sent();
					}
				}
			}
		}

		/* check for ulog streaming messages */
		if (_mavlink_ulog) {
			if (cmd_logging_stop_acknowledgement) {
				_mavlink_ulog->stop();
				_mavlink_ulog = nullptr;

			} else {
				if (cmd_logging_start_acknowledgement) {
					_mavlink_ulog->start_ack_received();
				}

				int ret = _mavlink_ulog->handle_update(get_channel());

				if (ret < 0) { //abort the streaming on error
					if (ret != -1) {
						PX4_WARN("mavlink ulog stream update failed, stopping (%i)", ret);
					}

					_mavlink_ulog->stop();
					_mavlink_ulog = nullptr;
				}
			}
		}

		/* handle new events */
		if (check_events()) {
			if (_event_sub.updated()) {
				LockGuard lg{mavlink_module_mutex};

				event_s orb_event;

				while (_event_sub.update(&orb_event)) {
					if (events::externalLogLevel(orb_event.log_levels) == events::LogLevel::Disabled) {
						++event_sequence_offset; // skip this event

					} else {
						events::Event e;
						e.id = orb_event.id;
						e.timestamp_ms = orb_event.timestamp / 1000;
						e.sequence = orb_event.event_sequence - event_sequence_offset;
						e.log_levels = orb_event.log_levels;
						static_assert(sizeof(e.arguments) == sizeof(orb_event.arguments),
							      "uorb message event: arguments size mismatch");
						memcpy(e.arguments, orb_event.arguments, sizeof(orb_event.arguments));
						_event_buffer->insert_event(e);
					}
				}
			}
		}

		_events.update(t);

		/* pass messages from other instances */
		if (get_forwarding_on()) {

			mavlink_message_t msg;
			size_t available_bytes;
			{
				// We only send one message at a time, not to put too much strain on a
				// link from forwarded messages.
				LockGuard lg{_message_buffer_mutex};
				available_bytes = _message_buffer.pop_front(reinterpret_cast<uint8_t *>(&msg), sizeof(msg));
				// We need to make sure to release the lock here before sending the
				// bytes out via IP or UART which could potentially take longer.
			}

			if (available_bytes > 0) {
				resend_message(&msg);
			}
		}

		/* update TX/RX rates*/
		if (t > _bytes_timestamp + 1_s) {
			if (_bytes_timestamp != 0) {
				const float dt = (t - _bytes_timestamp) * 1e-6f;

				_tstatus.tx_rate_avg = _bytes_tx / dt;
				_tstatus.tx_error_rate_avg = _bytes_txerr / dt;
				_tstatus.rx_rate_avg = _bytes_rx / dt;

				_bytes_tx = 0;
				_bytes_txerr = 0;
				_bytes_rx = 0;
			}

			_bytes_timestamp = t;
		}

		// publish status at 1 Hz, or sooner if HEARTBEAT has updated
		if ((hrt_elapsed_time(&_tstatus.timestamp) >= 1_s) || _tstatus_updated) {
			publish_telemetry_status();
		}

		perf_end(_loop_perf);
	}

	_receiver.stop();

	delete _subscribe_to_stream;
	_subscribe_to_stream = nullptr;

	/* delete streams */
	_streams.clear();

	if (_uart_fd >= 0) {
		/* discard all pending data, as close() might block otherwise on NuttX with flow control enabled */
		tcflush(_uart_fd, TCIOFLUSH);
		/* close UART */
		::close(_uart_fd);
	}

	if (_socket_fd >= 0) {
		close(_socket_fd);
		_socket_fd = -1;
	}

	if (_mavlink_ulog) {
		_mavlink_ulog->stop();
		_mavlink_ulog = nullptr;
	}

	pthread_mutex_destroy(&_send_mutex);
	pthread_mutex_destroy(&_radio_status_mutex);
	pthread_mutex_destroy(&_message_buffer_mutex);

	PX4_INFO("exiting channel %i", (int)_channel);

	_task_running.store(false);

	return OK;
}

void Mavlink::check_requested_subscriptions()
{
	if (_subscribe_to_stream != nullptr) {
		if (_subscribe_to_stream_rate < -1.5f) {
			if (configure_streams_to_default(_subscribe_to_stream) == 0) {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s set to default rate", _subscribe_to_stream, _device_name);
				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu set to default rate", _subscribe_to_stream, _network_port);
				}

#endif // MAVLINK_UDP

			} else {
				PX4_ERR("setting stream %s to default failed", _subscribe_to_stream);
			}

		} else if (configure_stream(_subscribe_to_stream, _subscribe_to_stream_rate) == 0) {
			if (fabsf(_subscribe_to_stream_rate) > 0.00001f) {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s enabled with rate %.1f Hz", _subscribe_to_stream, _device_name,
						  (double)_subscribe_to_stream_rate);

				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu enabled with rate %.1f Hz", _subscribe_to_stream, _network_port,
						  (double)_subscribe_to_stream_rate);
				}

#endif // MAVLINK_UDP

			} else {
				if (get_protocol() == Protocol::SERIAL) {
					PX4_DEBUG("stream %s on device %s disabled", _subscribe_to_stream, _device_name);

				}

#if defined(MAVLINK_UDP)

				else if (get_protocol() == Protocol::UDP) {
					PX4_DEBUG("stream %s on UDP port %hu disabled", _subscribe_to_stream, _network_port);
				}

#endif // MAVLINK_UDP
			}

		} else {
			if (get_protocol() == Protocol::SERIAL) {
				PX4_ERR("stream %s on device %s not found", _subscribe_to_stream, _device_name);

			}

#if defined(MAVLINK_UDP)

			else if (get_protocol() == Protocol::UDP) {
				PX4_ERR("stream %s on UDP port %hu not found", _subscribe_to_stream, _network_port);
			}

#endif // MAVLINK_UDP
		}

		_subscribe_to_stream = nullptr;
	}
}

void Mavlink::publish_telemetry_status()
{
	// many fields are populated in place

	_tstatus.mode = _mode;
	_tstatus.data_rate = _datarate;
	_tstatus.rate_multiplier = _rate_mult;
	_tstatus.flow_control = get_flow_control_enabled();
	_tstatus.ftp = ftp_enabled();
	_tstatus.forwarding = get_forwarding_on();
	_tstatus.mavlink_v2 = (_protocol_version == 2);

	_tstatus.streams = _streams.size();

	// telemetry_status is also updated from the receiver thread, but never the same fields
	_tstatus.timestamp = hrt_absolute_time();
	_telemetry_status_pub.publish(_tstatus);
	_tstatus_updated = false;
}

void Mavlink::configure_sik_radio()
{
	/* radio config check */
	if (_uart_fd >= 0 && _param_sik_radio_id.get() != 0) {
		/* request to configure radio and radio is present */
		FILE *fs = fdopen(_uart_fd, "w");

		if (fs) {
			/* switch to AT command mode */
			px4_usleep(1200000);
			fprintf(fs, "+++");
			fflush(fs);
			px4_usleep(1200000);

			if (_param_sik_radio_id.get() > 0) {
				/* set channel */
				fprintf(fs, "ATS3=%" PRIu32 "\r\n", _param_sik_radio_id.get());
				px4_usleep(200000);

			} else {
				/* reset to factory defaults */
				fprintf(fs, "AT&F\r\n");
				px4_usleep(200000);
			}

			/* write config */
			fprintf(fs, "AT&W\r\n");
			px4_usleep(200000);

			/* reboot */
			fprintf(fs, "ATZ\r\n");
			px4_usleep(200000);

			// XXX NuttX suffers from a bug where
			// fclose() also closes the fd, not just
			// the file stream. Since this is a one-time
			// config thing, we leave the file struct
			// allocated.
#ifndef __PX4_NUTTX
			fclose(fs);
#endif

		} else {
			PX4_WARN("open fd %d failed", _uart_fd);
		}

		/* reset param and save */
		_param_sik_radio_id.set(0);
		_param_sik_radio_id.commit_no_notification();
	}
}

int Mavlink::start_helper(int argc, char *argv[])
{
	/* create the instance in task context */
	Mavlink *instance = new Mavlink();

	int res;

	if (!instance) {
		/* out of memory */
		res = -ENOMEM;
		PX4_ERR("OUT OF MEM");

	} else {
		/* this will actually only return once MAVLink exits, unless there's a startup error */
		res = instance->task_main(argc, argv);

		if (res != PX4_OK) {
			delete instance;
		}
	}

	return res;
}

int
Mavlink::start(int argc, char *argv[])
{
	MavlinkULog::initialize();
	MavlinkCommandSender::initialize();

	if (!_event_buffer) {
		_event_buffer = new events::EventBuffer();
		int ret;

		if (_event_buffer && (ret = _event_buffer->init()) != 0) {
			PX4_ERR("EventBuffer init failed (%i)", ret);
			delete _event_buffer;
			_event_buffer = nullptr;
		}

		if (!_event_buffer) {
			PX4_ERR("EventBuffer alloc failed");
			return 1;
		}
	}

	// Wait for the instance count to go up one
	// before returning to the shell
	int ic = Mavlink::instance_count();

	if (ic == MAVLINK_COMM_NUM_BUFFERS) {
		PX4_ERR("Maximum MAVLink instance count of %d reached.", MAVLINK_COMM_NUM_BUFFERS);
		return 1;
	}

	// Instantiate thread

	// This is where the control flow splits
	// between the starting task and the spawned
	// task - start_helper() only returns
	// when the started task exits.
	px4_task_spawn_cmd("mavlink_main",
			   SCHED_DEFAULT,
			   SCHED_PRIORITY_DEFAULT,
			   PX4_STACK_ADJUSTED(2896) + MAVLINK_NET_ADDED_STACK,
			   (px4_main_t)&Mavlink::start_helper,
			   (char *const *)argv);

	// Ensure that this shell command
	// does not return before the instance
	// is fully initialized. As this is also
	// the only path to create a new instance,
	// this is effectively a lock on concurrent
	// instance starting. XXX do a real lock.

	// Sleep 500 us between each attempt
	const unsigned sleeptime = 500;

	// Wait 100 ms max for the startup.
	const unsigned limit = 100 * 1000 / sleeptime;

	unsigned count = 0;

	while (ic == Mavlink::instance_count() && count < limit) {
		px4_usleep(sleeptime);
		count++;
	}

	if (ic == Mavlink::instance_count()) {
		return PX4_ERROR;

	} else {
		return PX4_OK;
	}
}

void
Mavlink::display_status()
{
#if !defined(CONSTRAINED_FLASH)
	_receiver.enable_message_statistics();
#endif // !CONSTRAINED_FLASH

	if (_tstatus.heartbeat_type_gcs) {
		printf("\tGCS heartbeat valid\n");
	}

	printf("\tmavlink chan: #%u\n", static_cast<unsigned>(_channel));

	if (_tstatus.timestamp > 0) {

		printf("\ttype:\t\t");

		if (_radio_status_available) {
			printf("RADIO Link\n");
			printf("\t  rssi:\t\t%" PRIu8 "\n", _rstatus.rssi);
			printf("\t  remote rssi:\t%" PRIu8 "\n", _rstatus.remote_rssi);
			printf("\t  txbuf:\t%" PRIu8 "\n", _rstatus.txbuf);
			printf("\t  noise:\t%" PRIu8 "\n", _rstatus.noise);
			printf("\t  remote noise:\t%" PRIu8 "\n", _rstatus.remote_noise);
			printf("\t  rx errors:\t%" PRIu16 "\n", _rstatus.rxerrors);
			printf("\t  fixed:\t%" PRIu16 "\n", _rstatus.fix);

		} else if (_tstatus.type == telemetry_status_s::LINK_TYPE_USB) {
			printf("USB CDC\n");

		} else {
			printf("GENERIC LINK OR RADIO\n");
		}

	} else {
		printf("\tno radio status.\n");
	}

	printf("\tflow control: %s\n", _flow_control_mode ? "ON" : "OFF");
	printf("\trates:\n");
	printf("\t  tx: %.1f B/s\n", (double)_tstatus.tx_rate_avg);
	printf("\t  txerr: %.1f B/s\n", (double)_tstatus.tx_error_rate_avg);
	printf("\t  tx rate mult: %.3f\n", (double)_rate_mult);
	printf("\t  tx rate max: %i B/s\n", _datarate);
	printf("\t  rx: %.1f B/s\n", (double)_tstatus.rx_rate_avg);
	printf("\t  rx loss: %.1f%%\n", (double)_tstatus.rx_message_lost_rate);

#if !defined(CONSTRAINED_FLASH)
	_receiver.print_detailed_rx_stats();
#endif // !CONSTRAINED_FLASH

	if (_mavlink_ulog) {
		printf("\tULog rate: %.1f%% of max %.1f%%\n", (double)_mavlink_ulog->current_data_rate() * 100.,
		       (double)_mavlink_ulog->maximum_data_rate() * 100.);
	}

	printf("\tFTP enabled: %s, TX enabled: %s\n",
	       _ftp_on ? "YES" : "NO",
	       _transmitting_enabled ? "YES" : "NO");
	printf("\tmode: %s\n", mavlink_mode_str(_mode));
	printf("\tForwarding: %s\n", get_forwarding_on() ? "On" : "Off");
	printf("\tMAVLink version: %" PRId32 "\n", _protocol_version);

	printf("\ttransport protocol: ");

	switch (_protocol) {
#if defined(MAVLINK_UDP)

	case Protocol::UDP:
		printf("UDP (%hu, remote port: %hu)\n", _network_port, _remote_port);
		printf("\tBroadcast enabled: %s\n",
		       broadcast_enabled() ? "YES" : "NO");
#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)
		printf("\tMulticast enabled: %s\n",
		       multicast_enabled() ? "YES" : "NO");
#endif
#ifdef __PX4_POSIX

		if (get_client_source_initialized()) {
			printf("\tpartner IP: %s\n", inet_ntoa(get_client_source_address().sin_addr));
		}

#endif
		break;
#endif // MAVLINK_UDP

	case Protocol::SERIAL:
		printf("serial (%s @%i)\n", _device_name, _baudrate);
		break;
	}

	if (_ping_stats.last_ping_time > 0) {
		printf("\tping statistics:\n");
		printf("\t  last: %0.2f ms\n", (double)_ping_stats.last_rtt);
		printf("\t  mean: %0.2f ms\n", (double)_ping_stats.mean_rtt);
		printf("\t  max: %0.2f ms\n", (double)_ping_stats.max_rtt);
		printf("\t  min: %0.2f ms\n", (double)_ping_stats.min_rtt);
		printf("\t  dropped packets: %" PRIi32 "\n", _ping_stats.dropped_packets);
	}
}

void
Mavlink::display_status_streams()
{
	printf("\t%-20s%-16s %s\n", "Name", "Rate Config (current) [Hz]", "Message Size (if active) [B]");

	const float rate_mult = _rate_mult;

	for (const auto &stream : _streams) {
		const int interval = stream->get_interval();
		const unsigned size = stream->get_size();
		char rate_str[20];

		if (interval < 0) {
			strcpy(rate_str, "unlimited");

		} else {
			float rate = 1000000.0f / (float)interval;
			// Note that the actual current rate can be lower if the associated uORB topic updates at a
			// lower rate.
			float rate_current = stream->const_rate() ? rate : rate * rate_mult;
			snprintf(rate_str, sizeof(rate_str), "%6.2f (%.3f)", (double)rate, (double)rate_current);
		}

		printf("\t%-30s%-16s", stream->get_name(), rate_str);

		if (size > 0) {
			printf(" %3u\n", size);

		} else {
			printf("\n");
		}
	}
}

int
Mavlink::stop_command(int argc, char *argv[])
{
	const char *device_name = nullptr;

#if defined(MAVLINK_UDP)
	char *eptr;
	int temp_int_arg;
	unsigned short network_port = 0;
#endif // MAVLINK_UDP

	bool provided_device = false;
	bool provided_network_port = false;

	/*
	 * Called via main with original argv
	 *   mavlink start
	 *
	 *  Remove 2
	 */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {
		if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

#if defined(MAVLINK_UDP)

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;
			temp_int_arg = strtoul(argv[i + 1], &eptr, 10);

			if (*eptr == '\0') {
				network_port = temp_int_arg;

			} else {
				err_flag = true;
			}

			i++;
#endif // MAVLINK_UDP

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag) {
		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

#if defined(MAVLINK_UDP)

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);
#endif // MAVLINK_UDP

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return PX4_ERROR;
		}

		if (inst != nullptr) {
			/* set flag to stop thread and wait for all threads to finish */
			if (inst->running() && !inst->should_exit()) {
				int iterations = 0;

				while ((iterations < 1000) && inst->running()) {
					inst->request_stop();
					iterations++;
					px4_usleep(1000);
				}

				if (inst->running()) {
					PX4_ERR("unable to stop instance %d", inst->get_instance_id());
					return PX4_ERROR;
				}

				LockGuard lg{mavlink_module_mutex};

				for (int mavlink_instance = 0; mavlink_instance < MAVLINK_COMM_NUM_BUFFERS; mavlink_instance++) {
					if (mavlink_module_instances[mavlink_instance] == inst) {
						delete inst;
						mavlink_module_instances[mavlink_instance] = nullptr;
						return PX4_OK;
					}
				}
			}

			return PX4_ERROR;
		}

	} else {
		usage();
	}

	return PX4_ERROR;
}

int
Mavlink::stream_command(int argc, char *argv[])
{
	const char *device_name = DEFAULT_DEVICE_NAME;
	float rate = -1.0f;
	const char *stream_name = nullptr;
#ifdef MAVLINK_UDP
	int temp_int_arg;
	unsigned short network_port = 0;
#endif // MAVLINK_UDP
	bool provided_device = false;
	bool provided_network_port = false;

	/*
	 * Called via main with original argv
	 *   mavlink start
	 *
	 *  Remove 2
	 */
	argc -= 2;
	argv += 2;

	/* don't exit from getopt loop to leave getopt global variables in consistent state,
	 * set error flag instead */
	bool err_flag = false;

	int i = 0;

	while (i < argc) {

		if (0 == strcmp(argv[i], "-r") && i < argc - 1) {
			rate = strtod(argv[i + 1], nullptr);

			if (rate < 0.0f) {
				err_flag = true;
			}

			i++;

		} else if (0 == strcmp(argv[i], "-d") && i < argc - 1) {
			provided_device = true;
			device_name = argv[i + 1];
			i++;

		} else if (0 == strcmp(argv[i], "-s") && i < argc - 1) {
			stream_name = argv[i + 1];
			i++;

#ifdef MAVLINK_UDP

		} else if (0 == strcmp(argv[i], "-u") && i < argc - 1) {
			provided_network_port = true;

			if (px4_get_parameter_value(argv[i + 1], temp_int_arg) != 0) {
				err_flag = true;

			} else {
				network_port = temp_int_arg;
			}

			i++;
#endif // MAVLINK_UDP

		} else {
			err_flag = true;
		}

		i++;
	}

	if (!err_flag && stream_name != nullptr) {

		Mavlink *inst = nullptr;

		if (provided_device && !provided_network_port) {
			inst = get_instance_for_device(device_name);

#ifdef MAVLINK_UDP

		} else if (provided_network_port && !provided_device) {
			inst = get_instance_for_network_port(network_port);
#endif // MAVLINK_UDP

		} else if (provided_device && provided_network_port) {
			PX4_WARN("please provide either a device name or a network port");
			return 1;
		}

		if (rate < 0.0f) {
			rate = -2.0f; // use default rate
		}

		if (inst != nullptr) {
			inst->configure_stream_threadsafe(stream_name, rate);

		} else {

			// If the link is not running we should complain, but not fall over
			// because this is so easy to get wrong and not fatal. Warning is sufficient.
			if (provided_device) {
				PX4_WARN("mavlink for device %s is not running", device_name);

			}

#ifdef MAVLINK_UDP

			else {
				PX4_WARN("mavlink for network on port %hu is not running", network_port);
			}

#endif // MAVLINK_UDP

			return 1;
		}

	} else {
		usage();
		return 1;
	}

	return OK;
}

void
Mavlink::set_boot_complete()
{
	_boot_complete = true;

#if defined(MAVLINK_UDP)
	LockGuard lg {mavlink_module_mutex};

	for (Mavlink *inst : mavlink_module_instances) {
		if (inst && (inst->get_mode() != MAVLINK_MODE_ONBOARD) &&
		    !inst->broadcast_enabled() && inst->get_protocol() == Protocol::UDP) {

			PX4_INFO("MAVLink only on localhost (set param MAV_{i}_BROADCAST = 1 to enable network)");
		}
	}

#endif // MAVLINK_UDP

}

static void usage()
{

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module implements the MAVLink protocol, which can be used on a Serial link or UDP network connection.
It communicates with the system via uORB: some messages are directly handled in the module (eg. mission
protocol), others are published via uORB (eg. vehicle_command).

Streams are used to send periodic messages with a specific rate, such as the vehicle attitude.
When starting the mavlink instance, a mode can be specified, which defines the set of enabled streams with their rates.
For a running instance, streams can be configured via `mavlink stream` command.

There can be multiple independent instances of the module, each connected to one serial device or network port.

### Implementation
The implementation uses 2 threads, a sending and a receiving thread. The sender runs at a fixed rate and dynamically
reduces the rates of the streams if the combined bandwidth is higher than the configured rate (`-r`) or the
physical link becomes saturated. This can be checked with `mavlink status`, see if `rate mult` is less than 1.

**Careful**: some of the data is accessed and modified from both threads, so when changing code or extend the
functionality, this needs to be take into account, in order to avoid race conditions and corrupt data.

### Examples
Start mavlink on ttyS1 serial with baudrate 921600 and maximum sending rate of 80kB/s:
$ mavlink start -d /dev/ttyS1 -b 921600 -m onboard -r 80000

Start mavlink on UDP port 14556 and enable the HIGHRES_IMU message with 50Hz:
$ mavlink start -u 14556 -r 1000000
$ mavlink stream -u 14556 -s HIGHRES_IMU -r 50
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mavlink", "communication");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start a new instance");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Select Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 57600, 9600, 3000000, "Baudrate (can also be p:<param_name>)", true);
	PRINT_MODULE_USAGE_PARAM_INT('r', 0, 10, 10000000, "Maximum sending data rate in B/s (if 0, use baudrate / 20)", true);
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_FLAG('p', "Enable Broadcast", true);
	PRINT_MODULE_USAGE_PARAM_INT('u', 14556, 0, 65536, "Select UDP Network Port (local)", true);
	PRINT_MODULE_USAGE_PARAM_INT('o', 14550, 0, 65536, "Select UDP Network Port (remote)", true);
	PRINT_MODULE_USAGE_PARAM_STRING('t', "127.0.0.1", nullptr, "Partner IP (broadcasting can be enabled via -p flag)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('m', "normal", "custom|camera|onboard|osd|magic|config|iridium|minimal|extvision|extvisionmin|gimbal|uavionix",
					"Mode: sets default streams and rates", true);
	PRINT_MODULE_USAGE_PARAM_STRING('n', nullptr, "<interface_name>", "wifi/ethernet interface name", true);
#if defined(CONFIG_NET_IGMP) && defined(CONFIG_NET_ROUTE)
	PRINT_MODULE_USAGE_PARAM_STRING('c', nullptr, "Multicast address in the range [239.0.0.0,239.255.255.255]", "Multicast address (multicasting can be enabled via MAV_{i}_BROADCAST param)", true);
#endif
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Enable message forwarding to other Mavlink instances", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('w', "Wait to send, until first message received", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Enable FTP", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('z', "Force hardware flow control always on", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('Z', "Force hardware flow control always off", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop-all", "Stop all instances");

	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop a running instance");
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', -1, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);


	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status for all instances");
	PRINT_MODULE_USAGE_ARG("streams", "Print all enabled streams", true);

	PRINT_MODULE_USAGE_COMMAND_DESCR("stream", "Configure the sending rate of a stream for a running instance");
#if defined(CONFIG_NET) || defined(__PX4_POSIX)
	PRINT_MODULE_USAGE_PARAM_INT('u', -1, 0, 65536, "Select Mavlink instance via local Network Port", true);
#endif
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Select Mavlink instance via Serial Device", true);
	PRINT_MODULE_USAGE_PARAM_STRING('s', nullptr, nullptr, "Mavlink stream to configure", false);
	PRINT_MODULE_USAGE_PARAM_FLOAT('r', -1.0f, 0.0f, 2000.0f, "Rate in Hz (0 = turn off, -1 = set to default)", false);

	PRINT_MODULE_USAGE_COMMAND_DESCR("boot_complete",
					 "Enable sending of messages. (Must be) called as last step in startup script.");

}

extern "C" __EXPORT int mavlink_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage();
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		return Mavlink::start(argc, argv);

	} else if (!strcmp(argv[1], "stop-all")) {
		return Mavlink::destroy_all_instances();

	} else if (!strcmp(argv[1], "status")) {
		bool show_streams_status = argc > 2 && strcmp(argv[2], "streams") == 0;
		return Mavlink::get_status_all_instances(show_streams_status);

	} else if (!strcmp(argv[1], "stop")) {
		return Mavlink::stop_command(argc, argv);

	} else if (!strcmp(argv[1], "stream")) {
		return Mavlink::stream_command(argc, argv);

	} else if (!strcmp(argv[1], "boot_complete")) {
		Mavlink::set_boot_complete();
		return 0;

	} else {
		usage();
		return 1;
	}

	return 0;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2022 PX4 Development Team. All rights reserved.
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
 * @file mavlink_messages.cpp
 * MAVLink 2.0 message formatters implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_main.h"
#include "mavlink_messages.h"
#include "mavlink_command_sender.h"
#include "mavlink_simple_analyzer.h"

#include <drivers/drv_pwm_output.h>
#include <lib/conversion/rotation.h>
#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/time.h>
#include <math.h>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/vehicle_status.h>

#include "streams/ACTUATOR_OUTPUT_STATUS.hpp"
#include "streams/ALTITUDE.hpp"
#include "streams/ATTITUDE.hpp"
#include "streams/ATTITUDE_QUATERNION.hpp"
#include "streams/ATTITUDE_TARGET.hpp"
#include "streams/AUTOPILOT_VERSION.hpp"
#include "streams/BATTERY_STATUS.hpp"
#include "streams/CAMERA_IMAGE_CAPTURED.hpp"
#include "streams/CAMERA_TRIGGER.hpp"
#include "streams/COLLISION.hpp"
#include "streams/COMMAND_LONG.hpp"
#include "streams/COMPONENT_INFORMATION.hpp"
#include "streams/COMPONENT_METADATA.hpp"
#include "streams/DISTANCE_SENSOR.hpp"
#include "streams/EFI_STATUS.hpp"
#include "streams/ESC_INFO.hpp"
#include "streams/ESC_STATUS.hpp"
#include "streams/ESTIMATOR_STATUS.hpp"
#include "streams/EXTENDED_SYS_STATE.hpp"
#include "streams/FLIGHT_INFORMATION.hpp"
#include "streams/GLOBAL_POSITION_INT.hpp"
#include "streams/GPS_GLOBAL_ORIGIN.hpp"
#include "streams/GPS_RAW_INT.hpp"
#include "streams/GPS_RTCM_DATA.hpp"
#include "streams/GPS_STATUS.hpp"
#include "streams/HEARTBEAT.hpp"
#include "streams/HIGHRES_IMU.hpp"
#include "streams/HIL_ACTUATOR_CONTROLS.hpp"
#include "streams/HIL_STATE_QUATERNION.hpp"
#include "streams/HOME_POSITION.hpp"
#include "streams/HYGROMETER_SENSOR.hpp"
#include "streams/LANDING_TARGET.hpp"
#include "streams/LOCAL_POSITION_NED.hpp"
#include "streams/MAG_CAL_REPORT.hpp"
#include "streams/MANUAL_CONTROL.hpp"
#include "streams/MOUNT_ORIENTATION.hpp"
#include "streams/NAV_CONTROLLER_OUTPUT.hpp"
#include "streams/OBSTACLE_DISTANCE.hpp"
#include "streams/OPEN_DRONE_ID_BASIC_ID.hpp"
#include "streams/OPEN_DRONE_ID_LOCATION.hpp"
#include "streams/OPEN_DRONE_ID_SYSTEM.hpp"
#include "streams/OPTICAL_FLOW_RAD.hpp"
#include "streams/ORBIT_EXECUTION_STATUS.hpp"
#include "streams/PING.hpp"
#include "streams/POSITION_TARGET_GLOBAL_INT.hpp"
#include "streams/POSITION_TARGET_LOCAL_NED.hpp"
#include "streams/PROTOCOL_VERSION.hpp"
#include "streams/RAW_RPM.hpp"
#include "streams/RC_CHANNELS.hpp"
#include "streams/SCALED_IMU.hpp"
#include "streams/SCALED_IMU2.hpp"
#include "streams/SCALED_IMU3.hpp"
#include "streams/SCALED_PRESSURE.hpp"
#include "streams/SERVO_OUTPUT_RAW.hpp"
#include "streams/STATUSTEXT.hpp"
#include "streams/STORAGE_INFORMATION.hpp"
#include "streams/SYS_STATUS.hpp"
#include "streams/SYSTEM_TIME.hpp"
#include "streams/TIME_ESTIMATE_TO_TARGET.hpp"
#include "streams/TIMESYNC.hpp"
#include "streams/TRAJECTORY_REPRESENTATION_WAYPOINTS.hpp"
#include "streams/VFR_HUD.hpp"
#include "streams/VIBRATION.hpp"
#include "streams/WIND_COV.hpp"

#if !defined(CONSTRAINED_FLASH)
# include "streams/ADSB_VEHICLE.hpp"
# include "streams/AUTOPILOT_STATE_FOR_GIMBAL_DEVICE.hpp"
# include "streams/DEBUG.hpp"
# include "streams/DEBUG_FLOAT_ARRAY.hpp"
# include "streams/DEBUG_VECT.hpp"
# include "streams/GIMBAL_DEVICE_ATTITUDE_STATUS.hpp"
# include "streams/GIMBAL_DEVICE_SET_ATTITUDE.hpp"
# include "streams/GIMBAL_MANAGER_INFORMATION.hpp"
# include "streams/GIMBAL_MANAGER_STATUS.hpp"
# include "streams/GPS2_RAW.hpp"
# include "streams/HIGH_LATENCY2.hpp"
# include "streams/LINK_NODE_STATUS.hpp"
# include "streams/NAMED_VALUE_FLOAT.hpp"
# include "streams/ODOMETRY.hpp"
# include "streams/SCALED_PRESSURE2.hpp"
# include "streams/SCALED_PRESSURE3.hpp"
# include "streams/SMART_BATTERY_INFO.hpp"
# include "streams/UAVIONIX_ADSB_OUT_CFG.hpp"
# include "streams/UAVIONIX_ADSB_OUT_DYNAMIC.hpp"
# include "streams/UTM_GLOBAL_POSITION.hpp"
#endif // !CONSTRAINED_FLASH

// ensure PX4 rotation enum and MAV_SENSOR_ROTATION align
static_assert(MAV_SENSOR_ROTATION_NONE == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_NONE),
	      "Roll: 0, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_45),
	      "Roll: 0, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_90),
	      "Roll: 0, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_135),
	      "Roll: 0, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_YAW_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_180),
	      "Roll: 0, Pitch: 0, Yaw: 180");
static_assert(MAV_SENSOR_ROTATION_YAW_225 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_225),
	      "Roll: 0, Pitch: 0, Yaw: 225");
static_assert(MAV_SENSOR_ROTATION_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_270),
	      "Roll: 0, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_YAW_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_YAW_315),
	      "Roll: 0, Pitch: 0, Yaw: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180),
	      "Roll: 180, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_45),
	      "Roll: 180, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_90),
	      "Roll: 180, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_135),
	      "Roll: 180, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180),
	      "Roll: 0, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_225 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_225),
	      "Roll: 180, Pitch: 0, Yaw: 225");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_270),
	      "Roll: 180, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_YAW_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_YAW_315),
	      "Roll: 180, Pitch: 0, Yaw: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90),
	      "Roll: 90, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_45),
	      "Roll: 90, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_90),
	      "Roll: 90, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_135),
	      "Roll: 90, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_ROLL_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270),
	      "Roll: 270, Pitch: 0, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_45 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_45),
	      "Roll: 270, Pitch: 0, Yaw: 45");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_90),
	      "Roll: 270, Pitch: 0, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_YAW_135 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_YAW_135),
	      "Roll: 270, Pitch: 0, Yaw: 135");
static_assert(MAV_SENSOR_ROTATION_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_90),
	      "Roll: 0, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_270),
	      "Roll: 0, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_PITCH_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180_YAW_90),
	      "Roll: 0, Pitch: 180, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_PITCH_180_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_180_YAW_270),
	      "Roll: 0, Pitch: 180, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_90),
	      "Roll: 90, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_180_PITCH_90),
	      "Roll: 180, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_90 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_270_PITCH_90),
	      "Roll: 270, Pitch: 90, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_180),
	      "Roll: 90, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_180 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_270_PITCH_180), "Roll: 270, Pitch: 180, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_270),
	      "Roll: 90, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_180_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_180_PITCH_270), "Roll: 180, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_270_PITCH_270 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_270_PITCH_270), "Roll: 270, Pitch: 270, Yaw: 0");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_90_PITCH_180_YAW_90),
	      "Roll: 90, Pitch: 180, Yaw: 90");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_YAW_270 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_YAW_270),
	      "Roll: 90, Pitch: 0, Yaw: 270");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293 == static_cast<MAV_SENSOR_ORIENTATION>
	      (ROTATION_ROLL_90_PITCH_68_YAW_293),
	      "Roll: 90, Pitch: 68, Yaw: 293");
static_assert(MAV_SENSOR_ROTATION_PITCH_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_PITCH_315), "Pitch: 315");
static_assert(MAV_SENSOR_ROTATION_ROLL_90_PITCH_315 == static_cast<MAV_SENSOR_ORIENTATION>(ROTATION_ROLL_90_PITCH_315),
	      "Roll: 90, Pitch: 315");
static_assert(41 == ROTATION_MAX, "Keep MAV_SENSOR_ROTATION and PX4 Rotation in sync");


static const StreamListItem streams_list[] = {
#if defined(HEARTBEAT_HPP)
	create_stream_list_item<MavlinkStreamHeartbeat>(),
#endif // HEARTBEAT_HPP
#if defined(STATUSTEXT_HPP)
	create_stream_list_item<MavlinkStreamStatustext>(),
#endif // STATUSTEXT_HPP
#if defined(COMMAND_LONG_HPP)
	create_stream_list_item<MavlinkStreamCommandLong>(),
#endif // COMMAND_LONG_HPP
#if defined(SYSTEM_TIME_HPP)
	create_stream_list_item<MavlinkStreamSysStatus>(),
#endif // SYSTEM_TIME_HPP
	create_stream_list_item<MavlinkStreamBatteryStatus>(),
#if defined(SMART_BATTERY_INFO_HPP)
	create_stream_list_item<MavlinkStreamSmartBatteryInfo>(),
#endif // SMART_BATTERY_INFO_HPP
#if defined(HIGHRES_IMU_HPP)
	create_stream_list_item<MavlinkStreamHighresIMU>(),
#endif // HIGHRES_IMU_HPP
#if defined(SCALED_IMU_HPP)
	create_stream_list_item<MavlinkStreamScaledIMU>(),
#endif // SCALED_IMU_HPP
#if defined(SCALED_IMU2_HPP)
	create_stream_list_item<MavlinkStreamScaledIMU2>(),
#endif // SCALED_IMU2_HPP
#if defined(SCALED_IMU3_HPP)
	create_stream_list_item<MavlinkStreamScaledIMU3>(),
#endif // SCALED_IMU3_HPP
#if defined(SCALED_PRESSURE_HPP)
	create_stream_list_item<MavlinkStreamScaledPressure>(),
#endif // SCALED_PRESSURE
#if defined(SCALED_PRESSURE2_HPP)
	create_stream_list_item<MavlinkStreamScaledPressure2>(),
#endif // SCALED_PRESSURE2
#if defined(SCALED_PRESSURE3_HPP)
	create_stream_list_item<MavlinkStreamScaledPressure3>(),
#endif // SCALED_PRESSURE3
#if defined(ACTUATOR_OUTPUT_STATUS_HPP)
	create_stream_list_item<MavlinkStreamActuatorOutputStatus>(),
#endif // ACTUATOR_OUTPUT_STATUS_HPP
#if defined(ATTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAttitude>(),
#endif // ATTITUDE_HPP
#if defined(ATTITUDE_QUATERNION_HPP)
	create_stream_list_item<MavlinkStreamAttitudeQuaternion>(),
#endif // ATTITUDE_QUATERNION_HPP
#if defined(VFR_HUD_HPP)
	create_stream_list_item<MavlinkStreamVFRHUD>(),
#endif // VFR_HUD_HPP
#if defined(GPS_GLOBAL_ORIGIN_HPP)
	create_stream_list_item<MavlinkStreamGpsGlobalOrigin>(),
#endif // GPS_GLOBAL_ORIGIN_HPP
#if defined(GPS_RAW_INT_HPP)
	create_stream_list_item<MavlinkStreamGPSRawInt>(),
#endif // GPS_RAW_INT_HPP
#if defined(GPS2_RAW_HPP)
	create_stream_list_item<MavlinkStreamGPS2Raw>(),
#endif // GPS2_RAW_HPP
#if defined(SYSTEM_TIME_HPP)
	create_stream_list_item<MavlinkStreamSystemTime>(),
#endif // SYSTEM_TIME_HPP
#if defined(TIME_ESTIMATE_TO_TARGET_HPP)
	create_stream_list_item<MavlinkStreamTimeEstimateToTarget>(),
#endif // TIME_ESTIMATE_TO_TARGET_HPP
#if defined(TIMESYNC_HPP)
	create_stream_list_item<MavlinkStreamTimesync>(),
#endif // TIMESYNC_HPP
#if defined(GLOBAL_POSITION_INT_HPP)
	create_stream_list_item<MavlinkStreamGlobalPositionInt>(),
#endif // GLOBAL_POSITION_INT_HPP
#if defined(LANDING_TARGET_HPP)
	create_stream_list_item<MavlinkStreamLandingTarget>(),
#endif
#if defined(LOCAL_POSITION_NED_HPP)
	create_stream_list_item<MavlinkStreamLocalPositionNED>(),
#endif // LOCAL_POSITION_NED_HPP
#if defined(MAG_CAL_REPORT_HPP)
	create_stream_list_item<MavlinkStreamMagCalReport>(),
#endif // MAG_CAL_REPORT_HPP
#if defined(ODOMETRY_HPP)
	create_stream_list_item<MavlinkStreamOdometry>(),
#endif // ODOMETRY_HPP
#if defined(ESTIMATOR_STATUS_HPP)
	create_stream_list_item<MavlinkStreamEstimatorStatus>(),
#endif // ESTIMATOR_STATUS_HPP
#if defined(VIBRATION_HPP)
	create_stream_list_item<MavlinkStreamVibration>(),
#endif // VIBRATION_HPP
#if defined(AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP)
	create_stream_list_item<MavlinkStreamAutopilotStateForGimbalDevice>(),
#endif // AUTOPILOT_STATE_FOR_GIMBAL_DEVICE_HPP
#if defined(GIMBAL_DEVICE_ATTITUDE_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGimbalDeviceAttitudeStatus>(),
#endif // GIMBAL_DEVICE_ATTITUDE_STATUS_HPP
#if defined(GIMBAL_MANAGER_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamGimbalManagerInformation>(),
#endif // GIMBAL_MANAGER_INFORMATION_HPP
#if defined(GIMBAL_MANAGER_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGimbalManagerStatus>(),
#endif // GIMBAL_MANAGER_STATUS_HPP
#if defined(GIMBAL_DEVICE_SET_ATTITUDE_HPP)
	create_stream_list_item<MavlinkStreamGimbalDeviceSetAttitude>(),
#endif // GIMBAL_DEVICE_SET_ATTITUDE_HPP
#if defined(HOME_POSITION_HPP)
	create_stream_list_item<MavlinkStreamHomePosition>(),
#endif // HOME_POSITION_HPP
#if defined(HYGROMETER_SENSOR_HPP)
	create_stream_list_item<MavlinkStreamHygrometerSensor>(),
#endif // HYGROMETER_SENSOR_HPP
#if defined(SERVO_OUTPUT_RAW_HPP)
	create_stream_list_item<MavlinkStreamServoOutputRaw<0> >(),
	create_stream_list_item<MavlinkStreamServoOutputRaw<1> >(),
#endif // SERVO_OUTPUT_RAW_HPP
#if defined(HIL_ACTUATOR_CONTROLS_HPP)
	create_stream_list_item<MavlinkStreamHILActuatorControls>(),
#endif // HIL_ACTUATOR_CONTROLS_HPP
#if defined(POSITION_TARGET_GLOBAL_INT_HPP)
	create_stream_list_item<MavlinkStreamPositionTargetGlobalInt>(),
#endif // POSITION_TARGET_GLOBAL_INT_HPP
#if defined(POSITION_TARGET_LOCAL_NED_HPP)
	create_stream_list_item<MavlinkStreamPositionTargetLocalNed>(),
#endif // POSITION_TARGET_LOCAL_NED_HPP
#if defined(ATTITUDE_TARGET_HPP)
	create_stream_list_item<MavlinkStreamAttitudeTarget>(),
#endif // ATTITUDE_TARGET_HPP
#if defined(RC_CHANNELS_HPP)
	create_stream_list_item<MavlinkStreamRCChannels>(),
#endif // RC_CHANNELS_HPP
#if defined(MANUAL_CONTROL_HPP)
	create_stream_list_item<MavlinkStreamManualControl>(),
#endif // MANUAL_CONTROL_HPP
#if defined(TRAJECTORY_REPRESENTATION_WAYPOINTS_HPP)
	create_stream_list_item<MavlinkStreamTrajectoryRepresentationWaypoints>(),
#endif // TRAJECTORY_REPRESENTATION_WAYPOINTS_HPP
#if defined(OPTICAL_FLOW_RAD_HPP)
	create_stream_list_item<MavlinkStreamOpticalFlowRad>(),
#endif // OPTICAL_FLOW_RAD_HPP
#if defined(NAMED_VALUE_FLOAT_HPP)
	create_stream_list_item<MavlinkStreamNamedValueFloat>(),
#endif // NAMED_VALUE_FLOAT_HPP
#if defined(DEBUG_HPP)
	create_stream_list_item<MavlinkStreamDebug>(),
#endif // DEBUG_HPP
#if defined(DEBUG_VECT_HPP)
	create_stream_list_item<MavlinkStreamDebugVect>(),
#endif // DEBUG_VECT_HPP
#if defined(DEBUG_FLOAT_ARRAY_HPP)
	create_stream_list_item<MavlinkStreamDebugFloatArray>(),
#endif // DEBUG_FLOAT_ARRAY_HPP
#if defined(NAV_CONTROLLER_OUTPUT_HPP)
	create_stream_list_item<MavlinkStreamNavControllerOutput>(),
#endif // NAV_CONTROLLER_OUTPUT_HPP
#if defined(CAMERA_TRIGGER_HPP)
	create_stream_list_item<MavlinkStreamCameraTrigger>(),
#endif // CAMERA_TRIGGER_HPP
#if defined(CAMERA_IMAGE_CAPTURED_HPP)
	create_stream_list_item<MavlinkStreamCameraImageCaptured>(),
#endif // CAMERA_IMAGE_CAPTURED_HPP
#if defined(DISTANCE_SENSOR_HPP)
	create_stream_list_item<MavlinkStreamDistanceSensor>(),
#endif // DISTANCE_SENSOR_HPP
#if defined(EXTENDED_SYS_STATE_HPP)
	create_stream_list_item<MavlinkStreamExtendedSysState>(),
#endif // EXTENDED_SYS_STATE_HPP
#if defined(ALTITUDE_HPP)
	create_stream_list_item<MavlinkStreamAltitude>(),
#endif // ALTITUDE_HPP
#if defined(ADSB_VEHICLE_HPP)
	create_stream_list_item<MavlinkStreamADSBVehicle>(),
#endif // ADSB_VEHICLE_HPP
#if defined(UTM_GLOBAL_POSITION_HPP)
	create_stream_list_item<MavlinkStreamUTMGlobalPosition>(),
#endif // UTM_GLOBAL_POSITION_HPP
#if defined(COLLISION_HPP)
	create_stream_list_item<MavlinkStreamCollision>(),
#endif // COLLISION_HPP
#if defined(WIND_COV_HPP)
	create_stream_list_item<MavlinkStreamWindCov>(),
#endif // WIND_COV_HPP
#if defined(MOUNT_ORIENTATION_HPP)
	create_stream_list_item<MavlinkStreamMountOrientation>(),
#endif // MOUNT_ORIENTATION_HPP
#if defined(HIGH_LATENCY2_HPP)
	create_stream_list_item<MavlinkStreamHighLatency2>(),
#endif // HIGH_LATENCY2_HPP
#if defined(HIL_STATE_QUATERNION_HPP)
	create_stream_list_item<MavlinkStreamHILStateQuaternion>(),
#endif // HIL_STATE_QUATERNION_HPP
#if defined(PING_HPP)
	create_stream_list_item<MavlinkStreamPing>(),
#endif // PING_HPP
#if defined(ORBIT_EXECUTION_STATUS_HPP)
	create_stream_list_item<MavlinkStreamOrbitStatus>(),
#endif // ORBIT_EXECUTION_STATUS_HPP
#if defined(OBSTACLE_DISTANCE_HPP)
	create_stream_list_item<MavlinkStreamObstacleDistance>(),
#endif // OBSTACLE_DISTANCE_HPP
#if defined(OPEN_DRONE_ID_BASIC_ID_HPP)
	create_stream_list_item<MavlinkStreamOpenDroneIdBasicId>(),
#endif // OPEN_DRONE_ID_BASIC_ID_HPP
#if defined(OPEN_DRONE_ID_LOCATION_HPP)
	create_stream_list_item<MavlinkStreamOpenDroneIdLocation>(),
#endif // OPEN_DRONE_ID_LOCATION_HPP
#if defined(OPEN_DRONE_ID_SYSTEM_HPP)
	create_stream_list_item<MavlinkStreamOpenDroneIdSystem>(),
#endif // OPEN_DRONE_ID_SYSTEM_HPP
#if defined(ESC_INFO_HPP)
	create_stream_list_item<MavlinkStreamESCInfo>(),
#endif // ESC_INFO_HPP
#if defined(ESC_STATUS_HPP)
	create_stream_list_item<MavlinkStreamESCStatus>(),
#endif // ESC_STATUS_HPP
#if defined(AUTOPILOT_VERSION_HPP)
	create_stream_list_item<MavlinkStreamAutopilotVersion>(),
#endif // AUTOPILOT_VERSION_HPP
#if defined(PROTOCOL_VERSION_HPP)
	create_stream_list_item<MavlinkStreamProtocolVersion>(),
#endif // PROTOCOL_VERSION_HPP
#if defined(FLIGHT_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamFlightInformation>(),
#endif // FLIGHT_INFORMATION_HPP
#if defined(GPS_STATUS_HPP)
	create_stream_list_item<MavlinkStreamGPSStatus>(),
#endif // GPS_STATUS_HPP
#if defined(LINK_NODE_STATUS_HPP)
	create_stream_list_item<MavlinkStreamLinkNodeStatus>(),
#endif // LINK_NODE_STATUS_HPP
#if defined(STORAGE_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamStorageInformation>(),
#endif // STORAGE_INFORMATION_HPP
#if defined(COMPONENT_INFORMATION_HPP)
	create_stream_list_item<MavlinkStreamComponentInformation>(),
#endif // COMPONENT_INFORMATION_HPP
#if defined(COMPONENT_METADATA_HPP)
	create_stream_list_item<MavlinkStreamComponentMetadata>(),
#endif // COMPONENT_METADATA_HPP
#if defined(RAW_RPM_HPP)
	create_stream_list_item<MavlinkStreamRawRpm>(),
#endif // RAW_RPM_HPP
#if defined(EFI_STATUS_HPP)
	create_stream_list_item<MavlinkStreamEfiStatus>(),
#endif // EFI_STATUS_HPP
#if defined(GPS_RTCM_DATA_HPP)
	create_stream_list_item<MavlinkStreamGPSRTCMData>(),
#endif // GPS_RTCM_DATA_HPP
#if defined(UAVIONIX_ADSB_OUT_CFG_HPP)
	create_stream_list_item<MavlinkStreamUavionixADSBOutCfg>(),
#endif // UAVIONIX_ADSB_OUT_CFG_HPP
#if defined(UAVIONIX_ADSB_OUT_DYNAMIC_HPP)
	create_stream_list_item<MavlinkStreamUavionixADSBOutDynamic>()
#endif // UAVIONIX_ADSB_OUT_DYNAMIC_HPP
};

const char *get_stream_name(const uint16_t msg_id)
{
	// search for stream with specified msg id in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.get_name();
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const char *stream_name, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	if (stream_name != nullptr) {
		for (const auto &stream : streams_list) {
			if (strcmp(stream_name, stream.get_name()) == 0) {
				return stream.new_instance(mavlink);
			}
		}
	}

	return nullptr;
}

MavlinkStream *create_mavlink_stream(const uint16_t msg_id, Mavlink *mavlink)
{
	// search for stream with specified name in supported streams list
	for (const auto &stream : streams_list) {
		if (msg_id == stream.get_id()) {
			return stream.new_instance(mavlink);
		}
	}

	return nullptr;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_mission.cpp
 * MAVLink mission manager implementation.
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Julian Oes <julian@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 */

#include "mavlink_mission.h"
#include "mavlink_main.h"

#include <lib/geo/geo.h>
#include <systemlib/err.h>
#include <drivers/drv_hrt.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/events.h>
#include <mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <navigator/navigation.h>
#include <uORB/topics/mission.h>
#include <uORB/topics/mission_result.h>

using matrix::wrap_2pi;

dm_item_t MavlinkMissionManager::_dataman_id = DM_KEY_WAYPOINTS_OFFBOARD_0;
bool MavlinkMissionManager::_dataman_init = false;
uint16_t MavlinkMissionManager::_count[3] = { 0, 0, 0 };
int32_t MavlinkMissionManager::_current_seq = 0;
bool MavlinkMissionManager::_transfer_in_progress = false;
constexpr uint16_t MavlinkMissionManager::MAX_COUNT[];
uint16_t MavlinkMissionManager::_geofence_update_counter = 0;
uint16_t MavlinkMissionManager::_safepoint_update_counter = 0;


#define CHECK_SYSID_COMPID_MISSION(_msg)		(_msg.target_system == mavlink_system.sysid && \
		((_msg.target_component == mavlink_system.compid) || \
		 (_msg.target_component == MAV_COMP_ID_MISSIONPLANNER) || \
		 (_msg.target_component == MAV_COMP_ID_ALL)))

MavlinkMissionManager::MavlinkMissionManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	init_offboard_mission();
}

void
MavlinkMissionManager::init_offboard_mission()
{
	if (!_dataman_init) {
		_dataman_init = true;

		/* lock MISSION_STATE item */
		int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

		if (dm_lock_ret != 0) {
			PX4_ERR("DM_KEY_MISSION_STATE lock failed");
		}

		mission_s mission_state;
		int ret = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s));

		/* unlock MISSION_STATE item */
		if (dm_lock_ret == 0) {
			dm_unlock(DM_KEY_MISSION_STATE);
		}

		if (ret > 0) {
			_dataman_id = (dm_item_t)mission_state.dataman_id;
			_count[MAV_MISSION_TYPE_MISSION] = mission_state.count;
			_current_seq = mission_state.current_seq;

		} else if (ret < 0) {
			PX4_WARN("offboard mission init failed (%i)", ret);
		}

		load_geofence_stats();

		load_safepoint_stats();
	}

	_my_dataman_id = _dataman_id;
}

int
MavlinkMissionManager::load_geofence_stats()
{
	mission_stats_entry_s stats;
	// initialize fence points count
	int ret = dm_read(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = stats.num_items;
		_geofence_update_counter = stats.update_counter;
	}

	return ret;
}

int
MavlinkMissionManager::load_safepoint_stats()
{
	mission_stats_entry_s stats;
	// initialize safe points count
	int ret = dm_read(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (ret == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = stats.num_items;
	}

	return ret;
}

/**
 * Publish mission topic to notify navigator about changes.
 */
int
MavlinkMissionManager::update_active_mission(dm_item_t dataman_id, uint16_t count, int32_t seq)
{
	// We want to make sure the whole struct is initialized including padding before getting written by dataman.
	mission_s mission{};
	mission.timestamp = hrt_absolute_time();
	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	/* update mission state in dataman */

	/* lock MISSION_STATE item */
	int dm_lock_ret = dm_lock(DM_KEY_MISSION_STATE);

	if (dm_lock_ret != 0) {
		PX4_ERR("DM_KEY_MISSION_STATE lock failed");
	}

	int res = dm_write(DM_KEY_MISSION_STATE, 0, &mission, sizeof(mission_s));

	/* unlock MISSION_STATE item */
	if (dm_lock_ret == 0) {
		dm_unlock(DM_KEY_MISSION_STATE);
	}

	if (res == sizeof(mission_s)) {
		/* update active mission state */
		_dataman_id = dataman_id;
		_count[MAV_MISSION_TYPE_MISSION] = count;
		_current_seq = seq;
		_my_dataman_id = _dataman_id;

		/* mission state saved successfully, publish offboard_mission topic */
		_offboard_mission_pub.publish(mission);

		return PX4_OK;

	} else {
		PX4_ERR("WPM: can't save mission state");

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD\t");
			events::send(events::ID("mavlink_mission_storage_write_failure"), events::Log::Critical,
				     "Mission: Unable to write to storage");
		}

		return PX4_ERROR;
	}
}

int
MavlinkMissionManager::update_geofence_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_geofence_update_counter; // this makes sure navigator will reload the fence data

	/* update stats in dataman */
	int res = dm_write(DM_KEY_FENCE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_FENCE] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD\t");
			events::send(events::ID("mavlink_mission_storage_write_failure2"), events::Log::Critical,
				     "Mission: Unable to write to storage");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

int
MavlinkMissionManager::update_safepoint_count(unsigned count)
{
	mission_stats_entry_s stats;
	stats.num_items = count;
	stats.update_counter = ++_safepoint_update_counter;

	/* update stats in dataman */
	int res = dm_write(DM_KEY_SAFE_POINTS, 0, &stats, sizeof(mission_stats_entry_s));

	if (res == sizeof(mission_stats_entry_s)) {
		_count[MAV_MISSION_TYPE_RALLY] = count;

	} else {

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD\t");
			events::send(events::ID("mavlink_mission_storage_write_failure3"), events::Log::Critical,
				     "Mission: Unable to write to storage");
		}

		return PX4_ERROR;
	}

	return PX4_OK;
}

void
MavlinkMissionManager::send_mission_ack(uint8_t sysid, uint8_t compid, uint8_t type)
{
	mavlink_mission_ack_t wpa{};

	wpa.target_system = sysid;
	wpa.target_component = compid;
	wpa.type = type;
	wpa.mission_type = _mission_type;

	mavlink_msg_mission_ack_send_struct(_mavlink->get_channel(), &wpa);

	PX4_DEBUG("WPM: Send MISSION_ACK type %u to ID %u", wpa.type, wpa.target_system);
}

void
MavlinkMissionManager::send_mission_current(uint16_t seq)
{
	mavlink_mission_current_t wpc{};
	wpc.seq = seq;
	mavlink_msg_mission_current_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_CURRENT seq %d", seq);
}

void
MavlinkMissionManager::send_mission_count(uint8_t sysid, uint8_t compid, uint16_t count, MAV_MISSION_TYPE mission_type)
{
	_time_last_sent = hrt_absolute_time();

	mavlink_mission_count_t wpc{};

	wpc.target_system = sysid;
	wpc.target_component = compid;
	wpc.count = count;
	wpc.mission_type = mission_type;

	mavlink_msg_mission_count_send_struct(_mavlink->get_channel(), &wpc);

	PX4_DEBUG("WPM: Send MISSION_COUNT %u to ID %u, mission type=%i", wpc.count, wpc.target_system, mission_type);
}

void
MavlinkMissionManager::send_mission_item(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	mission_item_s mission_item{};
	int16_t bytes_read = 0;
	bool read_success = false;

	switch (_mission_type) {

	case MAV_MISSION_TYPE_MISSION: {
			bytes_read = dm_read(_dataman_id, seq, &mission_item, sizeof(mission_item_s));
			read_success = (bytes_read == sizeof(mission_item_s));
		}
		break;

	case MAV_MISSION_TYPE_FENCE: { // Read a geofence point
			mission_fence_point_s mission_fence_point;
			bytes_read = dm_read(DM_KEY_FENCE_POINTS, seq + 1, &mission_fence_point, sizeof(mission_fence_point_s));
			read_success = (bytes_read == sizeof(mission_fence_point_s));

			mission_item.nav_cmd = mission_fence_point.nav_cmd;
			mission_item.frame = mission_fence_point.frame;
			mission_item.lat = mission_fence_point.lat;
			mission_item.lon = mission_fence_point.lon;
			mission_item.altitude = mission_fence_point.alt;

			if (mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
			    mission_fence_point.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
				mission_item.vertex_count = mission_fence_point.vertex_count;

			} else {
				mission_item.circle_radius = mission_fence_point.circle_radius;
			}
		}
		break;

	case MAV_MISSION_TYPE_RALLY: { // Read a safe point / rally point
			mission_safe_point_s mission_safe_point;
			bytes_read = dm_read(DM_KEY_SAFE_POINTS, seq + 1, &mission_safe_point, sizeof(mission_safe_point_s));
			read_success = (bytes_read == sizeof(mission_safe_point_s));

			mission_item.nav_cmd = MAV_CMD_NAV_RALLY_POINT;
			mission_item.frame = mission_safe_point.frame;
			mission_item.lat = mission_safe_point.lat;
			mission_item.lon = mission_safe_point.lon;
			mission_item.altitude = mission_safe_point.alt;
		}
		break;

	default:
		_mavlink->send_statustext_critical("Received unknown mission type, abort.\t");
		events::send(events::ID("mavlink_mission_recv_unknown_mis_type"), events::Log::Error,
			     "Received unknown mission type, abort");
		break;
	}

	if (read_success) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_item_int_t wp{};
			format_mavlink_mission_item(&mission_item, reinterpret_cast<mavlink_mission_item_t *>(&wp));

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_int_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM_INT seq %u to ID %u", wp.seq, wp.target_system);

		} else {
			mavlink_mission_item_t wp{};
			format_mavlink_mission_item(&mission_item, &wp);

			wp.target_system = sysid;
			wp.target_component = compid;
			wp.seq = seq;
			wp.current = (_current_seq == seq) ? 1 : 0;

			mavlink_msg_mission_item_send_struct(_mavlink->get_channel(), &wp);

			PX4_DEBUG("WPM: Send MISSION_ITEM seq %u to ID %u", wp.seq, wp.target_system);
		}

	} else {
		send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

		if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			mavlink_log_critical(_mavlink->get_mavlink_log_pub(),
					     "Mission storage: Unable to read from storage, type: %" PRId8 " bytes read: %" PRId16 "\t",
					     (uint8_t)_mission_type, bytes_read);
			/* EVENT
			 * @description Mission type: {1}. Number of bytes read: {2}
			 */
			events::send<uint8_t, int16_t>(events::ID("mavlink_mission_storage_read_failure"), events::Log::Error,
						       "Mission: Unable to read from storage", _mission_type, bytes_read);
		}

		PX4_DEBUG("WPM: Send MISSION_ITEM ERROR: could not read seq %u from dataman ID %i", seq, _dataman_id);
	}
}

uint16_t
MavlinkMissionManager::current_max_item_count()
{
	if (_mission_type >= sizeof(MAX_COUNT) / sizeof(MAX_COUNT[0])) {
		PX4_ERR("WPM: MAX_COUNT out of bounds (%u)", _mission_type);
		return 0;
	}

	return MAX_COUNT[_mission_type];
}

uint16_t
MavlinkMissionManager::current_item_count()
{
	if (_mission_type >= sizeof(_count) / sizeof(_count[0])) {
		PX4_ERR("WPM: _count out of bounds (%u)", _mission_type);
		return 0;
	}

	return _count[_mission_type];
}

void
MavlinkMissionManager::send_mission_request(uint8_t sysid, uint8_t compid, uint16_t seq)
{
	if (seq < current_max_item_count()) {
		_time_last_sent = hrt_absolute_time();

		if (_int_mode) {
			mavlink_mission_request_int_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;
			mavlink_msg_mission_request_int_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST_INT seq %u to ID %u", wpr.seq, wpr.target_system);

		} else {

			mavlink_mission_request_t wpr{};
			wpr.target_system = sysid;
			wpr.target_component = compid;
			wpr.seq = seq;
			wpr.mission_type = _mission_type;

			mavlink_msg_mission_request_send_struct(_mavlink->get_channel(), &wpr);

			PX4_DEBUG("WPM: Send MISSION_REQUEST seq %u to ID %u", wpr.seq, wpr.target_system);
		}

	} else {
		_mavlink->send_statustext_critical("ERROR: Waypoint index exceeds list capacity\t");
		events::send<uint16_t>(events::ID("mavlink_mission_wp_index_exceeds_list"), events::Log::Error,
				       "Waypoint index eceeds list capacity (maximum: {1})", current_max_item_count());

		PX4_DEBUG("WPM: Send MISSION_REQUEST ERROR: seq %u exceeds list capacity", seq);
	}
}

void
MavlinkMissionManager::send_mission_item_reached(uint16_t seq)
{
	mavlink_mission_item_reached_t wp_reached{};

	wp_reached.seq = seq;

	mavlink_msg_mission_item_reached_send_struct(_mavlink->get_channel(), &wp_reached);

	PX4_DEBUG("WPM: Send MISSION_ITEM_REACHED reached_seq %u", wp_reached.seq);
}

void
MavlinkMissionManager::send()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	mission_result_s mission_result{};

	if (_mission_result_sub.update(&mission_result)) {

		if (_current_seq != mission_result.seq_current) {

			_current_seq = mission_result.seq_current;

			PX4_DEBUG("WPM: got mission result, new current_seq: %ld", _current_seq);

			if (mission_result.seq_total > 0) {
				if (mission_result.seq_current < mission_result.seq_total) {
					send_mission_current(_current_seq);

				} else {
					_mavlink->send_statustext_critical("ERROR: wp index out of bounds\t");
					events::send<int32_t, int32_t>(events::ID("mavlink_mission_wp_index_out_of_bounds"), events::Log::Error,
								       "Waypoint index out of bounds ({1} \\< {2})", mission_result.seq_current, mission_result.seq_total);
				}
			}
		}

		if (_last_reached != mission_result.seq_reached) {

			_last_reached = mission_result.seq_reached;
			_reached_sent_count = 0;

			PX4_DEBUG("WPM: got mission result, new seq_reached: %ld", _last_reached);

			if ((mission_result.seq_total > 0) && (_last_reached >= 0)) {
				send_mission_item_reached((uint16_t)_last_reached);
			}
		}

		if (mission_result.item_do_jump_changed
		    && (mission_result.seq_total > 0)
		    && (mission_result.item_changed_index < mission_result.seq_total)) {

			/* Send a mission item again if the remaining DO_JUMPs has changed, but don't interfere
			* if there are ongoing transfers happening already. */
			if (_state == MAVLINK_WPM_STATE_IDLE) {
				_mission_type = MAV_MISSION_TYPE_MISSION;
				send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, mission_result.item_changed_index);
			}
		}

	} else if (_slow_rate_limiter.check(hrt_absolute_time())) {
		if ((_count[MAV_MISSION_TYPE_MISSION] > 0) && (_current_seq >= 0)) {

			send_mission_current(_current_seq);

			// send the reached message another 10 times
			if (_last_reached >= 0 && (_reached_sent_count < 10)) {
				send_mission_item_reached((uint16_t)_last_reached);
				_reached_sent_count++;
			}
		}
	}

	/* check for timed-out operations */
	if (_state == MAVLINK_WPM_STATE_GETLIST && (_time_last_sent > 0)
	    && hrt_elapsed_time(&_time_last_sent) > MAVLINK_MISSION_RETRY_TIMEOUT_DEFAULT) {

		// try to request item again after timeout
		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

	} else if (_state != MAVLINK_WPM_STATE_IDLE && (_time_last_recv > 0)
		   && hrt_elapsed_time(&_time_last_recv) > MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT) {

		_mavlink->send_statustext_critical("Operation timeout\t");
		events::send(events::ID("mavlink_mission_op_timeout"), events::Log::Error,
			     "Operation timeout, aborting transfer");

		PX4_DEBUG("WPM: Last operation (state=%d) timed out, changing state to MAVLINK_WPM_STATE_IDLE", _state);

		switch_to_idle_state();

		// since we are giving up, reset this state also, so another request can be started.
		_transfer_in_progress = false;

	} else if (_state == MAVLINK_WPM_STATE_IDLE) {
		// reset flags
		_time_last_sent = 0;
		_time_last_recv = 0;
	}
}

void
MavlinkMissionManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_MISSION_ACK:
		handle_mission_ack(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		handle_mission_set_current(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		handle_mission_request_list(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST:
		handle_mission_request(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
		handle_mission_request_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_COUNT:
		handle_mission_count(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM:
		handle_mission_item(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_ITEM_INT:
		handle_mission_item_int(msg);
		break;

	case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		handle_mission_clear_all(msg);
		break;

	default:
		break;
	}
}

void
MavlinkMissionManager::handle_mission_ack(const mavlink_message_t *msg)
{
	mavlink_mission_ack_t wpa{};
	mavlink_msg_mission_ack_decode(msg, &wpa);

	if (CHECK_SYSID_COMPID_MISSION(wpa)) {
		if ((msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid)) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST && _mission_type == wpa.mission_type) {

				_time_last_recv = hrt_absolute_time();

				if (wpa.type == MAV_MISSION_ACCEPTED && _transfer_seq == current_item_count()) {
					PX4_DEBUG("WPM: MISSION_ACK OK all items sent, switch to state IDLE");

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");

				} else {
					PX4_DEBUG("WPM: MISSION_ACK ERROR: not all items sent, switch to state IDLE anyway");
				}

				switch_to_idle_state();

			} else if (_state == MAVLINK_WPM_STATE_GETLIST) {

				// INT or float mode is not supported
				if (wpa.type == MAV_MISSION_UNSUPPORTED) {

					if (_int_mode) {
						_int_mode = false;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);

					} else {
						_int_mode = true;
						send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
					}

				} else if (wpa.type == MAV_MISSION_OPERATION_CANCELLED) {
					PX4_DEBUG("WPM: MISSION_ACK CANCELLED, switch to state IDLE");
					switch_to_idle_state();
					_transfer_in_progress = false;

				} else if (wpa.type != MAV_MISSION_ACCEPTED) {
					PX4_WARN("Mission ack result was %d", wpa.type);
				}
			}

		} else {
			_mavlink->send_statustext_critical("REJ. WP CMD: partner id mismatch\t");
			events::send(events::ID("mavlink_mission_partner_id_mismatch"), events::Log::Error,
				     "Rejecting waypoint command, component or system ID mismatch");

			PX4_DEBUG("WPM: MISSION_ACK ERR: ID mismatch");
		}
	}
}

void
MavlinkMissionManager::handle_mission_set_current(const mavlink_message_t *msg)
{
	mavlink_mission_set_current_t wpc;
	mavlink_msg_mission_set_current_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (wpc.seq < _count[MAV_MISSION_TYPE_MISSION]) {
				if (update_active_mission(_dataman_id, _count[MAV_MISSION_TYPE_MISSION], wpc.seq) == PX4_OK) {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d OK", wpc.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_SET_CURRENT seq=%d ERROR", wpc.seq);

					_mavlink->send_statustext_critical("WPM: WP CURR CMD: Error setting ID\t");
					events::send(events::ID("mavlink_mission_err_id"), events::Log::Error,
						     "Failed to write current mission ID to storage");
				}

			} else {
				PX4_ERR("WPM: MISSION_SET_CURRENT seq=%d ERROR: not in list", wpc.seq);

				_mavlink->send_statustext_critical("WPM: WP CURR CMD: Not in list\t");
				events::send(events::ID("mavlink_mission_seq_out_of_bounds"), events::Log::Error,
					     "New mission waypoint sequence out of bounds");
			}

		} else {
			PX4_DEBUG("WPM: MISSION_SET_CURRENT ERROR: busy");

			_mavlink->send_statustext_critical("WPM: IGN WP CURR CMD: Busy\t");
			events::send(events::ID("mavlink_mission_state_busy"), events::Log::Error,
				     "Mission manager currently busy, ignoring new waypoint index");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request_list(const mavlink_message_t *msg)
{
	mavlink_mission_request_list_t wprl;
	mavlink_msg_mission_request_list_decode(msg, &wprl);

	if (CHECK_SYSID_COMPID_MISSION(wprl)) {
		const bool maybe_completed = (_transfer_seq == current_item_count());

		// If all mission items have been sent and a new mission request list comes in, we can proceed even if  MISSION_ACK was
		// never received. This could happen on a quick reconnect that doesn't trigger MAVLINK_MISSION_PROTOCOL_TIMEOUT_DEFAULT
		if (maybe_completed) {
			switch_to_idle_state();
		}

		if (_state == MAVLINK_WPM_STATE_IDLE || (_state == MAVLINK_WPM_STATE_SENDLIST
				&& (uint8_t)_mission_type == wprl.mission_type)) {
			_time_last_recv = hrt_absolute_time();

			_state = MAVLINK_WPM_STATE_SENDLIST;
			_mission_type = (MAV_MISSION_TYPE)wprl.mission_type;

			// make sure our item counts are up-to-date
			switch (_mission_type) {
			case MAV_MISSION_TYPE_FENCE:
				load_geofence_stats();
				break;

			case MAV_MISSION_TYPE_RALLY:
				load_safepoint_stats();
				break;

			default:
				break;
			}

			_transfer_seq = 0;
			_transfer_count = current_item_count();
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;

			if (_transfer_count > 0) {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK, %u mission items to send, mission type=%i", _transfer_count, _mission_type);

			} else {
				PX4_DEBUG("WPM: MISSION_REQUEST_LIST OK nothing to send, mission is empty, mission type=%i", _mission_type);
			}

			send_mission_count(msg->sysid, msg->compid, _transfer_count, _mission_type);

		} else {
			PX4_DEBUG("WPM: MISSION_REQUEST_LIST ERROR: busy");

			_mavlink->send_statustext_info("Mission download request ignored, already active\t");
			events::send(events::ID("mavlink_mission_req_ignored"), events::Log::Warning,
				     "Mission download request ignored, already active");
		}
	}
}


void
MavlinkMissionManager::handle_mission_request(const mavlink_message_t *msg)
{
	// The request comes in the old float mode, so we switch to it.
	if (_int_mode) {
		_int_mode = false;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_int(const mavlink_message_t *msg)
{
	// The request comes in the new int mode, so we switch to it.
	if (!_int_mode) {
		_int_mode = true;
	}

	handle_mission_request_both(msg);
}

void
MavlinkMissionManager::handle_mission_request_both(const mavlink_message_t *msg)
{
	/* The mavlink_message_t could also be a mavlink_mission_request_int_t, however the structs
	 * are basically the same, so we can ignore it. */
	mavlink_mission_request_t wpr;
	mavlink_msg_mission_request_decode(msg, &wpr);

	if (CHECK_SYSID_COMPID_MISSION(wpr)) {
		if (msg->sysid == _transfer_partner_sysid && msg->compid == _transfer_partner_compid) {
			if (_state == MAVLINK_WPM_STATE_SENDLIST) {

				if (_mission_type != wpr.mission_type) {
					PX4_WARN("WPM: Unexpected mission type (%u %u)", wpr.mission_type, _mission_type);
					return;
				}

				_time_last_recv = hrt_absolute_time();

				/* _transfer_seq contains sequence of expected request */
				if (wpr.seq == _transfer_seq && _transfer_seq < _transfer_count) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u", wpr.seq, msg->sysid);

					_transfer_seq++;

				} else if (wpr.seq == _transfer_seq - 1) {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) seq %u from ID %u (again)", wpr.seq, msg->sysid);

				} else {
					if (_transfer_seq > 0 && _transfer_seq < _transfer_count) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i or %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1, _transfer_seq);

					} else if (_transfer_seq <= 0) {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq);

					} else {
						PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u from ID %u unexpected, must be %i", wpr.seq, msg->sysid,
							  _transfer_seq - 1);
					}

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected\t");
					events::send(events::ID("mavlink_mission_wp_unexpected"), events::Log::Error,
						     "Unexpected waypoint index, aborting transfer");
					return;
				}

				/* double check bounds in case of items count changed */
				if (wpr.seq < current_item_count()) {
					send_mission_item(_transfer_partner_sysid, _transfer_partner_compid, wpr.seq);

				} else {
					PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: seq %u out of bound [%u, %u]", wpr.seq, wpr.seq,
						  current_item_count() - 1);

					switch_to_idle_state();

					send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
					_mavlink->send_statustext_critical("WPM: REJ. CMD: Req. WP was unexpected\t");
					events::send(events::ID("mavlink_mission_wp_unexpected2"), events::Log::Error,
						     "Unexpected waypoint index, aborting mission transfer");
				}

			} else if (_state == MAVLINK_WPM_STATE_IDLE) {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: no transfer");

				// Silently ignore this as some OSDs have buggy mission protocol implementations
				//_mavlink->send_statustext_critical("IGN MISSION_ITEM_REQUEST(_INT): No active transfer");

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: busy (state %d).", _state);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy\t");
				events::send(events::ID("mavlink_mission_mis_req_ignored_busy"), events::Log::Error,
					     "Ignoring mission request, currently busy");
			}

		} else {
			_mavlink->send_statustext_critical("WPM: REJ. CMD: partner id mismatch\t");
			events::send(events::ID("mavlink_mission_partner_id_mismatch2"), events::Log::Error,
				     "Rejecting mission request command, component or system ID mismatch");

			PX4_DEBUG("WPM: MISSION_ITEM_REQUEST(_INT) ERROR: rejected, partner ID mismatch");
		}
	}
}


void
MavlinkMissionManager::handle_mission_count(const mavlink_message_t *msg)
{
	mavlink_mission_count_t wpc;
	mavlink_msg_mission_count_decode(msg, &wpc);

	if (CHECK_SYSID_COMPID_MISSION(wpc)) {
		if (_state == MAVLINK_WPM_STATE_IDLE) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_in_progress) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

			_transfer_in_progress = true;
			_mission_type = (MAV_MISSION_TYPE)wpc.mission_type;

			if (wpc.count > current_max_item_count()) {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: too many waypoints (%d), supported: %d", wpc.count, current_max_item_count());

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_NO_SPACE);
				_transfer_in_progress = false;
				return;
			}

			if (wpc.count == 0) {
				PX4_DEBUG("WPM: MISSION_COUNT 0, clearing waypoints list and staying in state MAVLINK_WPM_STATE_IDLE");

				switch (_mission_type) {
				case MAV_MISSION_TYPE_MISSION:

					/* alternate dataman ID anyway to let navigator know about changes */

					if (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0) {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_1, 0, 0);

					} else {
						update_active_mission(DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
					}

					break;

				case MAV_MISSION_TYPE_FENCE:
					update_geofence_count(0);
					break;

				case MAV_MISSION_TYPE_RALLY:
					update_safepoint_count(0);
					break;

				default:
					PX4_ERR("mission type %u not handled", _mission_type);
					break;
				}

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);
				_transfer_in_progress = false;
				return;
			}

			PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u, changing state to MAVLINK_WPM_STATE_GETLIST", wpc.count, msg->sysid);

			_state = MAVLINK_WPM_STATE_GETLIST;
			_transfer_seq = 0;
			_transfer_partner_sysid = msg->sysid;
			_transfer_partner_compid = msg->compid;
			_transfer_count = wpc.count;
			_transfer_dataman_id = (_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
						DM_KEY_WAYPOINTS_OFFBOARD_0);	// use inactive storage for transmission
			_transfer_current_seq = -1;

			if (_mission_type == MAV_MISSION_TYPE_FENCE) {
				// We're about to write new geofence items, so take the lock. It will be released when
				// switching back to idle
				PX4_DEBUG("locking fence dataman items");

				int ret = dm_lock(DM_KEY_FENCE_POINTS);

				if (ret == 0) {
					_geofence_locked = true;

				} else {
					PX4_ERR("locking failed (%i)", errno);
				}
			}

		} else if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (_transfer_seq == 0) {
				/* looks like our MISSION_REQUEST was lost, try again */
				PX4_DEBUG("WPM: MISSION_COUNT %u from ID %u (again)", wpc.count, msg->sysid);

			} else {
				PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, already receiving seq %u", _transfer_seq);

				_mavlink->send_statustext_critical("WPM: REJ. CMD: Busy\t");
				events::send(events::ID("mavlink_mission_getlist_busy"), events::Log::Error,
					     "Mission upload busy, already receiving waypoint");

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
				return;
			}

		} else {
			PX4_DEBUG("WPM: MISSION_COUNT ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("WPM: IGN MISSION_COUNT: Busy\t");
			events::send(events::ID("mavlink_mission_ignore_mis_count"), events::Log::Error,
				     "Mission upload busy, ignoring MISSION_COUNT");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
	}
}

void
MavlinkMissionManager::switch_to_idle_state()
{
	// when switching to idle, we *always* check if the lock was held and release it.
	// This is to ensure we don't end up in a state where we forget to release it.
	if (_geofence_locked) {
		dm_unlock(DM_KEY_FENCE_POINTS);
		_geofence_locked = false;

		PX4_DEBUG("unlocking geofence");
	}

	_state = MAVLINK_WPM_STATE_IDLE;
}


void
MavlinkMissionManager::handle_mission_item(const mavlink_message_t *msg)
{
	if (_int_mode) {
		// It seems that we should be using the float mode, let's switch out of int mode.
		_int_mode = false;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_int(const mavlink_message_t *msg)
{
	if (!_int_mode) {
		// It seems that we should be using the int mode, let's switch to it.
		_int_mode = true;
	}

	handle_mission_item_both(msg);
}

void
MavlinkMissionManager::handle_mission_item_both(const mavlink_message_t *msg)
{

	// The mavlink_message could also contain a mavlink_mission_item_int_t. We ignore that here
	// and take care of it later in parse_mavlink_mission_item depending on _int_mode.

	mavlink_mission_item_t wp;
	mavlink_msg_mission_item_decode(msg, &wp);

	if (CHECK_SYSID_COMPID_MISSION(wp)) {

		if (wp.mission_type != _mission_type) {
			PX4_WARN("WPM: Unexpected mission type (%d %d)", (int)wp.mission_type, (int)_mission_type);
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		if (_state == MAVLINK_WPM_STATE_GETLIST) {
			_time_last_recv = hrt_absolute_time();

			if (wp.seq != _transfer_seq) {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u was not the expected %u", wp.seq, _transfer_seq);

				/* Item sequence not expected, ignore item */
				return;
			}

		} else if (_state == MAVLINK_WPM_STATE_IDLE) {
			if (_transfer_seq == wp.seq + 1) {
				// Assume this is a duplicate, where we already successfully got all mission items,
				// but the GCS did not receive the last ack and sent the same item again
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				PX4_DEBUG("WPM: MISSION_ITEM ERROR: no transfer");

				_mavlink->send_statustext_critical("IGN MISSION_ITEM: No transfer\t");
				events::send(events::ID("mavlink_mission_no_transfer"), events::Log::Error,
					     "Ignoring mission item, no transfer in progress");
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			return;

		} else {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: busy, state %i", _state);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Busy\t");
			events::send(events::ID("mavlink_mission_mis_item_busy"), events::Log::Error,
				     "Ignoring mission item, busy");
			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			return;
		}

		struct mission_item_s mission_item = {};

		int ret = parse_mavlink_mission_item(&wp, &mission_item);

		if (ret != PX4_OK) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: seq %u invalid item", wp.seq);

			_mavlink->send_statustext_critical("IGN MISSION_ITEM: Invalid item\t");
			events::send(events::ID("mavlink_mission_mis_item_invalid"), events::Log::Error,
				     "Ignoring mission item, invalid item");

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, ret);
			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		bool write_failed = false;
		bool check_failed = false;

		switch (_mission_type) {

		case MAV_MISSION_TYPE_MISSION: {
				// check that we don't get a wrong item (hardening against wrong client implementations, the list here
				// does not need to be complete)
				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_RALLY_POINT) {
					check_failed = true;

				} else {
					dm_item_t dm_item = _transfer_dataman_id;

					write_failed = dm_write(dm_item, wp.seq, &mission_item, sizeof(struct mission_item_s)) != sizeof(struct mission_item_s);

					if (!write_failed) {
						/* waypoint marked as current */
						if (wp.current) {
							_transfer_current_seq = wp.seq;
						}
					}
				}
			}
			break;

		case MAV_MISSION_TYPE_FENCE: { // Write a geofence point
				mission_fence_point_s mission_fence_point;
				mission_fence_point.nav_cmd = mission_item.nav_cmd;
				mission_fence_point.lat = mission_item.lat;
				mission_fence_point.lon = mission_item.lon;
				mission_fence_point.alt = mission_item.altitude;

				if (mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION ||
				    mission_item.nav_cmd == MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION) {
					mission_fence_point.vertex_count = mission_item.vertex_count;

					if (mission_item.vertex_count < 3) { // feasibility check
						PX4_ERR("Fence: too few vertices");
						check_failed = true;
						update_geofence_count(0);
					}

				} else {
					mission_fence_point.circle_radius = mission_item.circle_radius;
				}

				mission_fence_point.frame = mission_item.frame;

				if (!check_failed) {
					write_failed = dm_write(DM_KEY_FENCE_POINTS, wp.seq + 1, &mission_fence_point,
								sizeof(mission_fence_point_s)) != sizeof(mission_fence_point_s);
				}

			}
			break;

		case MAV_MISSION_TYPE_RALLY: { // Write a safe point / rally point
				mission_safe_point_s mission_safe_point;
				mission_safe_point.lat = mission_item.lat;
				mission_safe_point.lon = mission_item.lon;
				mission_safe_point.alt = mission_item.altitude;
				mission_safe_point.frame = mission_item.frame;
				write_failed = dm_write(DM_KEY_SAFE_POINTS, wp.seq + 1, &mission_safe_point,
							sizeof(mission_safe_point_s)) != sizeof(mission_safe_point_s);
			}
			break;

		default:
			_mavlink->send_statustext_critical("Received unknown mission type, abort.\t");
			events::send(events::ID("mavlink_mission_unknown_mis_type"), events::Log::Error,
				     "Received unknown mission type, abort");
			break;
		}

		if (write_failed || check_failed) {
			PX4_DEBUG("WPM: MISSION_ITEM ERROR: error writing seq %u to dataman ID %i", wp.seq, _transfer_dataman_id);

			send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);

			if (write_failed) {
				_mavlink->send_statustext_critical("Unable to write on micro SD\t");
				events::send(events::ID("mavlink_mission_storage_failure"), events::Log::Error,
					     "Mission: unable to write to storage");
			}

			switch_to_idle_state();
			_transfer_in_progress = false;
			return;
		}

		/* waypoint marked as current */
		if (wp.current) {
			_transfer_current_seq = wp.seq;
		}

		PX4_DEBUG("WPM: MISSION_ITEM seq %u received", wp.seq);

		_transfer_seq = wp.seq + 1;

		if (_transfer_seq == _transfer_count) {
			/* got all new mission items successfully */
			PX4_DEBUG("WPM: MISSION_ITEM got all %u items, current_seq=%ld, changing state to MAVLINK_WPM_STATE_IDLE",
				  _transfer_count, _transfer_current_seq);

			ret = 0;

			switch (_mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_transfer_dataman_id, _transfer_count, _transfer_current_seq);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(_transfer_count);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(_transfer_count);
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			// Note: the switch to idle needs to happen after update_geofence_count is called, for proper unlocking order
			switch_to_idle_state();


			if (ret == PX4_OK) {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

			_transfer_in_progress = false;

		} else {
			/* request next item */
			send_mission_request(_transfer_partner_sysid, _transfer_partner_compid, _transfer_seq);
		}
	}
}


void
MavlinkMissionManager::handle_mission_clear_all(const mavlink_message_t *msg)
{
	mavlink_mission_clear_all_t wpca;
	mavlink_msg_mission_clear_all_decode(msg, &wpca);

	if (CHECK_SYSID_COMPID_MISSION(wpca)) {

		if (_state == MAVLINK_WPM_STATE_IDLE) {
			/* don't touch mission items storage itself, but only items count in mission state */
			_time_last_recv = hrt_absolute_time();

			_mission_type = (MAV_MISSION_TYPE)wpca.mission_type; // this is needed for the returned ack
			int ret = 0;

			switch (wpca.mission_type) {
			case MAV_MISSION_TYPE_MISSION:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				break;

			case MAV_MISSION_TYPE_FENCE:
				ret = update_geofence_count(0);
				break;

			case MAV_MISSION_TYPE_RALLY:
				ret = update_safepoint_count(0);
				break;

			case MAV_MISSION_TYPE_ALL:
				ret = update_active_mission(_dataman_id == DM_KEY_WAYPOINTS_OFFBOARD_0 ? DM_KEY_WAYPOINTS_OFFBOARD_1 :
							    DM_KEY_WAYPOINTS_OFFBOARD_0, 0, 0);
				ret = update_geofence_count(0) || ret;
				ret = update_safepoint_count(0) || ret;
				break;

			default:
				PX4_ERR("mission type %u not handled", _mission_type);
				break;
			}

			if (ret == PX4_OK) {
				PX4_DEBUG("WPM: CLEAR_ALL OK (mission_type=%i)", _mission_type);

				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ACCEPTED);

			} else {
				send_mission_ack(_transfer_partner_sysid, _transfer_partner_compid, MAV_MISSION_ERROR);
			}

		} else {
			_mavlink->send_statustext_critical("WPM: IGN CLEAR CMD: Busy\t");
			events::send(events::ID("mavlink_mission_ignore_clear"), events::Log::Error,
				     "Ignoring mission clear command, busy");

			PX4_DEBUG("WPM: CLEAR_ALL IGNORED: busy");
		}
	}
}

int
MavlinkMissionManager::parse_mavlink_mission_item(const mavlink_mission_item_t *mavlink_mission_item,
		struct mission_item_s *mission_item)
{
	if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
	    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
	    (_int_mode && (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT))) {
		// This is a mission item with a global coordinate

		// Switch to int mode if that is what we are receiving
		if ((mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT ||
		     mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT)) {
			_int_mode = true;
		}

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->lat = ((double)item_int->x) * 1e-7;
			mission_item->lon = ((double)item_int->y) * 1e-7;

		} else {
			mission_item->lat = (double)mavlink_mission_item->x;
			mission_item->lon = (double)mavlink_mission_item->y;
		}

		mission_item->altitude = mavlink_mission_item->z;

		if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL ||
		    mavlink_mission_item->frame == MAV_FRAME_GLOBAL_INT) {
			mission_item->altitude_is_relative = false;

		} else if (mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT ||
			   mavlink_mission_item->frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
			mission_item->altitude_is_relative = true;
		}

		// Depending on the received MAV_CMD_* (MAVLink Commands), assign the corresponding
		// NAV_CMD value to the mission item's nav_cmd.
		switch (mavlink_mission_item->command) {
		case MAV_CMD_NAV_WAYPOINT:
			mission_item->nav_cmd = NAV_CMD_WAYPOINT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->acceptance_radius = mavlink_mission_item->param2;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_UNLIM:
			mission_item->nav_cmd = NAV_CMD_LOITER_UNLIMITED;
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LOITER_TIME:
			mission_item->nav_cmd = NAV_CMD_LOITER_TIME_LIMIT;
			mission_item->time_inside = mavlink_mission_item->param1;
			mission_item->force_heading = (mavlink_mission_item->param2 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param3;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			// Yaw is only valid for multicopter but we set it always because
			// it's just ignored for fixedwing.
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_NAV_LAND:
			mission_item->nav_cmd = NAV_CMD_LAND;
			// TODO: abort alt param1
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			mission_item->land_precision = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_TAKEOFF:
			mission_item->nav_cmd = NAV_CMD_TAKEOFF;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));

			break;

		case MAV_CMD_NAV_LOITER_TO_ALT:
			mission_item->nav_cmd = NAV_CMD_LOITER_TO_ALT;
			mission_item->force_heading = (mavlink_mission_item->param1 > 0);
			mission_item->loiter_radius = mavlink_mission_item->param2;
			mission_item->loiter_exit_xtrack = (mavlink_mission_item->param4 > 0);
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI:
			if ((int)mavlink_mission_item->param1 == MAV_ROI_LOCATION) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_LOCATION;

				mission_item->params[6] = mavlink_mission_item->z;

			} else if ((int)mavlink_mission_item->param1 == MAV_ROI_NONE) {
				mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;
				mission_item->params[0] = MAV_ROI_NONE;

			} else {
				return MAV_MISSION_INVALID_PARAM1;
			}

			break;

		case MAV_CMD_DO_SET_ROI_LOCATION:
			mission_item->nav_cmd = NAV_CMD_DO_SET_ROI_LOCATION;
			mission_item->params[6] = mavlink_mission_item->z;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->yaw = wrap_2pi(math::radians(mavlink_mission_item->param4));
			break;

		case MAV_CMD_CONDITION_GATE:
			mission_item->nav_cmd = NAV_CMD_CONDITION_GATE;
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			mission_item->nav_cmd = NAV_CMD_FENCE_RETURN_POINT;
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->vertex_count = (uint16_t)(mavlink_mission_item->param1 + 0.5f);
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			mission_item->circle_radius = mavlink_mission_item->param1;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;
			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = mavlink_mission_item->frame;

	} else if (mavlink_mission_item->frame == MAV_FRAME_MISSION) {

		// This is a mission item with no coordinates

		mission_item->params[0] = mavlink_mission_item->param1;
		mission_item->params[1] = mavlink_mission_item->param2;
		mission_item->params[2] = mavlink_mission_item->param3;
		mission_item->params[3] = mavlink_mission_item->param4;

		if (_int_mode) {
			/* The argument is actually a mavlink_mission_item_int_t in int_mode.
			 * mavlink_mission_item_t and mavlink_mission_item_int_t have the same
			 * alignment, so we can just swap float for int32_t. */
			const mavlink_mission_item_int_t *item_int
				= reinterpret_cast<const mavlink_mission_item_int_t *>(mavlink_mission_item);
			mission_item->params[4] = ((double)item_int->x);
			mission_item->params[5] = ((double)item_int->y);

		} else {
			mission_item->params[4] = (double)mavlink_mission_item->x;
			mission_item->params[5] = (double)mavlink_mission_item->y;
		}

		mission_item->params[6] = mavlink_mission_item->z;

		switch (mavlink_mission_item->command) {
		case MAV_CMD_DO_JUMP:
			mission_item->nav_cmd = NAV_CMD_DO_JUMP;
			mission_item->do_jump_mission_index = mavlink_mission_item->param1;
			mission_item->do_jump_current_count = 0;
			mission_item->do_jump_repeat_count = mavlink_mission_item->param2;
			break;

		case MAV_CMD_NAV_ROI:
		case MAV_CMD_DO_SET_ROI: {
				const int roi_mode = mavlink_mission_item->param1;

				if (roi_mode == MAV_ROI_NONE || roi_mode == MAV_ROI_WPNEXT || roi_mode == MAV_ROI_WPINDEX) {
					mission_item->nav_cmd = NAV_CMD_DO_SET_ROI;

				} else {
					return MAV_MISSION_INVALID_PARAM1;
				}
			}
			break;

		case MAV_CMD_DO_WINCH:
		case MAV_CMD_DO_GRIPPER:
			mission_item->nav_cmd = mavlink_mission_item->command;
			MavlinkMissionManager::copy_params_from_mavlink_to_mission_item(mission_item, mavlink_mission_item, 1, 2);
			break;

		case MAV_CMD_DO_CHANGE_SPEED:
		case MAV_CMD_DO_SET_HOME:
		case MAV_CMD_DO_LAND_START:
		case MAV_CMD_DO_TRIGGER_CONTROL:
		case MAV_CMD_DO_DIGICAM_CONTROL:
		case MAV_CMD_DO_MOUNT_CONFIGURE:
		case MAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case MAV_CMD_IMAGE_START_CAPTURE:
		case MAV_CMD_IMAGE_STOP_CAPTURE:
		case MAV_CMD_VIDEO_START_CAPTURE:
		case MAV_CMD_VIDEO_STOP_CAPTURE:
		case MAV_CMD_DO_CONTROL_VIDEO:
		case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case MAV_CMD_OBLIQUE_SURVEY:
		case MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case MAV_CMD_SET_CAMERA_MODE:
		case MAV_CMD_DO_VTOL_TRANSITION:
		case MAV_CMD_NAV_DELAY:
		case MAV_CMD_NAV_RETURN_TO_LAUNCH:
		case MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET:
		case MAV_CMD_DO_SET_ROI_NONE:
		case MAV_CMD_CONDITION_DELAY:
		case MAV_CMD_CONDITION_DISTANCE:
		case MAV_CMD_DO_SET_ACTUATOR:
			mission_item->nav_cmd = (NAV_CMD)mavlink_mission_item->command;
			break;

		default:
			mission_item->nav_cmd = NAV_CMD_INVALID;

			PX4_DEBUG("Unsupported command %d", mavlink_mission_item->command);

			return MAV_MISSION_UNSUPPORTED;
		}

		mission_item->frame = MAV_FRAME_MISSION;

	} else {
		PX4_DEBUG("Unsupported frame %d", mavlink_mission_item->frame);

		return MAV_MISSION_UNSUPPORTED_FRAME;
	}

	mission_item->autocontinue = mavlink_mission_item->autocontinue;
	// mission_item->index = mavlink_mission_item->seq;

	mission_item->origin = ORIGIN_MAVLINK;

	return MAV_MISSION_ACCEPTED;
}


int
MavlinkMissionManager::format_mavlink_mission_item(const struct mission_item_s *mission_item,
		mavlink_mission_item_t *mavlink_mission_item)
{
	mavlink_mission_item->frame = mission_item->frame;
	mavlink_mission_item->command = mission_item->nav_cmd;
	mavlink_mission_item->autocontinue = mission_item->autocontinue;
	mavlink_mission_item->mission_type = _mission_type;

	/* default mappings for generic commands */
	if (mission_item->frame == MAV_FRAME_MISSION) {
		mavlink_mission_item->param1 = mission_item->params[0];
		mavlink_mission_item->param2 = mission_item->params[1];
		mavlink_mission_item->param3 = mission_item->params[2];
		mavlink_mission_item->param4 = mission_item->params[3];

		mavlink_mission_item->x = mission_item->params[4];
		mavlink_mission_item->y = mission_item->params[5];

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->params[4]);
			item_int->y = round(mission_item->params[5]);

		} else {
			mavlink_mission_item->x = (float)mission_item->params[4];
			mavlink_mission_item->y = (float)mission_item->params[5];
		}

		mavlink_mission_item->z = mission_item->params[6];

		switch (mavlink_mission_item->command) {
		case NAV_CMD_DO_JUMP:
			mavlink_mission_item->param1 = mission_item->do_jump_mission_index;
			mavlink_mission_item->param2 = mission_item->do_jump_repeat_count;
			break;

		case NAV_CMD_DO_CHANGE_SPEED:
		case NAV_CMD_DO_SET_HOME:
		case NAV_CMD_DO_LAND_START:
		case NAV_CMD_DO_TRIGGER_CONTROL:
		case NAV_CMD_DO_DIGICAM_CONTROL:
		case NAV_CMD_IMAGE_START_CAPTURE:
		case NAV_CMD_IMAGE_STOP_CAPTURE:
		case NAV_CMD_VIDEO_START_CAPTURE:
		case NAV_CMD_VIDEO_STOP_CAPTURE:
		case NAV_CMD_DO_CONTROL_VIDEO:
		case NAV_CMD_DO_MOUNT_CONFIGURE:
		case NAV_CMD_DO_MOUNT_CONTROL:
		case NAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW:
		case NAV_CMD_DO_GIMBAL_MANAGER_CONFIGURE:
		case NAV_CMD_DO_SET_ROI:
		case NAV_CMD_DO_SET_CAM_TRIGG_DIST:
		case NAV_CMD_OBLIQUE_SURVEY:
		case NAV_CMD_DO_SET_CAM_TRIGG_INTERVAL:
		case NAV_CMD_SET_CAMERA_MODE:
		case NAV_CMD_SET_CAMERA_ZOOM:
		case NAV_CMD_SET_CAMERA_FOCUS:
		case NAV_CMD_DO_VTOL_TRANSITION:
			break;

		default:
			return PX4_ERROR;
		}

	} else {
		mavlink_mission_item->param1 = 0.0f;
		mavlink_mission_item->param2 = 0.0f;
		mavlink_mission_item->param3 = 0.0f;
		mavlink_mission_item->param4 = 0.0f;

		if (_int_mode) {
			// This function actually receives a mavlink_mission_item_int_t in _int_mode
			// which has the same alignment as mavlink_mission_item_t and the only
			// difference is int32_t vs. float for x and y.
			mavlink_mission_item_int_t *item_int =
				reinterpret_cast<mavlink_mission_item_int_t *>(mavlink_mission_item);

			item_int->x = round(mission_item->lat * 1e7);
			item_int->y = round(mission_item->lon * 1e7);

		} else {
			mavlink_mission_item->x = (float)mission_item->lat;
			mavlink_mission_item->y = (float)mission_item->lon;
		}

		mavlink_mission_item->z = mission_item->altitude;

		if (mission_item->altitude_is_relative) {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			}

		} else {
			if (_int_mode) {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL_INT;

			} else {
				mavlink_mission_item->frame = MAV_FRAME_GLOBAL;
			}
		}

		switch (mission_item->nav_cmd) {
		case NAV_CMD_WAYPOINT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->acceptance_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_UNLIMITED:
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TIME_LIMIT:
			mavlink_mission_item->param1 = mission_item->time_inside;
			mavlink_mission_item->param2 = mission_item->force_heading;
			mavlink_mission_item->param3 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case NAV_CMD_LAND:
			// TODO: param1 abort alt
			mavlink_mission_item->param2 = mission_item->land_precision;
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_TAKEOFF:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case NAV_CMD_LOITER_TO_ALT:
			mavlink_mission_item->param1 = mission_item->force_heading;
			mavlink_mission_item->param2 = mission_item->loiter_radius;
			mavlink_mission_item->param4 = mission_item->loiter_exit_xtrack;
			break;

		case MAV_CMD_NAV_VTOL_TAKEOFF:
		case MAV_CMD_NAV_VTOL_LAND:
			mavlink_mission_item->param4 = math::degrees(mission_item->yaw);
			break;

		case MAV_CMD_NAV_FENCE_RETURN_POINT:
			break;

		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION:
		case MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION:
			mavlink_mission_item->param1 = (float)mission_item->vertex_count;
			break;

		case MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION:
		case MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION:
			mavlink_mission_item->param1 = mission_item->circle_radius;
			break;

		case MAV_CMD_NAV_RALLY_POINT:
			break;


		default:
			return PX4_ERROR;
		}
	}

	return PX4_OK;
}

void MavlinkMissionManager::copy_params_from_mavlink_to_mission_item(struct mission_item_s *mission_item,
		const mavlink_mission_item_t *mavlink_mission_item, int8_t start_idx, int8_t end_idx)
{
	// Copy each param1 ~ 7 if they are within the range specified
	if (start_idx <= 1 && 1 <= end_idx) {
		mission_item->params[0] = mavlink_mission_item->param1;
	}

	if (start_idx <= 2 && 2 <= end_idx) {
		mission_item->params[1] = mavlink_mission_item->param2;
	}

	if (start_idx <= 3 && 3 <= end_idx) {
		mission_item->params[2] = mavlink_mission_item->param3;
	}

	if (start_idx <= 4 && 4 <= end_idx) {
		mission_item->params[3] = mavlink_mission_item->param4;
	}

	/* Param5, 6 and 7 are named x, y and z since it is used as position coordinates as well */
	if (start_idx <= 5 && 5 <= end_idx) {
		mission_item->params[4] = mavlink_mission_item->x;
	}

	if (start_idx <= 6 && 6 <= end_idx) {
		mission_item->params[5] = mavlink_mission_item->y;
	}

	if (start_idx <= 7 && 7 <= end_idx) {
		mission_item->params[6] = mavlink_mission_item->z;
	}
}

void MavlinkMissionManager::check_active_mission()
{
	// do not send anything over high latency communication
	if (_mavlink->get_mode() == Mavlink::MAVLINK_MODE_IRIDIUM) {
		return;
	}

	if (!(_my_dataman_id == _dataman_id)) {
		PX4_DEBUG("WPM: New mission detected (possibly over different Mavlink instance) Updating");

		_my_dataman_id = _dataman_id;
		send_mission_count(_transfer_partner_sysid, _transfer_partner_compid, _count[MAV_MISSION_TYPE_MISSION],
				   MAV_MISSION_TYPE_MISSION);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_parameters.cpp
 * Mavlink parameters manager implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Beat Kueng <beat@px4.io>
 */

#include <stdio.h>

#include "mavlink_parameters.h"
#include "mavlink_main.h"
#include <lib/systemlib/mavlink_log.h>

MavlinkParametersManager::MavlinkParametersManager(Mavlink *mavlink) :
	_mavlink(mavlink)
{
}

unsigned
MavlinkParametersManager::get_size()
{
	return MAVLINK_MSG_ID_PARAM_VALUE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES;
}

void
MavlinkParametersManager::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
			/* request all parameters */
			mavlink_param_request_list_t req_list;
			mavlink_msg_param_request_list_decode(msg, &req_list);

			if (req_list.target_system == mavlink_system.sysid &&
			    (req_list.target_component == mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				if (_send_all_index < 0) {
					_send_all_index = PARAM_HASH;

				} else {
					/* a restart should skip the hash check on the ground */
					_send_all_index = 0;
				}
			}

#if defined(CONFIG_MAVLINK_UAVCAN_PARAMETERS)

			if (req_list.target_system == mavlink_system.sysid && req_list.target_component < 127 &&
			    (req_list.target_component != mavlink_system.compid || req_list.target_component == MAV_COMP_ID_ALL)) {
				// publish list request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.message_type = msg->msgid;
				req.node_id = req_list.target_component;
				req.param_index = 0;
				req.timestamp = hrt_absolute_time();
				_uavcan_parameter_request_pub.publish(req);
			}

#endif // CONFIG_MAVLINK_UAVCAN_PARAMETERS
			break;
		}

	case MAVLINK_MSG_ID_PARAM_SET: {
			/* set parameter */
			mavlink_param_set_t set;
			mavlink_msg_param_set_decode(msg, &set);

			if (set.target_system == mavlink_system.sysid &&
			    (set.target_component == mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {

				/* local name buffer to enforce null-terminated string */
				char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
				strncpy(name, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				/* Whatever the value is, we're being told to stop sending */
				if (strncmp(name, "_HASH_CHECK", sizeof(name)) == 0) {

					if (_mavlink->hash_check_enabled()) {
						_send_all_index = -1;
					}

					/* No other action taken, return */
					return;
				}

				/* attempt to find parameter, set and send it */
				param_t param = param_find_no_notification(name);

				if (param == PARAM_INVALID) {
					PX4_ERR("unknown param: %s", name);

				} else if (!((param_type(param) == PARAM_TYPE_INT32 && set.param_type == MAV_PARAM_TYPE_INT32) ||
					     (param_type(param) == PARAM_TYPE_FLOAT && set.param_type == MAV_PARAM_TYPE_REAL32))) {
					PX4_ERR("param types mismatch param: %s", name);

				} else {
					// According to the mavlink spec we should always acknowledge a write operation.
					param_set(param, &(set.param_value));
					send_param(param);
				}
			}

#if defined(CONFIG_MAVLINK_UAVCAN_PARAMETERS)

			if (set.target_system == mavlink_system.sysid && set.target_component < 127 &&
			    (set.target_component != mavlink_system.compid || set.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.message_type = msg->msgid;
				req.node_id = set.target_component;
				req.param_index = -1;
				strncpy(req.param_id, set.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				if (set.param_type == MAV_PARAM_TYPE_REAL32) {
					req.param_type = MAV_PARAM_TYPE_REAL32;
					req.real_value = set.param_value;

				} else {
					int32_t val;
					memcpy(&val, &set.param_value, sizeof(int32_t));
					req.param_type = MAV_PARAM_TYPE_INT64;
					req.int_value = val;
				}

				req.timestamp = hrt_absolute_time();
				_uavcan_parameter_request_pub.publish(req);
			}

#endif // CONFIG_MAVLINK_UAVCAN_PARAMETERS
			break;
		}

	case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
			/* request one parameter */
			mavlink_param_request_read_t req_read;
			mavlink_msg_param_request_read_decode(msg, &req_read);

			if (req_read.target_system == mavlink_system.sysid &&
			    (req_read.target_component == mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {

				/* when no index is given, loop through string ids and compare them */
				if (req_read.param_index < 0) {
					/* XXX: I left this in so older versions of QGC wouldn't break */
					if (strncmp(req_read.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
						/* return hash check for cached params */
						uint32_t hash = param_hash_check();

						/* build the one-off response message */
						mavlink_param_value_t param_value;
						param_value.param_count = param_count_used();
						param_value.param_index = -1;
						strncpy(param_value.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						param_value.param_type = MAV_PARAM_TYPE_UINT32;
						memcpy(&param_value.param_value, &hash, sizeof(hash));
						mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &param_value);

					} else {
						/* local name buffer to enforce null-terminated string */
						char name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1];
						strncpy(name, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
						/* enforce null termination */
						name[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';
						/* attempt to find parameter and send it */
						send_param(param_find_no_notification(name));
					}

				} else {
					/* when index is >= 0, send this parameter again */
					int ret = send_param(param_for_used_index(req_read.param_index));

					if (ret == 1) {
						PX4_ERR("unknown param ID: %i", req_read.param_index);

					} else if (ret == 2) {
						PX4_ERR("failed loading param from storage ID: %i", req_read.param_index);
					}
				}
			}

#if defined(CONFIG_MAVLINK_UAVCAN_PARAMETERS)

			if (req_read.target_system == mavlink_system.sysid && req_read.target_component < 127 &&
			    (req_read.target_component != mavlink_system.compid || req_read.target_component == MAV_COMP_ID_ALL)) {
				// publish set request to UAVCAN driver via uORB.
				uavcan_parameter_request_s req{};
				req.timestamp = hrt_absolute_time();
				req.message_type = msg->msgid;
				req.node_id = req_read.target_component;
				req.param_index = req_read.param_index;
				strncpy(req.param_id, req_read.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1);
				req.param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

				// Enque the request and forward the first to the uavcan node
				enque_uavcan_request(&req);
				request_next_uavcan_parameter();
			}

#endif // CONFIG_MAVLINK_UAVCAN_PARAMETERS
			break;
		}

	case MAVLINK_MSG_ID_PARAM_MAP_RC: {
			/* map a rc channel to a parameter */
			mavlink_param_map_rc_t map_rc;
			mavlink_msg_param_map_rc_decode(msg, &map_rc);

			if (map_rc.target_system == mavlink_system.sysid &&
			    (map_rc.target_component == mavlink_system.compid ||
			     map_rc.target_component == MAV_COMP_ID_ALL)) {

				/* Copy values from msg to uorb using the parameter_rc_channel_index as index */
				size_t i = map_rc.parameter_rc_channel_index;

				if (i >= sizeof(_rc_param_map.param_index) / sizeof(_rc_param_map.param_index[0])) {
					mavlink_log_warning(_mavlink->get_mavlink_log_pub(), "parameter_rc_channel_index out of bounds\t");
					events::send(events::ID("mavlink_param_rc_chan_out_of_bounds"), events::Log::Warning,
						     "parameter_rc_channel_index out of bounds");
					break;
				}

				_rc_param_map.param_index[i] = map_rc.param_index;
				strncpy(&(_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1)]), map_rc.param_id,
					MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
				/* enforce null termination */
				_rc_param_map.param_id[i * (rc_parameter_map_s::PARAM_ID_LEN + 1) + rc_parameter_map_s::PARAM_ID_LEN] = '\0';
				_rc_param_map.scale[i] = map_rc.scale;
				_rc_param_map.value0[i] = map_rc.param_value0;
				_rc_param_map.value_min[i] = map_rc.param_value_min;
				_rc_param_map.value_max[i] = map_rc.param_value_max;

				if (map_rc.param_index == -2) { // -2 means unset map
					_rc_param_map.valid[i] = false;

				} else {
					_rc_param_map.valid[i] = true;
				}

				_rc_param_map.timestamp = hrt_absolute_time();
				_rc_param_map_pub.publish(_rc_param_map);
			}

			break;
		}

	default:
		break;
	}
}

void
MavlinkParametersManager::send()
{
	if (!_first_send) {
		// parameters QGC can't tolerate not finding (2020-11-11)
		param_find("BAT_CRIT_THR");
		param_find("BAT_EMERGEN_THR");
		param_find("BAT_LOW_THR");
		param_find("BAT_N_CELLS");     // deprecated
		param_find("BAT_V_CHARGED");   // deprecated
		param_find("BAT_V_EMPTY");     // deprecated
		param_find("BAT_V_LOAD_DROP"); // deprecated
		param_find("CAL_ACC0_ID");
		param_find("CAL_GYRO0_ID");
		param_find("CAL_MAG0_ID");
		param_find("CAL_MAG0_ROT");
		param_find("CAL_MAG1_ID");
		param_find("CAL_MAG1_ROT");
		param_find("CAL_MAG2_ID");
		param_find("CAL_MAG2_ROT");
		param_find("CAL_MAG3_ID");
		param_find("CAL_MAG3_ROT");
		param_find("SENS_BOARD_ROT");
		param_find("SENS_BOARD_X_OFF");
		param_find("SENS_BOARD_Y_OFF");
		param_find("SENS_BOARD_Z_OFF");
		param_find("SENS_DPRES_OFF");
		param_find("TRIG_MODE");
		param_find("UAVCAN_ENABLE");

		_first_send = true;
	}

	int max_num_to_send;

	if (_mavlink->get_protocol() == Protocol::SERIAL && !_mavlink->is_usb_uart()) {
		max_num_to_send = 3;

	} else {
		// speed up parameter loading via UDP or USB: try to send 20 at once
		max_num_to_send = 20;
	}

	int i = 0;

	// Send while burst is not exceeded, we still have buffer space and still something to send
	while ((i++ < max_num_to_send) && (_mavlink->get_free_tx_buf() >= get_size()) && !_mavlink->radio_status_critical()
	       && send_params()) {}
}

bool
MavlinkParametersManager::send_params()
{
#if defined(CONFIG_MAVLINK_UAVCAN_PARAMETERS)

	if (send_uavcan()) {
		return true;
	}

#endif // CONFIG_MAVLINK_UAVCAN_PARAMETERS

	if (send_one()) {
		return true;

	} else if (send_untransmitted()) {
		return true;
	}

	return false;
}

bool
MavlinkParametersManager::send_untransmitted()
{
	bool sent_one = false;

	if (_parameter_update_sub.updated()) {
		// clear the update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// Schedule an update if not already the case
		if (_param_update_time == 0) {
			_param_update_time = pupdate.timestamp;
			_param_update_index = 0;
		}
	}

	if ((_param_update_time != 0) && ((_param_update_time + 5 * 1000) < hrt_absolute_time())) {

		param_t param = 0;

		// send out all changed values
		do {
			// skip over all parameters which are not invalid and not used
			do {
				param = param_for_index(_param_update_index);
				++_param_update_index;
			} while (param != PARAM_INVALID && !param_used(param));

			// send parameters which are untransmitted while there is
			// space in the TX buffer
			if ((param != PARAM_INVALID) && param_value_unsaved(param)) {
				int ret = send_param(param);
				sent_one = true;

				if (ret != PX4_OK) {
					break;
				}
			}
		} while ((_mavlink->get_free_tx_buf() >= get_size()) && !_mavlink->radio_status_critical()
			 && (_param_update_index < (int) param_count()));

		// Flag work as done once all params have been sent
		if (_param_update_index >= (int) param_count()) {
			_param_update_time = 0;
		}
	}

	return sent_one;
}

bool
MavlinkParametersManager::send_one()
{
	if (_send_all_index >= 0) {
		/* send all parameters if requested, but only after the system has booted */

		/* The first thing we send is a hash of all values for the ground
		 * station to try and quickly load a cached copy of our params
		 */
		if (_send_all_index == PARAM_HASH) {
			/* return hash check for cached params */
			uint32_t hash = param_hash_check();

			/* build the one-off response message */
			mavlink_param_value_t msg;
			msg.param_count = param_count_used();
			msg.param_index = -1;
			strncpy(msg.param_id, HASH_PARAM, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
			msg.param_type = MAV_PARAM_TYPE_UINT32;
			memcpy(&msg.param_value, &hash, sizeof(hash));
			mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

			/* after this we should start sending all params */
			_send_all_index = 0;

			/* No further action, return now */
			return true;
		}

		/* look for the first parameter which is used */
		param_t p;

		do {
			/* walk through all parameters, including unused ones */
			p = param_for_index(_send_all_index);
			_send_all_index++;
		} while (p != PARAM_INVALID && !param_used(p));

		if (p != PARAM_INVALID) {
			send_param(p);
		}

		if ((p == PARAM_INVALID) || (_send_all_index >= (int) param_count())) {
			_send_all_index = -1;
			return false;

		} else {
			return true;
		}
	}

	return false;
}

int
MavlinkParametersManager::send_param(param_t param, int component_id)
{
	if (param == PARAM_INVALID) {
		return 1;
	}

	/* no free TX buf to send this param */
	if (_mavlink->get_free_tx_buf() < MAVLINK_MSG_ID_PARAM_VALUE_LEN) {
		return 1;
	}

	mavlink_param_value_t msg;

	/*
	 * get param value, since MAVLink encodes float and int params in the same
	 * space during transmission, copy param onto float val_buf
	 */
	if (param_type(param) == PARAM_TYPE_INT32) {
		int32_t param_value;

		if (param_get(param, &param_value) != OK) {
			return 2;
		}

		memcpy(&msg.param_value, &param_value, sizeof(param_value));

	} else {
		float param_value;

		if (param_get(param, &param_value) != OK) {
			return 2;
		}

		msg.param_value = param_value;
	}

	msg.param_count = param_count_used();
	msg.param_index = param_get_used_index(param);

#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
	/*
	 * coverity[buffer_size_warning : FALSE]
	 *
	 * The MAVLink spec does not require the string to be NUL-terminated if it
	 * has length 16. In this case the receiving end needs to terminate it
	 * when copying it.
	 */
	strncpy(msg.param_id, param_name(param), MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

	/* query parameter type */
	param_type_t type = param_type(param);

	/*
	 * Map onboard parameter type to MAVLink type,
	 * endianess matches (both little endian)
	 */
	if (type == PARAM_TYPE_INT32) {
		msg.param_type = MAVLINK_TYPE_INT32_T;

	} else if (type == PARAM_TYPE_FLOAT) {
		msg.param_type = MAVLINK_TYPE_FLOAT;

	} else {
		msg.param_type = MAVLINK_TYPE_FLOAT;
	}

	/* default component ID */
	if (component_id < 0) {
		mavlink_msg_param_value_send_struct(_mavlink->get_channel(), &msg);

	} else {
		// Re-pack the message with a different component ID
		mavlink_message_t mavlink_packet;
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, component_id, _mavlink->get_channel(), &mavlink_packet, &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);
	}

	return 0;
}

#if defined(CONFIG_MAVLINK_UAVCAN_PARAMETERS)

bool MavlinkParametersManager::send_uavcan()
{
	/* Send parameter values received from the UAVCAN topic */
	uavcan_parameter_value_s value{};

	if (_uavcan_parameter_value_sub.update(&value)) {

		// Check if we received a matching parameter, drop it from the list and request the next
		if ((_uavcan_open_request_list != nullptr)
		    && (value.param_index == _uavcan_open_request_list->req.param_index)
		    && (value.node_id == _uavcan_open_request_list->req.node_id)) {

			dequeue_uavcan_request();
			request_next_uavcan_parameter();
		}

		mavlink_param_value_t msg{};
		msg.param_count = value.param_count;
		msg.param_index = value.param_index;
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic ignored "-Wstringop-truncation"
#endif
		/*
		 * coverity[buffer_size_warning : FALSE]
		 *
		 * The MAVLink spec does not require the string to be NUL-terminated if it
		 * has length 16. In this case the receiving end needs to terminate it
		 * when copying it.
		 */
		strncpy(msg.param_id, value.param_id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
#if defined(__GNUC__) && __GNUC__ >= 8
#pragma GCC diagnostic pop
#endif

		if (value.param_type == MAV_PARAM_TYPE_REAL32) {
			msg.param_type = MAVLINK_TYPE_FLOAT;
			msg.param_value = value.real_value;

		} else {
			int32_t val = (int32_t)value.int_value;
			memcpy(&msg.param_value, &val, sizeof(int32_t));
			msg.param_type = MAVLINK_TYPE_INT32_T;
		}

		// Re-pack the message with the UAVCAN node ID
		mavlink_message_t mavlink_packet{};
		mavlink_msg_param_value_encode_chan(mavlink_system.sysid, value.node_id, _mavlink->get_channel(), &mavlink_packet,
						    &msg);
		_mavlink_resend_uart(_mavlink->get_channel(), &mavlink_packet);

		return true;
	}

	return false;
}

void MavlinkParametersManager::request_next_uavcan_parameter()
{
	// Request a parameter if we are not already waiting on a response and if the list is not empty
	if (!_uavcan_waiting_for_request_response && _uavcan_open_request_list != nullptr) {
		uavcan_parameter_request_s req = _uavcan_open_request_list->req;

		_uavcan_parameter_request_pub.publish(req);

		_uavcan_waiting_for_request_response = true;
	}
}

void MavlinkParametersManager::enque_uavcan_request(uavcan_parameter_request_s *req)
{
	// We store at max 10 requests to keep memory consumption low.
	// Dropped requests will be repeated by the ground station
	if (_uavcan_queued_request_items >= 10) {
		return;
	}

	_uavcan_open_request_list_item *new_reqest = new _uavcan_open_request_list_item;
	new_reqest->req = *req;
	new_reqest->next = nullptr;

	_uavcan_open_request_list_item *item = _uavcan_open_request_list;
	++_uavcan_queued_request_items;

	if (item == nullptr) {
		// Add the first item to the list
		_uavcan_open_request_list = new_reqest;

	} else {
		// Find the last item and add the new request at the end
		while (item->next != nullptr) {
			item = item->next;
		}

		item->next = new_reqest;
	}
}

void MavlinkParametersManager::dequeue_uavcan_request()
{
	if (_uavcan_open_request_list != nullptr) {
		// Drop the first item in the list and free the used memory
		_uavcan_open_request_list_item *first = _uavcan_open_request_list;
		_uavcan_open_request_list = first->next;
		--_uavcan_queued_request_items;
		delete first;
		_uavcan_waiting_for_request_response = false;
	}
}

#endif // CONFIG_MAVLINK_UAVCAN_PARAMETERS
/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file mavlink_rate_limiter.cpp
 * Message rate limiter implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include "mavlink_rate_limiter.h"

bool
MavlinkRateLimiter::check(const hrt_abstime &t)
{
	uint64_t dt = t - _last_sent;

	if (dt > 0 && dt >= _interval) {
		_last_sent = t;
		return true;
	}

	return false;
}
/****************************************************************************
 *
 *   Copyright (c) 2012-2021 PX4 Development Team. All rights reserved.
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
 * @file mavlink_receiver.cpp
 * MAVLink protocol message receive and dispatch
 *
 * @author Lorenz Meier <lorenz@px4.io>
 * @author Anton Babushkin <anton@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

#include <lib/airspeed/airspeed.h>
#include <lib/conversion/rotation.h>
#include <lib/systemlib/px4_macros.h>

#include <math.h>
#include <poll.h>

#ifdef CONFIG_NET
#include <net/if.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#endif

#ifndef __PX4_POSIX
#include <termios.h>
#endif

#include "mavlink_command_sender.h"
#include "mavlink_main.h"
#include "mavlink_receiver.h"

#include <lib/drivers/device/Device.hpp> // For DeviceId union

#ifdef CONFIG_NET
#define MAVLINK_RECEIVER_NET_ADDED_STACK 1360
#else
#define MAVLINK_RECEIVER_NET_ADDED_STACK 0
#endif

MavlinkReceiver::~MavlinkReceiver()
{
	delete _tune_publisher;
	delete _px4_accel;
	delete _px4_gyro;
	delete _px4_mag;
#if !defined(CONSTRAINED_FLASH)
	delete[] _received_msg_stats;
#endif // !CONSTRAINED_FLASH
}

static constexpr vehicle_odometry_s vehicle_odometry_empty {
	.timestamp = 0,
	.timestamp_sample = 0,
	.position = {NAN, NAN, NAN},
	.q = {NAN, NAN, NAN, NAN},
	.velocity = {NAN, NAN, NAN},
	.angular_velocity = {NAN, NAN, NAN},
	.position_variance = {NAN, NAN, NAN},
	.orientation_variance = {NAN, NAN, NAN},
	.velocity_variance = {NAN, NAN, NAN},
	.pose_frame = vehicle_odometry_s::POSE_FRAME_UNKNOWN,
	.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_UNKNOWN,
	.reset_counter = 0,
	.quality = 0
};

MavlinkReceiver::MavlinkReceiver(Mavlink *parent) :
	ModuleParams(nullptr),
	_mavlink(parent),
	_mavlink_ftp(parent),
	_mavlink_log_handler(parent),
	_mission_manager(parent),
	_parameters_manager(parent),
	_mavlink_timesync(parent)
{
}

void
MavlinkReceiver::acknowledge(uint8_t sysid, uint8_t compid, uint16_t command, uint8_t result, uint8_t progress)
{
	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.command = command;
	command_ack.result = result;
	command_ack.target_system = sysid;
	command_ack.target_component = compid;
	command_ack.result_param1 = progress;

	_cmd_ack_pub.publish(command_ack);
}

void
MavlinkReceiver::handle_message(mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_COMMAND_LONG:
		handle_message_command_long(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_INT:
		handle_message_command_int(msg);
		break;

	case MAVLINK_MSG_ID_COMMAND_ACK:
		handle_message_command_ack(msg);
		break;

	case MAVLINK_MSG_ID_OPTICAL_FLOW_RAD:
		handle_message_optical_flow_rad(msg);
		break;

	case MAVLINK_MSG_ID_PING:
		handle_message_ping(msg);
		break;

	case MAVLINK_MSG_ID_SET_MODE:
		handle_message_set_mode(msg);
		break;

	case MAVLINK_MSG_ID_ATT_POS_MOCAP:
		handle_message_att_pos_mocap(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED:
		handle_message_set_position_target_local_ned(msg);
		break;

	case MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT:
		handle_message_set_position_target_global_int(msg);
		break;

	case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
		handle_message_set_attitude_target(msg);
		break;

	case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
		handle_message_vision_position_estimate(msg);
		break;

	case MAVLINK_MSG_ID_ODOMETRY:
		handle_message_odometry(msg);
		break;

	case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
		handle_message_set_gps_global_origin(msg);
		break;

	case MAVLINK_MSG_ID_RADIO_STATUS:
		handle_message_radio_status(msg);
		break;

	case MAVLINK_MSG_ID_MANUAL_CONTROL:
		handle_message_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
		handle_message_rc_channels_override(msg);
		break;

	case MAVLINK_MSG_ID_HEARTBEAT:
		handle_message_heartbeat(msg);
		break;

	case MAVLINK_MSG_ID_DISTANCE_SENSOR:
		handle_message_distance_sensor(msg);
		break;

	case MAVLINK_MSG_ID_FOLLOW_TARGET:
		handle_message_follow_target(msg);
		break;

	case MAVLINK_MSG_ID_LANDING_TARGET:
		handle_message_landing_target(msg);
		break;

	case MAVLINK_MSG_ID_CELLULAR_STATUS:
		handle_message_cellular_status(msg);
		break;

	case MAVLINK_MSG_ID_ADSB_VEHICLE:
		handle_message_adsb_vehicle(msg);
		break;

	case MAVLINK_MSG_ID_UTM_GLOBAL_POSITION:
		handle_message_utm_global_position(msg);
		break;

	case MAVLINK_MSG_ID_COLLISION:
		handle_message_collision(msg);
		break;

	case MAVLINK_MSG_ID_GPS_RTCM_DATA:
		handle_message_gps_rtcm_data(msg);
		break;

	case MAVLINK_MSG_ID_BATTERY_STATUS:
		handle_message_battery_status(msg);
		break;

	case MAVLINK_MSG_ID_SERIAL_CONTROL:
		handle_message_serial_control(msg);
		break;

	case MAVLINK_MSG_ID_LOGGING_ACK:
		handle_message_logging_ack(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE:
		handle_message_play_tune(msg);
		break;

	case MAVLINK_MSG_ID_PLAY_TUNE_V2:
		handle_message_play_tune_v2(msg);
		break;

	case MAVLINK_MSG_ID_OBSTACLE_DISTANCE:
		handle_message_obstacle_distance(msg);
		break;

	case MAVLINK_MSG_ID_TUNNEL:
		handle_message_tunnel(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER:
		handle_message_trajectory_representation_bezier(msg);
		break;

	case MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS:
		handle_message_trajectory_representation_waypoints(msg);
		break;

	case MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS:
		handle_message_onboard_computer_status(msg);
		break;

	case MAVLINK_MSG_ID_GENERATOR_STATUS:
		handle_message_generator_status(msg);
		break;

	case MAVLINK_MSG_ID_STATUSTEXT:
		handle_message_statustext(msg);
		break;

#if !defined(CONSTRAINED_FLASH)

	case MAVLINK_MSG_ID_NAMED_VALUE_FLOAT:
		handle_message_named_value_float(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG:
		handle_message_debug(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_VECT:
		handle_message_debug_vect(msg);
		break;

	case MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY:
		handle_message_debug_float_array(msg);
		break;
#endif // !CONSTRAINED_FLASH

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_ATTITUDE:
		handle_message_gimbal_manager_set_attitude(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_MANAGER_SET_MANUAL_CONTROL:
		handle_message_gimbal_manager_set_manual_control(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION:
		handle_message_gimbal_device_information(msg);
		break;

	case MAVLINK_MSG_ID_REQUEST_EVENT:
		handle_message_request_event(msg);
		break;

	case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS:
		handle_message_gimbal_device_attitude_status(msg);
		break;

	default:
		break;
	}

	/*
	 * Only decode hil messages in HIL mode.
	 *
	 * The HIL mode is enabled by the HIL bit flag
	 * in the system mode. Either send a set mode
	 * COMMAND_LONG message or a SET_MODE message
	 *
	 * Accept HIL GPS messages if use_hil_gps flag is true.
	 * This allows to provide fake gps measurements to the system.
	 */
	if (_mavlink->get_hil_enabled()) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_SENSOR:
			handle_message_hil_sensor(msg);
			break;

		case MAVLINK_MSG_ID_HIL_STATE_QUATERNION:
			handle_message_hil_state_quaternion(msg);
			break;

		case MAVLINK_MSG_ID_HIL_OPTICAL_FLOW:
			handle_message_hil_optical_flow(msg);
			break;

		default:
			break;
		}
	}


	if (_mavlink->get_hil_enabled() || (_mavlink->get_use_hil_gps() && msg->sysid == mavlink_system.sysid)) {
		switch (msg->msgid) {
		case MAVLINK_MSG_ID_HIL_GPS:
			handle_message_hil_gps(msg);
			break;

		default:
			break;
		}

	}

	/* If we've received a valid message, mark the flag indicating so.
	   This is used in the '-w' command-line flag. */
	_mavlink->set_has_received_messages(true);
}

bool
MavlinkReceiver::evaluate_target_ok(int command, int target_system, int target_component)
{
	/* evaluate if this system should accept this command */
	bool target_ok = false;

	switch (command) {

	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
	case MAV_CMD_REQUEST_PROTOCOL_VERSION:
		/* broadcast and ignore component */
		target_ok = (target_system == 0) || (target_system == mavlink_system.sysid);
		break;

	default:
		target_ok = (target_system == mavlink_system.sysid) && ((target_component == mavlink_system.compid)
				|| (target_component == MAV_COMP_ID_ALL));
		break;
	}

	return target_ok;
}

void
MavlinkReceiver::handle_message_command_long(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_long_t cmd_mavlink;
	mavlink_msg_command_long_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();

	const float before_int32_max = nextafterf((float)INT32_MAX, 0.0f);
	const float after_int32_max = nextafterf((float)INT32_MAX, (float)INFINITY);

	if (cmd_mavlink.param5 >= before_int32_max && cmd_mavlink.param5 <= after_int32_max &&
	    cmd_mavlink.param6 >= before_int32_max && cmd_mavlink.param6 <= after_int32_max) {
		// This looks suspicously like INT32_MAX was sent in a COMMAND_LONG instead of
		// a COMMAND_INT.
		PX4_ERR("param5/param6 invalid of command %" PRIu16, cmd_mavlink.command);
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

	/* Copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;
	vcmd.param5 = (double)cmd_mavlink.param5;
	vcmd.param6 = (double)cmd_mavlink.param6;
	vcmd.param7 = cmd_mavlink.param7;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = cmd_mavlink.confirmation;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

void
MavlinkReceiver::handle_message_command_int(mavlink_message_t *msg)
{
	/* command */
	mavlink_command_int_t cmd_mavlink;
	mavlink_msg_command_int_decode(msg, &cmd_mavlink);

	vehicle_command_s vcmd{};
	vcmd.timestamp = hrt_absolute_time();

	if (cmd_mavlink.x == 0x7ff80000 && cmd_mavlink.y == 0x7ff80000) {
		// This looks like NAN was by accident sent as int.
		PX4_ERR("x/y invalid of command %" PRIu16, cmd_mavlink.command);
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
		return;
	}

	/* Copy the content of mavlink_command_int_t cmd_mavlink into command_t cmd */
	vcmd.param1 = cmd_mavlink.param1;
	vcmd.param2 = cmd_mavlink.param2;
	vcmd.param3 = cmd_mavlink.param3;
	vcmd.param4 = cmd_mavlink.param4;

	if (cmd_mavlink.x == INT32_MAX && cmd_mavlink.y == INT32_MAX) {
		// INT32_MAX for x and y means to ignore it.
		vcmd.param5 = (double)NAN;
		vcmd.param6 = (double)NAN;

	} else {
		vcmd.param5 = ((double)cmd_mavlink.x) / 1e7;
		vcmd.param6 = ((double)cmd_mavlink.y) / 1e7;
	}

	vcmd.param7 = cmd_mavlink.z;
	vcmd.command = cmd_mavlink.command;
	vcmd.target_system = cmd_mavlink.target_system;
	vcmd.target_component = cmd_mavlink.target_component;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = false;
	vcmd.from_external = true;

	handle_message_command_both(msg, cmd_mavlink, vcmd);
}

template <class T>
void MavlinkReceiver::handle_message_command_both(mavlink_message_t *msg, const T &cmd_mavlink,
		const vehicle_command_s &vehicle_command)
{
	bool target_ok = evaluate_target_ok(cmd_mavlink.command, cmd_mavlink.target_system, cmd_mavlink.target_component);
	bool send_ack = true;
	uint8_t result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
	uint8_t progress = 0; // TODO: should be 255, 0 for backwards compatibility

	if (!target_ok) {
		// Reject alien commands only if there is no forwarding or we've never seen target component before
		if (!_mavlink->get_forwarding_on()
		    || !_mavlink->component_was_seen(cmd_mavlink.target_system, cmd_mavlink.target_component, _mavlink)) {
			acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED);
		}

		return;
	}

	// First we handle legacy support requests which were used before we had
	// the generic MAV_CMD_REQUEST_MESSAGE.
	if (cmd_mavlink.command == MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES) {
		result = handle_request_message_command(MAVLINK_MSG_ID_AUTOPILOT_VERSION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_PROTOCOL_VERSION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_PROTOCOL_VERSION);

	} else if (cmd_mavlink.command == MAV_CMD_GET_HOME_POSITION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_HOME_POSITION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_FLIGHT_INFORMATION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_FLIGHT_INFORMATION);

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_STORAGE_INFORMATION) {
		result = handle_request_message_command(MAVLINK_MSG_ID_STORAGE_INFORMATION);

	} else if (cmd_mavlink.command == MAV_CMD_SET_MESSAGE_INTERVAL) {
		if (set_message_interval((int)roundf(cmd_mavlink.param1), cmd_mavlink.param2, cmd_mavlink.param3)) {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
		}

	} else if (cmd_mavlink.command == MAV_CMD_GET_MESSAGE_INTERVAL) {
		get_message_interval((int)roundf(cmd_mavlink.param1));

	} else if (cmd_mavlink.command == MAV_CMD_REQUEST_MESSAGE) {

		uint16_t message_id = (uint16_t)roundf(vehicle_command.param1);
		result = handle_request_message_command(message_id,
							vehicle_command.param2, vehicle_command.param3, vehicle_command.param4,
							vehicle_command.param5, vehicle_command.param6, vehicle_command.param7);

	} else if (cmd_mavlink.command == MAV_CMD_INJECT_FAILURE) {
		if (_mavlink->failure_injection_enabled()) {
			_cmd_pub.publish(vehicle_command);
			send_ack = false;

		} else {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
			send_ack = true;
		}

	} else if (cmd_mavlink.command == MAV_CMD_DO_AUTOTUNE_ENABLE) {

		bool has_module = true;
		autotune_attitude_control_status_s status{};
		_autotune_attitude_control_status_sub.copy(&status);

		// if not busy enable via the parameter
		// do not check the return value of the uORB copy above because the module
		// starts publishing only when MC_AT_START is set
		if (status.state == autotune_attitude_control_status_s::STATE_IDLE) {
			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (!vehicle_status.in_transition_mode) {
				param_t atune_start;

				switch (vehicle_status.vehicle_type) {
				case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
					atune_start = param_find("FW_AT_START");

					break;

				case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
					atune_start = param_find("MC_AT_START");

					break;

				default:
					atune_start = PARAM_INVALID;
					break;
				}

				if (atune_start == PARAM_INVALID) {
					has_module = false;

				} else {
					int32_t start = 1;
					param_set(atune_start, &start);
				}

			} else {
				has_module = false;
			}
		}

		if (has_module) {

			// most are in progress
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_IN_PROGRESS;

			switch (status.state) {
			case autotune_attitude_control_status_s::STATE_IDLE:
			case autotune_attitude_control_status_s::STATE_INIT:
				progress = 0;
				break;

			case autotune_attitude_control_status_s::STATE_ROLL:
			case autotune_attitude_control_status_s::STATE_ROLL_PAUSE:
				progress = 20;
				break;

			case autotune_attitude_control_status_s::STATE_PITCH:
			case autotune_attitude_control_status_s::STATE_PITCH_PAUSE:
				progress = 40;
				break;

			case autotune_attitude_control_status_s::STATE_YAW:
			case autotune_attitude_control_status_s::STATE_YAW_PAUSE:
				progress = 60;
				break;

			case autotune_attitude_control_status_s::STATE_VERIFICATION:
				progress = 80;
				break;

			case autotune_attitude_control_status_s::STATE_APPLY:
				progress = 85;
				break;

			case autotune_attitude_control_status_s::STATE_TEST:
				progress = 90;
				break;

			case autotune_attitude_control_status_s::STATE_WAIT_FOR_DISARM:
				progress = 95;
				break;

			case autotune_attitude_control_status_s::STATE_COMPLETE:
				progress = 100;
				// ack it properly with an ACCEPTED once we're done
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED;
				break;

			case autotune_attitude_control_status_s::STATE_FAIL:
				progress = 0;
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_FAILED;
				break;
			}

		} else {
			result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED;
		}

		send_ack = true;

	} else {
		send_ack = false;

		if (msg->sysid == mavlink_system.sysid && msg->compid == mavlink_system.compid) {
			PX4_WARN("ignoring CMD with same SYS/COMP (%" PRIu8 "/%" PRIu8 ") ID", mavlink_system.sysid, mavlink_system.compid);
			return;
		}

		if (cmd_mavlink.command == MAV_CMD_LOGGING_START) {
			// check that we have enough bandwidth available: this is given by the configured logger topics
			// and rates. The 5000 is somewhat arbitrary, but makes sure that we cannot enable log streaming
			// on a radio link
			if (_mavlink->get_data_rate() < 5000) {
				send_ack = true;
				result = vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED;
				_mavlink->send_statustext_critical("Not enough bandwidth to enable log streaming\t");
				events::send<uint32_t>(events::ID("mavlink_log_not_enough_bw"), events::Log::Error,
						       "Not enough bandwidth to enable log streaming ({1} \\< 5000)", _mavlink->get_data_rate());

			} else {
				// we already instanciate the streaming object, because at this point we know on which
				// mavlink channel streaming was requested. But in fact it's possible that the logger is
				// not even running. The main mavlink thread takes care of this by waiting for an ack
				// from the logger.
				_mavlink->try_start_ulog_streaming(msg->sysid, msg->compid);
			}
		}

		if (!send_ack) {
			_cmd_pub.publish(vehicle_command);
		}
	}

	if (send_ack) {
		acknowledge(msg->sysid, msg->compid, cmd_mavlink.command, result, progress);
	}
}

uint8_t MavlinkReceiver::handle_request_message_command(uint16_t message_id, float param2, float param3, float param4,
		float param5, float param6, float param7)
{
	bool stream_found = false;
	bool message_sent = false;

	for (const auto &stream : _mavlink->get_streams()) {
		if (stream->get_id() == message_id) {
			stream_found = true;
			message_sent = stream->request_message(param2, param3, param4, param5, param6, param7);
			break;
		}
	}

	if (!stream_found) {
		// If we don't find the stream, we can configure it with rate 0 and then trigger it once.
		const char *stream_name = get_stream_name(message_id);

		if (stream_name != nullptr) {
			_mavlink->configure_stream_threadsafe(stream_name, 0.0f);

			// Now we try again to send it.
			for (const auto &stream : _mavlink->get_streams()) {
				if (stream->get_id() == message_id) {
					message_sent = stream->request_message(param2, param3, param4, param5, param6, param7);
					break;
				}
			}
		}
	}

	return (message_sent ? vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED :
		vehicle_command_ack_s::VEHICLE_CMD_RESULT_DENIED);
}


void
MavlinkReceiver::handle_message_command_ack(mavlink_message_t *msg)
{
	mavlink_command_ack_t ack;
	mavlink_msg_command_ack_decode(msg, &ack);

	MavlinkCommandSender::instance().handle_mavlink_command_ack(ack, msg->sysid, msg->compid, _mavlink->get_channel());

	vehicle_command_ack_s command_ack{};

	command_ack.timestamp = hrt_absolute_time();
	command_ack.result_param2 = ack.result_param2;
	command_ack.command = ack.command;
	command_ack.result = ack.result;
	command_ack.from_external = true;
	command_ack.result_param1 = ack.progress;
	command_ack.target_system = ack.target_system;
	command_ack.target_component = ack.target_component;

	_cmd_ack_pub.publish(command_ack);

	// TODO: move it to the same place that sent the command
	if (ack.result == MAV_RESULT_FAILED) {
		if (msg->compid == MAV_COMP_ID_CAMERA) {
			PX4_WARN("Got unsuccessful result %" PRIu8 " from camera", ack.result);
		}
	}
}

void
MavlinkReceiver::handle_message_optical_flow_rad(mavlink_message_t *msg)
{
	mavlink_optical_flow_rad_t flow;
	mavlink_msg_optical_flow_rad_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink->get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_MAVLINK;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void
MavlinkReceiver::handle_message_hil_optical_flow(mavlink_message_t *msg)
{
	mavlink_hil_optical_flow_t flow;
	mavlink_msg_hil_optical_flow_decode(msg, &flow);

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink->get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_FLOW_DEVTYPE_SIM;

	sensor_optical_flow_s sensor_optical_flow{};

	sensor_optical_flow.timestamp_sample = hrt_absolute_time();
	sensor_optical_flow.device_id = device_id.devid;

	sensor_optical_flow.pixel_flow[0] = flow.integrated_x;
	sensor_optical_flow.pixel_flow[1] = flow.integrated_y;

	sensor_optical_flow.integration_timespan_us = flow.integration_time_us;
	sensor_optical_flow.quality = flow.quality;

	const matrix::Vector3f integrated_gyro(flow.integrated_xgyro, flow.integrated_ygyro, flow.integrated_zgyro);

	if (integrated_gyro.isAllFinite()) {
		integrated_gyro.copyTo(sensor_optical_flow.delta_angle);
		sensor_optical_flow.delta_angle_available = true;
	}

	sensor_optical_flow.max_flow_rate       = NAN;
	sensor_optical_flow.min_ground_distance = NAN;
	sensor_optical_flow.max_ground_distance = NAN;

	// Use distance value for distance sensor topic
	if (PX4_ISFINITE(flow.distance) && (flow.distance >= 0.f)) {
		// Positive value (including zero): distance known. Negative value: Unknown distance.
		sensor_optical_flow.distance_m = flow.distance;
		sensor_optical_flow.distance_available = true;
	}

	sensor_optical_flow.timestamp = hrt_absolute_time();

	_sensor_optical_flow_pub.publish(sensor_optical_flow);
}

void
MavlinkReceiver::handle_message_set_mode(mavlink_message_t *msg)
{
	mavlink_set_mode_t new_mode;
	mavlink_msg_set_mode_decode(msg, &new_mode);

	union px4_custom_mode custom_mode;
	custom_mode.data = new_mode.custom_mode;

	vehicle_command_s vcmd{};

	vcmd.timestamp = hrt_absolute_time();

	/* copy the content of mavlink_command_long_t cmd_mavlink into command_t cmd */
	vcmd.param1 = (float)new_mode.base_mode;
	vcmd.param2 = (float)custom_mode.main_mode;
	vcmd.param3 = (float)custom_mode.sub_mode;

	vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_SET_MODE;
	vcmd.target_system = new_mode.target_system;
	vcmd.target_component = MAV_COMP_ID_ALL;
	vcmd.source_system = msg->sysid;
	vcmd.source_component = msg->compid;
	vcmd.confirmation = true;
	vcmd.from_external = true;

	_cmd_pub.publish(vcmd);
}

void
MavlinkReceiver::handle_message_distance_sensor(mavlink_message_t *msg)
{
	mavlink_distance_sensor_t dist_sensor;
	mavlink_msg_distance_sensor_decode(msg, &dist_sensor);

	distance_sensor_s ds{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink->get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_MAVLINK;

	ds.timestamp        = hrt_absolute_time(); /* Use system time for now, don't trust sender to attach correct timestamp */
	ds.min_distance     = static_cast<float>(dist_sensor.min_distance) * 1e-2f;     /* cm to m */
	ds.max_distance     = static_cast<float>(dist_sensor.max_distance) * 1e-2f;     /* cm to m */
	ds.current_distance = static_cast<float>(dist_sensor.current_distance) * 1e-2f; /* cm to m */
	ds.variance         = dist_sensor.covariance * 1e-4f;                           /* cm^2 to m^2 */
	ds.h_fov            = dist_sensor.horizontal_fov;
	ds.v_fov            = dist_sensor.vertical_fov;
	ds.q[0]             = dist_sensor.quaternion[0];
	ds.q[1]             = dist_sensor.quaternion[1];
	ds.q[2]             = dist_sensor.quaternion[2];
	ds.q[3]             = dist_sensor.quaternion[3];
	ds.type             = dist_sensor.type;
	ds.device_id        = device_id.devid;
	ds.orientation      = dist_sensor.orientation;

	// MAVLink DISTANCE_SENSOR signal_quality value of 0 means unset/unknown
	// quality value. Also it comes normalised between 1 and 100 while the uORB
	// signal quality is normalised between 0 and 100.
	ds.signal_quality = dist_sensor.signal_quality == 0 ? -1 : 100 * (dist_sensor.signal_quality - 1) / 99;

	_distance_sensor_pub.publish(ds);
}

void
MavlinkReceiver::handle_message_att_pos_mocap(mavlink_message_t *msg)
{
	mavlink_att_pos_mocap_t att_pos_mocap;
	mavlink_msg_att_pos_mocap_decode(msg, &att_pos_mocap);

	// fill vehicle_odometry from Mavlink ATT_POS_MOCAP
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(att_pos_mocap.time_usec);

	odom.q[0] = att_pos_mocap.q[0];
	odom.q[1] = att_pos_mocap.q[1];
	odom.q[2] = att_pos_mocap.q[2];
	odom.q[3] = att_pos_mocap.q[3];

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = att_pos_mocap.x;
	odom.position[1] = att_pos_mocap.y;
	odom.position[2] = att_pos_mocap.z;

	// ATT_POS_MOCAP covariance
	//  Row-major representation of a pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = att_pos_mocap.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = att_pos_mocap.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = att_pos_mocap.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = att_pos_mocap.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = att_pos_mocap.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = att_pos_mocap.covariance[20]; // Y  row 5, col 5

	odom.timestamp = hrt_absolute_time();

	_mocap_odometry_pub.publish(odom);
}

void
MavlinkReceiver::handle_message_set_position_target_local_ned(mavlink_message_t *msg)
{
	mavlink_set_position_target_local_ned_t target_local_ned;
	mavlink_msg_set_position_target_local_ned_decode(msg, &target_local_ned);

	/* Only accept messages which are intended for this system */
	if (_mavlink->get_forward_externalsp() &&
	    (mavlink_system.sysid == target_local_ned.target_system || target_local_ned.target_system == 0) &&
	    (mavlink_system.compid == target_local_ned.target_component || target_local_ned.target_component == 0)) {

		trajectory_setpoint_s setpoint{};

		const uint16_t type_mask = target_local_ned.type_mask;

		if (target_local_ned.coordinate_frame == MAV_FRAME_LOCAL_NED) {
			setpoint.position[0] = (type_mask & POSITION_TARGET_TYPEMASK_X_IGNORE) ? (float)NAN : target_local_ned.x;
			setpoint.position[1] = (type_mask & POSITION_TARGET_TYPEMASK_Y_IGNORE) ? (float)NAN : target_local_ned.y;
			setpoint.position[2] = (type_mask & POSITION_TARGET_TYPEMASK_Z_IGNORE) ? (float)NAN : target_local_ned.z;

			setpoint.velocity[0] = (type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? (float)NAN : target_local_ned.vx;
			setpoint.velocity[1] = (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? (float)NAN : target_local_ned.vy;
			setpoint.velocity[2] = (type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? (float)NAN : target_local_ned.vz;

			setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_local_ned.afx;
			setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_local_ned.afy;
			setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_local_ned.afz;

		} else if (target_local_ned.coordinate_frame == MAV_FRAME_BODY_NED) {

			vehicle_attitude_s vehicle_attitude{};
			_vehicle_attitude_sub.copy(&vehicle_attitude);
			const matrix::Dcmf R{matrix::Quatf{vehicle_attitude.q}};

			const bool ignore_velocity = type_mask & (POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE |
						     POSITION_TARGET_TYPEMASK_VZ_IGNORE);

			if (!ignore_velocity) {
				const matrix::Vector3f velocity_body_sp{
					(type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? 0.f : target_local_ned.vx,
					(type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? 0.f : target_local_ned.vy,
					(type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? 0.f : target_local_ned.vz
				};


				const float yaw = matrix::Eulerf{R}(2);

				setpoint.velocity[0] = cosf(yaw) * velocity_body_sp(0) - sinf(yaw) * velocity_body_sp(1);
				setpoint.velocity[1] = sinf(yaw) * velocity_body_sp(0) + cosf(yaw) * velocity_body_sp(1);
				setpoint.velocity[2] = velocity_body_sp(2);

			} else {
				matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.velocity);
			}

			const bool ignore_acceleration = type_mask & (POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE |
							 POSITION_TARGET_TYPEMASK_AZ_IGNORE);

			if (!ignore_acceleration) {
				const matrix::Vector3f acceleration_body_sp{
					(type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? 0.f : target_local_ned.afx,
					(type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? 0.f : target_local_ned.afy,
					(type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? 0.f : target_local_ned.afz
				};

				const matrix::Vector3f acceleration_setpoint{R * acceleration_body_sp};
				acceleration_setpoint.copyTo(setpoint.acceleration);

			} else {
				matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.acceleration);
			}

			matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.position);

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED coordinate frame %" PRIu8 " unsupported\t",
					     target_local_ned.coordinate_frame);
			events::send<uint8_t>(events::ID("mavlink_rcv_sp_target_unsup_coord"), events::Log::Error,
					      "SET_POSITION_TARGET_LOCAL_NED: coordinate frame {1} unsupported", target_local_ned.coordinate_frame);
			return;
		}

		setpoint.yaw      = (type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)      ? (float)NAN : target_local_ned.yaw;
		setpoint.yawspeed = (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) ? (float)NAN : target_local_ned.yaw_rate;


		offboard_control_mode_s ocm{};
		ocm.position = !matrix::Vector3f(setpoint.position).isAllNan();
		ocm.velocity = !matrix::Vector3f(setpoint.velocity).isAllNan();
		ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();

		if (ocm.acceleration && (type_mask & POSITION_TARGET_TYPEMASK_FORCE_SET)) {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED force not supported\t");
			events::send(events::ID("mavlink_rcv_sp_target_unsup_force"), events::Log::Error,
				     "SET_POSITION_TARGET_LOCAL_NED: FORCE is not supported");
			return;
		}

		if (ocm.position || ocm.velocity || ocm.acceleration) {
			// publish offboard_control_mode
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// only publish setpoint once in OFFBOARD
				setpoint.timestamp = hrt_absolute_time();
				_trajectory_setpoint_pub.publish(setpoint);
			}

		} else {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_LOCAL_NED invalid\t");
			events::send(events::ID("mavlink_rcv_sp_target_invalid"), events::Log::Error,
				     "SET_POSITION_TARGET_LOCAL_NED: invalid, missing position, velocity or acceleration");
		}
	}
}

void
MavlinkReceiver::handle_message_set_position_target_global_int(mavlink_message_t *msg)
{
	mavlink_set_position_target_global_int_t target_global_int;
	mavlink_msg_set_position_target_global_int_decode(msg, &target_global_int);

	/* Only accept messages which are intended for this system */
	if (_mavlink->get_forward_externalsp() &&
	    (mavlink_system.sysid == target_global_int.target_system || target_global_int.target_system == 0) &&
	    (mavlink_system.compid == target_global_int.target_component || target_global_int.target_component == 0)) {

		trajectory_setpoint_s setpoint{};

		const uint16_t type_mask = target_global_int.type_mask;

		// position
		if (!(type_mask & (POSITION_TARGET_TYPEMASK_X_IGNORE | POSITION_TARGET_TYPEMASK_Y_IGNORE |
				   POSITION_TARGET_TYPEMASK_Z_IGNORE))) {

			vehicle_local_position_s local_pos{};
			_vehicle_local_position_sub.copy(&local_pos);

			if (!local_pos.xy_global || !local_pos.z_global) {
				return;
			}

			MapProjection global_local_proj_ref{local_pos.ref_lat, local_pos.ref_lon, local_pos.ref_timestamp};

			// global -> local
			const double lat = target_global_int.lat_int / 1e7;
			const double lon = target_global_int.lon_int / 1e7;
			global_local_proj_ref.project(lat, lon, setpoint.position[0], setpoint.position[1]);

			if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_INT) {
				setpoint.position[2] = local_pos.ref_alt - target_global_int.alt;

			} else if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT) {
				home_position_s home_position{};
				_home_position_sub.copy(&home_position);

				if (home_position.valid_alt) {
					const float alt = home_position.alt - target_global_int.alt;
					setpoint.position[2] = alt - local_pos.ref_alt;

				} else {
					// home altitude required
					return;
				}

			} else if (target_global_int.coordinate_frame == MAV_FRAME_GLOBAL_TERRAIN_ALT_INT) {
				vehicle_global_position_s vehicle_global_position{};
				_vehicle_global_position_sub.copy(&vehicle_global_position);

				if (vehicle_global_position.terrain_alt_valid) {
					const float alt = target_global_int.alt + vehicle_global_position.terrain_alt;
					setpoint.position[2] = local_pos.ref_alt - alt;

				} else {
					// valid terrain alt required
					return;
				}

			} else {
				mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_GLOBAL_INT invalid coordinate frame %" PRIu8 "\t",
						     target_global_int.coordinate_frame);
				events::send<uint8_t>(events::ID("mavlink_rcv_sp_target_gl_invalid_coord"), events::Log::Error,
						      "SET_POSITION_TARGET_GLOBAL_INT invalid coordinate frame {1}", target_global_int.coordinate_frame);
				return;
			}

		} else {
			matrix::Vector3f(NAN, NAN, NAN).copyTo(setpoint.position);
		}

		// velocity
		setpoint.velocity[0] = (type_mask & POSITION_TARGET_TYPEMASK_VX_IGNORE) ? (float)NAN : target_global_int.vx;
		setpoint.velocity[1] = (type_mask & POSITION_TARGET_TYPEMASK_VY_IGNORE) ? (float)NAN : target_global_int.vy;
		setpoint.velocity[2] = (type_mask & POSITION_TARGET_TYPEMASK_VZ_IGNORE) ? (float)NAN : target_global_int.vz;

		// acceleration
		setpoint.acceleration[0] = (type_mask & POSITION_TARGET_TYPEMASK_AX_IGNORE) ? (float)NAN : target_global_int.afx;
		setpoint.acceleration[1] = (type_mask & POSITION_TARGET_TYPEMASK_AY_IGNORE) ? (float)NAN : target_global_int.afy;
		setpoint.acceleration[2] = (type_mask & POSITION_TARGET_TYPEMASK_AZ_IGNORE) ? (float)NAN : target_global_int.afz;

		setpoint.yaw      = (type_mask & POSITION_TARGET_TYPEMASK_YAW_IGNORE)      ? (float)NAN : target_global_int.yaw;
		setpoint.yawspeed = (type_mask & POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE) ? (float)NAN : target_global_int.yaw_rate;


		offboard_control_mode_s ocm{};
		ocm.position = !matrix::Vector3f(setpoint.position).isAllNan();
		ocm.velocity = !matrix::Vector3f(setpoint.velocity).isAllNan();
		ocm.acceleration = !matrix::Vector3f(setpoint.acceleration).isAllNan();

		if (ocm.acceleration && (type_mask & POSITION_TARGET_TYPEMASK_FORCE_SET)) {
			mavlink_log_critical(&_mavlink_log_pub, "SET_POSITION_TARGET_GLOBAL_INT force not supported\t");
			events::send(events::ID("mavlink_rcv_sp_target_gl_unsup_force"), events::Log::Error,
				     "SET_POSITION_TARGET_GLOBAL_INT: FORCE is not supported");
			return;
		}

		if (ocm.position || ocm.velocity || ocm.acceleration) {
			// publish offboard_control_mode
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);

			vehicle_status_s vehicle_status{};
			_vehicle_status_sub.copy(&vehicle_status);

			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				// only publish setpoint once in OFFBOARD
				setpoint.timestamp = hrt_absolute_time();
				_trajectory_setpoint_pub.publish(setpoint);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_set_gps_global_origin(mavlink_message_t *msg)
{
	mavlink_set_gps_global_origin_t gps_global_origin;
	mavlink_msg_set_gps_global_origin_decode(msg, &gps_global_origin);

	if (gps_global_origin.target_system == _mavlink->get_system_id()) {
		vehicle_command_s vcmd{};
		vcmd.param5 = (double)gps_global_origin.latitude * 1.e-7;
		vcmd.param6 = (double)gps_global_origin.longitude * 1.e-7;
		vcmd.param7 = (float)gps_global_origin.altitude * 1.e-3f;
		vcmd.command = vehicle_command_s::VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN;
		vcmd.target_system = _mavlink->get_system_id();
		vcmd.target_component = MAV_COMP_ID_ALL;
		vcmd.source_system = msg->sysid;
		vcmd.source_component = msg->compid;
		vcmd.confirmation = false;
		vcmd.from_external = true;
		vcmd.timestamp = hrt_absolute_time();
		_cmd_pub.publish(vcmd);
	}

	handle_request_message_command(MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN);
}

void
MavlinkReceiver::handle_message_vision_position_estimate(mavlink_message_t *msg)
{
	mavlink_vision_position_estimate_t vpe;
	mavlink_msg_vision_position_estimate_decode(msg, &vpe);

	// fill vehicle_odometry from Mavlink VISION_POSITION_ESTIMATE
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(vpe.usec);

	odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
	odom.position[0] = vpe.x;
	odom.position[1] = vpe.y;
	odom.position[2] = vpe.z;

	const matrix::Quatf q(matrix::Eulerf(vpe.roll, vpe.pitch, vpe.yaw));
	q.copyTo(odom.q);

	// VISION_POSITION_ESTIMATE covariance
	//  Row-major representation of pose 6x6 cross-covariance matrix upper right triangle
	//  (states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.).
	//  If unknown, assign NaN value to first element in the array.
	odom.position_variance[0] = vpe.covariance[0];  // X  row 0, col 0
	odom.position_variance[1] = vpe.covariance[6];  // Y  row 1, col 1
	odom.position_variance[2] = vpe.covariance[11]; // Z  row 2, col 2

	odom.orientation_variance[0] = vpe.covariance[15]; // R  row 3, col 3
	odom.orientation_variance[1] = vpe.covariance[18]; // P  row 4, col 4
	odom.orientation_variance[2] = vpe.covariance[20]; // Y  row 5, col 5

	odom.reset_counter = vpe.reset_counter;

	odom.timestamp = hrt_absolute_time();

	_visual_odometry_pub.publish(odom);
}

void
MavlinkReceiver::handle_message_odometry(mavlink_message_t *msg)
{
	mavlink_odometry_t odom_in;
	mavlink_msg_odometry_decode(msg, &odom_in);

	// fill vehicle_odometry from Mavlink ODOMETRY
	vehicle_odometry_s odom{vehicle_odometry_empty};

	odom.timestamp_sample = _mavlink_timesync.sync_stamp(odom_in.time_usec);

	const matrix::Vector3f odom_in_p(odom_in.x, odom_in.y, odom_in.z);

	// position x/y/z (m)
	if (odom_in_p.isAllFinite()) {
		// frame_id: Coordinate frame of reference for the pose data.
		switch (odom_in.frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_NED;
			odom.position[0] =  odom_in.y; // y: North
			odom.position[1] =  odom_in.x; // x: East
			odom.position[2] = -odom_in.z; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom_in_p.copyTo(odom.position);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.pose_frame = vehicle_odometry_s::POSE_FRAME_FRD;
			odom.position[0] =  odom_in.x; // x: Forward
			odom.position[1] = -odom_in.y; // y: Left
			odom.position[2] = -odom_in.z; // z: Up
			break;

		default:
			break;
		}

		// pose_covariance
		//  Row-major representation of a 6x6 pose cross-covariance matrix upper right triangle (states: x, y, z, roll, pitch, yaw)
		//  first six entries are the first ROW, next five entries are the second ROW, etc.
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
				// position variances copied directly
				odom.position_variance[0] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[1] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.position_variance[0] = odom_in.pose_covariance[6];  // Y  row 1, col 1
				odom.position_variance[1] = odom_in.pose_covariance[0];  // X  row 0, col 0
				odom.position_variance[2] = odom_in.pose_covariance[11]; // Z  row 2, col 2
				break;

			default:
				break;
			}
		}
	}

	// q: the quaternion of the ODOMETRY msg represents a rotation from body frame to a local frame
	if (matrix::Quatf(odom_in.q).isAllFinite()) {

		odom.q[0] = odom_in.q[0];
		odom.q[1] = odom_in.q[1];
		odom.q[2] = odom_in.q[2];
		odom.q[3] = odom_in.q[3];

		// pose_covariance (roll, pitch, yaw)
		//  states: x, y, z, roll, pitch, yaw; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix pose_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			odom.orientation_variance[0] = odom_in.pose_covariance[15]; // R  row 3, col 3
			odom.orientation_variance[1] = odom_in.pose_covariance[18]; // P  row 4, col 4
			odom.orientation_variance[2] = odom_in.pose_covariance[20]; // Y  row 5, col 5
		}
	}

	const matrix::Vector3f odom_in_v(odom_in.vx, odom_in.vy, odom_in.vz);

	// velocity vx/vy/vz (m/s)
	if (odom_in_v.isAllFinite()) {
		// child_frame_id: Coordinate frame of reference for the velocity in free space (twist) data.
		switch (odom_in.child_frame_id) {
		case MAV_FRAME_LOCAL_NED:
			// NED local tangent frame (x: North, y: East, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_ENU:
			// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_NED;
			odom.velocity[0] =  odom_in.vy; // y: East
			odom.velocity[1] =  odom_in.vx; // x: North
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_LOCAL_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom_in_v.copyTo(odom.velocity);
			break;

		case MAV_FRAME_LOCAL_FLU:
			// FLU local tangent frame (x: Forward, y: Left, z: Up) with origin fixed relative to earth.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_FRD;
			odom.velocity[0] =  odom_in.vx; // x: Forward
			odom.velocity[1] = -odom_in.vy; // y: Left
			odom.velocity[2] = -odom_in.vz; // z: Up
			break;

		case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
		case MAV_FRAME_BODY_FRD:
			// FRD local tangent frame (x: Forward, y: Right, z: Down) with origin that travels with vehicle.
			odom.velocity_frame = vehicle_odometry_s::VELOCITY_FRAME_BODY_FRD;
			odom.velocity[0] = odom_in.vx;
			odom.velocity[1] = odom_in.vy;
			odom.velocity[2] = odom_in.vz;
			break;

		default:
			// unsupported child_frame_id
			break;
		}

		// velocity_covariance (vx, vy, vz)
		//  states: vx, vy, vz, rollspeed, pitchspeed, yawspeed; first six entries are the first ROW, next five entries are the second ROW, etc.
		//  TODO: fix velocity_covariance for MAV_FRAME_LOCAL_ENU, MAV_FRAME_LOCAL_FLU, MAV_FRAME_LOCAL_FLU
		if (odom_in.estimator_type != MAV_ESTIMATOR_TYPE_NAIVE) {
			switch (odom_in.child_frame_id) {
			case MAV_FRAME_LOCAL_NED:
			case MAV_FRAME_LOCAL_FRD:
			case MAV_FRAME_LOCAL_FLU:
			case MAV_FRAME_BODY_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_OFFSET_NED: // DEPRECATED: Replaced by MAV_FRAME_BODY_FRD (2019-08).
			case MAV_FRAME_BODY_FRD:
				// velocity covariances copied directly
				odom.velocity_variance[0] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[1] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			case MAV_FRAME_LOCAL_ENU:
				// ENU local tangent frame (x: East, y: North, z: Up) with origin fixed relative to earth.
				odom.velocity_variance[0] = odom_in.velocity_covariance[6];  // Y  row 1, col 1
				odom.velocity_variance[1] = odom_in.velocity_covariance[0];  // X  row 0, col 0
				odom.velocity_variance[2] = odom_in.velocity_covariance[11]; // Z  row 2, col 2
				break;

			default:
				// unsupported child_frame_id
				break;
			}
		}
	}

	// Roll/Pitch/Yaw angular speed (rad/s)
	if (PX4_ISFINITE(odom_in.rollspeed)
	    && PX4_ISFINITE(odom_in.pitchspeed)
	    && PX4_ISFINITE(odom_in.yawspeed)) {

		odom.angular_velocity[0] = odom_in.rollspeed;
		odom.angular_velocity[1] = odom_in.pitchspeed;
		odom.angular_velocity[2] = odom_in.yawspeed;
	}

	odom.reset_counter = odom_in.reset_counter;
	odom.quality = odom_in.quality;

	switch (odom_in.estimator_type) {
	case MAV_ESTIMATOR_TYPE_UNKNOWN: // accept MAV_ESTIMATOR_TYPE_UNKNOWN for legacy support
	case MAV_ESTIMATOR_TYPE_NAIVE:
	case MAV_ESTIMATOR_TYPE_VISION:
	case MAV_ESTIMATOR_TYPE_VIO:
		odom.timestamp = hrt_absolute_time();
		_visual_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_MOCAP:
		odom.timestamp = hrt_absolute_time();
		_mocap_odometry_pub.publish(odom);
		break;

	case MAV_ESTIMATOR_TYPE_GPS:
	case MAV_ESTIMATOR_TYPE_GPS_INS:
	case MAV_ESTIMATOR_TYPE_LIDAR:
	case MAV_ESTIMATOR_TYPE_AUTOPILOT:
	default:
		mavlink_log_critical(&_mavlink_log_pub, "ODOMETRY: estimator_type %" PRIu8 " unsupported\t",
				     odom_in.estimator_type);
		events::send<uint8_t>(events::ID("mavlink_rcv_odom_unsup_estimator_type"), events::Log::Error,
				      "ODOMETRY: unsupported estimator_type {1}", odom_in.estimator_type);
		return;
	}
}

void MavlinkReceiver::fill_thrust(float *thrust_body_array, uint8_t vehicle_type, float thrust)
{
	// Fill correct field by checking frametype
	// TODO: add as needed
	switch (_mavlink->get_system_type()) {
	case MAV_TYPE_GENERIC:
		break;

	case MAV_TYPE_FIXED_WING:
	case MAV_TYPE_GROUND_ROVER:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_QUADROTOR:
	case MAV_TYPE_HEXAROTOR:
	case MAV_TYPE_OCTOROTOR:
	case MAV_TYPE_TRICOPTER:
	case MAV_TYPE_HELICOPTER:
	case MAV_TYPE_COAXIAL:
		thrust_body_array[2] = -thrust;
		break;

	case MAV_TYPE_SUBMARINE:
		thrust_body_array[0] = thrust;
		break;

	case MAV_TYPE_VTOL_TAILSITTER_DUOROTOR:
	case MAV_TYPE_VTOL_TAILSITTER_QUADROTOR:
	case MAV_TYPE_VTOL_TILTROTOR:
	case MAV_TYPE_VTOL_FIXEDROTOR:
	case MAV_TYPE_VTOL_TAILSITTER:
	case MAV_TYPE_VTOL_TILTWING:
	case MAV_TYPE_VTOL_RESERVED5:
		switch (vehicle_type) {
		case vehicle_status_s::VEHICLE_TYPE_FIXED_WING:
			thrust_body_array[0] = thrust;

			break;

		case vehicle_status_s::VEHICLE_TYPE_ROTARY_WING:
			thrust_body_array[2] = -thrust;

			break;

		default:
			// This should never happen
			break;
		}

		break;
	}
}

void
MavlinkReceiver::handle_message_set_attitude_target(mavlink_message_t *msg)
{
	mavlink_set_attitude_target_t attitude_target;
	mavlink_msg_set_attitude_target_decode(msg, &attitude_target);

	/* Only accept messages which are intended for this system */
	if (_mavlink->get_forward_externalsp() &&
	    (mavlink_system.sysid == attitude_target.target_system || attitude_target.target_system == 0) &&
	    (mavlink_system.compid == attitude_target.target_component || attitude_target.target_component == 0)) {

		const uint8_t type_mask = attitude_target.type_mask;

		const bool attitude = !(type_mask & ATTITUDE_TARGET_TYPEMASK_ATTITUDE_IGNORE);
		const bool body_rates = !(type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)
					&& !(type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE);
		const bool thrust_body = (type_mask & ATTITUDE_TARGET_TYPEMASK_THRUST_BODY_SET);

		vehicle_status_s vehicle_status{};
		_vehicle_status_sub.copy(&vehicle_status);

		if (attitude || body_rates) {
			offboard_control_mode_s ocm{};
			ocm.attitude = attitude;
			ocm.body_rate = body_rates;
			ocm.timestamp = hrt_absolute_time();
			_offboard_control_mode_pub.publish(ocm);
		}

		if (attitude) {
			vehicle_attitude_setpoint_s attitude_setpoint{};

			const matrix::Quatf q{attitude_target.q};
			q.copyTo(attitude_setpoint.q_d);

			matrix::Eulerf euler{q};
			attitude_setpoint.roll_body = euler.phi();
			attitude_setpoint.pitch_body = euler.theta();
			attitude_setpoint.yaw_body = euler.psi();

			// TODO: review use case
			attitude_setpoint.yaw_sp_move_rate = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE) ?
							     (float)NAN : attitude_target.body_yaw_rate;

			if (!thrust_body && !(attitude_target.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)) {
				fill_thrust(attitude_setpoint.thrust_body, vehicle_status.vehicle_type, attitude_target.thrust);

			} else if (thrust_body) {
				attitude_setpoint.thrust_body[0] = attitude_target.thrust_body[0];
				attitude_setpoint.thrust_body[1] = attitude_target.thrust_body[1];
				attitude_setpoint.thrust_body[2] = attitude_target.thrust_body[2];
			}

			// Publish attitude setpoint only once in OFFBOARD
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				attitude_setpoint.timestamp = hrt_absolute_time();

				if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_ROTARY_WING)) {
					_mc_virtual_att_sp_pub.publish(attitude_setpoint);

				} else if (vehicle_status.is_vtol && (vehicle_status.vehicle_type == vehicle_status_s::VEHICLE_TYPE_FIXED_WING)) {
					_fw_virtual_att_sp_pub.publish(attitude_setpoint);

				} else {
					_att_sp_pub.publish(attitude_setpoint);
				}
			}

		}

		if (body_rates) {
			vehicle_rates_setpoint_s setpoint{};
			setpoint.roll  = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_ROLL_RATE_IGNORE)  ? (float)NAN :
					 attitude_target.body_roll_rate;
			setpoint.pitch = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_PITCH_RATE_IGNORE) ? (float)NAN :
					 attitude_target.body_pitch_rate;
			setpoint.yaw   = (type_mask & ATTITUDE_TARGET_TYPEMASK_BODY_YAW_RATE_IGNORE)   ? (float)NAN :
					 attitude_target.body_yaw_rate;

			if (!(attitude_target.type_mask & ATTITUDE_TARGET_TYPEMASK_THROTTLE_IGNORE)) {
				fill_thrust(setpoint.thrust_body, vehicle_status.vehicle_type, attitude_target.thrust);
			}

			// Publish rate setpoint only once in OFFBOARD
			if (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_OFFBOARD) {
				setpoint.timestamp = hrt_absolute_time();
				_rates_sp_pub.publish(setpoint);
			}
		}
	}
}

void
MavlinkReceiver::handle_message_radio_status(mavlink_message_t *msg)
{
	/* telemetry status supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {
		mavlink_radio_status_t rstatus;
		mavlink_msg_radio_status_decode(msg, &rstatus);

		radio_status_s status{};

		status.timestamp = hrt_absolute_time();
		status.rssi = rstatus.rssi;
		status.remote_rssi = rstatus.remrssi;
		status.txbuf = rstatus.txbuf;
		status.noise = rstatus.noise;
		status.remote_noise = rstatus.remnoise;
		status.rxerrors = rstatus.rxerrors;
		status.fix = rstatus.fixed;

		_mavlink->update_radio_status(status);

		_radio_status_pub.publish(status);
	}
}

void
MavlinkReceiver::handle_message_ping(mavlink_message_t *msg)
{
	mavlink_ping_t ping;
	mavlink_msg_ping_decode(msg, &ping);

	if ((ping.target_system == 0) &&
	    (ping.target_component == 0)) {	   // This is a ping request. Return it to the system which requested the ping.

		ping.target_system = msg->sysid;
		ping.target_component = msg->compid;
		mavlink_msg_ping_send_struct(_mavlink->get_channel(), &ping);

	} else if ((ping.target_system == mavlink_system.sysid) &&
		   (ping.target_component ==
		    mavlink_system.compid)) { // This is a returned ping message from this system. Calculate latency from it.

		const hrt_abstime now = hrt_absolute_time();

		// Calculate round trip time
		float rtt_ms = (now - ping.time_usec) / 1000.0f;

		// Update ping statistics
		struct Mavlink::ping_statistics_s &pstats = _mavlink->get_ping_statistics();

		pstats.last_ping_time = now;

		if (pstats.last_ping_seq == 0 && ping.seq > 0) {
			// This is the first reply we are receiving from an offboard system.
			// We may have been broadcasting pings for some time before it came online,
			// and these do not count as dropped packets.

			// Reset last_ping_seq counter for correct packet drop detection
			pstats.last_ping_seq = ping.seq - 1;
		}

		// We can only count dropped packets after the first message
		if (ping.seq > pstats.last_ping_seq) {
			pstats.dropped_packets += ping.seq - pstats.last_ping_seq - 1;
		}

		pstats.last_ping_seq = ping.seq;
		pstats.last_rtt = rtt_ms;
		pstats.mean_rtt = (rtt_ms + pstats.mean_rtt) / 2.0f;
		pstats.max_rtt = fmaxf(rtt_ms, pstats.max_rtt);
		pstats.min_rtt = pstats.min_rtt > 0.0f ? fminf(rtt_ms, pstats.min_rtt) : rtt_ms;

		/* Ping status is supported only on first ORB_MULTI_MAX_INSTANCES mavlink channels */
		if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

			ping_s uorb_ping_msg{};

			uorb_ping_msg.timestamp = now;
			uorb_ping_msg.ping_time = ping.time_usec;
			uorb_ping_msg.ping_sequence = ping.seq;
			uorb_ping_msg.dropped_packets = pstats.dropped_packets;
			uorb_ping_msg.rtt_ms = rtt_ms;
			uorb_ping_msg.system_id = msg->sysid;
			uorb_ping_msg.component_id = msg->compid;

			_ping_pub.publish(uorb_ping_msg);
		}
	}
}

void
MavlinkReceiver::handle_message_battery_status(mavlink_message_t *msg)
{
	if ((msg->sysid != mavlink_system.sysid) || (msg->compid == mavlink_system.compid)) {
		// ignore battery status coming from other systems or from the autopilot itself
		return;
	}

	// external battery measurements
	mavlink_battery_status_t battery_mavlink;
	mavlink_msg_battery_status_decode(msg, &battery_mavlink);

	battery_status_s battery_status{};
	battery_status.timestamp = hrt_absolute_time();

	float voltage_sum = 0.0f;
	uint8_t cell_count = 0;

	while ((cell_count < 10) && (battery_mavlink.voltages[cell_count] < UINT16_MAX)) {
		battery_status.voltage_cell_v[cell_count] = (float)(battery_mavlink.voltages[cell_count]) / 1000.0f;
		voltage_sum += battery_status.voltage_cell_v[cell_count];
		cell_count++;
	}

	battery_status.voltage_v = voltage_sum;
	battery_status.voltage_filtered_v  = voltage_sum;
	battery_status.current_a = battery_status.current_filtered_a = (float)(battery_mavlink.current_battery) / 100.0f;
	battery_status.current_filtered_a = battery_status.current_a;
	battery_status.remaining = (float)battery_mavlink.battery_remaining / 100.0f;
	battery_status.discharged_mah = (float)battery_mavlink.current_consumed;
	battery_status.cell_count = cell_count;
	battery_status.temperature = (float)battery_mavlink.temperature;
	battery_status.connected = true;

	// Set the battery warning based on remaining charge.
	//  Note: Smallest values must come first in evaluation.
	if (battery_status.remaining < _param_bat_emergen_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_EMERGENCY;

	} else if (battery_status.remaining < _param_bat_crit_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_CRITICAL;

	} else if (battery_status.remaining < _param_bat_low_thr.get()) {
		battery_status.warning = battery_status_s::BATTERY_WARNING_LOW;
	}

	_battery_pub.publish(battery_status);
}

void
MavlinkReceiver::handle_message_serial_control(mavlink_message_t *msg)
{
	mavlink_serial_control_t serial_control_mavlink;
	mavlink_msg_serial_control_decode(msg, &serial_control_mavlink);

	// Check if the message is targeted at us.
	if ((serial_control_mavlink.target_system != 0 &&
	     mavlink_system.sysid != serial_control_mavlink.target_system) ||
	    (serial_control_mavlink.target_component != 0 &&
	     mavlink_system.compid != serial_control_mavlink.target_component)) {
		return;
	}

	// we only support shell commands
	if (serial_control_mavlink.device != SERIAL_CONTROL_DEV_SHELL
	    || (serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_REPLY)) {
		return;
	}

	MavlinkShell *shell = _mavlink->get_shell();

	if (shell) {
		// we ignore the timeout, EXCLUSIVE & BLOCKING flags of the SERIAL_CONTROL message
		if (serial_control_mavlink.count > 0) {
			shell->setTargetID(msg->sysid, msg->compid);
			shell->write(serial_control_mavlink.data, serial_control_mavlink.count);
		}

		// if no response requested, assume the shell is no longer used
		if ((serial_control_mavlink.flags & SERIAL_CONTROL_FLAG_RESPOND) == 0) {
			_mavlink->close_shell();
		}
	}
}

void
MavlinkReceiver::handle_message_logging_ack(mavlink_message_t *msg)
{
	mavlink_logging_ack_t logging_ack;
	mavlink_msg_logging_ack_decode(msg, &logging_ack);

	MavlinkULog *ulog_streaming = _mavlink->get_ulog_streaming();

	if (ulog_streaming) {
		ulog_streaming->handle_ack(logging_ack);
	}
}

void
MavlinkReceiver::handle_message_play_tune(mavlink_message_t *msg)
{
	mavlink_play_tune_t play_tune;
	mavlink_msg_play_tune_decode(msg, &play_tune);

	if ((mavlink_system.sysid == play_tune.target_system || play_tune.target_system == 0) &&
	    (mavlink_system.compid == play_tune.target_component || play_tune.target_component == 0)) {

		// Let's make sure the input is 0 terminated, so we don't ever overrun it.
		play_tune.tune2[sizeof(play_tune.tune2) - 1] = '\0';

		schedule_tune(play_tune.tune);
	}
}

void
MavlinkReceiver::handle_message_play_tune_v2(mavlink_message_t *msg)
{
	mavlink_play_tune_v2_t play_tune_v2;
	mavlink_msg_play_tune_v2_decode(msg, &play_tune_v2);

	if ((mavlink_system.sysid == play_tune_v2.target_system || play_tune_v2.target_system == 0) &&
	    (mavlink_system.compid == play_tune_v2.target_component || play_tune_v2.target_component == 0)) {

		if (play_tune_v2.format != TUNE_FORMAT_QBASIC1_1) {
			PX4_ERR("Tune format %" PRIu32 " not supported", play_tune_v2.format);
			return;
		}

		// Let's make sure the input is 0 terminated, so we don't ever overrun it.
		play_tune_v2.tune[sizeof(play_tune_v2.tune) - 1] = '\0';

		schedule_tune(play_tune_v2.tune);
	}
}

void MavlinkReceiver::schedule_tune(const char *tune)
{
	// We only allocate the TunePublisher object if we ever use it but we
	// don't remove it to avoid fragmentation over time.
	if (_tune_publisher == nullptr) {
		_tune_publisher = new TunePublisher();

		if (_tune_publisher == nullptr) {
			PX4_ERR("Could not allocate tune publisher");
			return;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	_tune_publisher->set_tune_string(tune, now);
	// Send first one straightaway.
	_tune_publisher->publish_next_tune(now);
}


void
MavlinkReceiver::handle_message_obstacle_distance(mavlink_message_t *msg)
{
	mavlink_obstacle_distance_t mavlink_obstacle_distance;
	mavlink_msg_obstacle_distance_decode(msg, &mavlink_obstacle_distance);

	obstacle_distance_s obstacle_distance{};

	obstacle_distance.timestamp = hrt_absolute_time();
	obstacle_distance.sensor_type = mavlink_obstacle_distance.sensor_type;
	memcpy(obstacle_distance.distances, mavlink_obstacle_distance.distances, sizeof(obstacle_distance.distances));

	if (mavlink_obstacle_distance.increment_f > 0.f) {
		obstacle_distance.increment = mavlink_obstacle_distance.increment_f;

	} else {
		obstacle_distance.increment = (float)mavlink_obstacle_distance.increment;
	}

	obstacle_distance.min_distance = mavlink_obstacle_distance.min_distance;
	obstacle_distance.max_distance = mavlink_obstacle_distance.max_distance;
	obstacle_distance.angle_offset = mavlink_obstacle_distance.angle_offset;
	obstacle_distance.frame = mavlink_obstacle_distance.frame;

	_obstacle_distance_pub.publish(obstacle_distance);
}

void
MavlinkReceiver::handle_message_tunnel(mavlink_message_t *msg)
{
	mavlink_tunnel_t mavlink_tunnel;
	mavlink_msg_tunnel_decode(msg, &mavlink_tunnel);

	mavlink_tunnel_s tunnel{};

	tunnel.timestamp = hrt_absolute_time();
	tunnel.payload_type = mavlink_tunnel.payload_type;
	tunnel.target_system = mavlink_tunnel.target_system;
	tunnel.target_component = mavlink_tunnel.target_component;
	tunnel.payload_length = mavlink_tunnel.payload_length;
	memcpy(tunnel.payload, mavlink_tunnel.payload, sizeof(tunnel.payload));
	static_assert(sizeof(tunnel.payload) == sizeof(mavlink_tunnel.payload), "mavlink_tunnel.payload size mismatch");

	_mavlink_tunnel_pub.publish(tunnel);

}

void
MavlinkReceiver::handle_message_trajectory_representation_bezier(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_bezier_t trajectory;
	mavlink_msg_trajectory_representation_bezier_decode(msg, &trajectory);

	vehicle_trajectory_bezier_s trajectory_bezier{};

	trajectory_bezier.timestamp =  _mavlink_timesync.sync_stamp(trajectory.time_usec);

	for (int i = 0; i < vehicle_trajectory_bezier_s::NUMBER_POINTS; ++i) {
		trajectory_bezier.control_points[i].position[0] = trajectory.pos_x[i];
		trajectory_bezier.control_points[i].position[1] = trajectory.pos_y[i];
		trajectory_bezier.control_points[i].position[2] = trajectory.pos_z[i];

		trajectory_bezier.control_points[i].delta = trajectory.delta[i];
		trajectory_bezier.control_points[i].yaw = trajectory.pos_yaw[i];
	}

	trajectory_bezier.bezier_order = math::min(trajectory.valid_points, vehicle_trajectory_bezier_s::NUMBER_POINTS);
	_trajectory_bezier_pub.publish(trajectory_bezier);
}

void
MavlinkReceiver::handle_message_trajectory_representation_waypoints(mavlink_message_t *msg)
{
	mavlink_trajectory_representation_waypoints_t trajectory;
	mavlink_msg_trajectory_representation_waypoints_decode(msg, &trajectory);

	vehicle_trajectory_waypoint_s trajectory_waypoint{};

	const int number_valid_points = math::min(trajectory.valid_points, vehicle_trajectory_waypoint_s::NUMBER_POINTS);

	for (int i = 0; i < number_valid_points; ++i) {
		trajectory_waypoint.waypoints[i].position[0] = trajectory.pos_x[i];
		trajectory_waypoint.waypoints[i].position[1] = trajectory.pos_y[i];
		trajectory_waypoint.waypoints[i].position[2] = trajectory.pos_z[i];

		trajectory_waypoint.waypoints[i].velocity[0] = trajectory.vel_x[i];
		trajectory_waypoint.waypoints[i].velocity[1] = trajectory.vel_y[i];
		trajectory_waypoint.waypoints[i].velocity[2] = trajectory.vel_z[i];

		trajectory_waypoint.waypoints[i].acceleration[0] = trajectory.acc_x[i];
		trajectory_waypoint.waypoints[i].acceleration[1] = trajectory.acc_y[i];
		trajectory_waypoint.waypoints[i].acceleration[2] = trajectory.acc_z[i];

		trajectory_waypoint.waypoints[i].yaw = trajectory.pos_yaw[i];
		trajectory_waypoint.waypoints[i].yaw_speed = trajectory.vel_yaw[i];

		trajectory_waypoint.waypoints[i].point_valid = true;

		trajectory_waypoint.waypoints[i].type = UINT8_MAX;
	}

	trajectory_waypoint.timestamp = hrt_absolute_time();
	_trajectory_waypoint_pub.publish(trajectory_waypoint);
}

void
MavlinkReceiver::handle_message_rc_channels_override(mavlink_message_t *msg)
{
	mavlink_rc_channels_override_t man;
	mavlink_msg_rc_channels_override_decode(msg, &man);

	// Check target
	if (man.target_system != 0 && man.target_system != _mavlink->get_system_id()) {
		return;
	}

	// fill uORB message
	input_rc_s rc{};
	// metadata
	rc.timestamp = rc.timestamp_last_signal = hrt_absolute_time();
	rc.rssi = input_rc_s::RSSI_MAX;
	rc.rc_failsafe = false;
	rc.rc_lost = false;
	rc.rc_lost_frame_count = 0;
	rc.rc_total_frame_count = 1;
	rc.input_source = input_rc_s::RC_INPUT_SOURCE_MAVLINK;

	// channels
	rc.values[0] = man.chan1_raw;
	rc.values[1] = man.chan2_raw;
	rc.values[2] = man.chan3_raw;
	rc.values[3] = man.chan4_raw;
	rc.values[4] = man.chan5_raw;
	rc.values[5] = man.chan6_raw;
	rc.values[6] = man.chan7_raw;
	rc.values[7] = man.chan8_raw;
	rc.values[8] = man.chan9_raw;
	rc.values[9] = man.chan10_raw;
	rc.values[10] = man.chan11_raw;
	rc.values[11] = man.chan12_raw;
	rc.values[12] = man.chan13_raw;
	rc.values[13] = man.chan14_raw;
	rc.values[14] = man.chan15_raw;
	rc.values[15] = man.chan16_raw;
	rc.values[16] = man.chan17_raw;
	rc.values[17] = man.chan18_raw;

#if defined(ATL_MANTIS_RC_INPUT_HACKS)

	// Sanity checking if the RC controller is really sending.
	if (rc.values[5] == 2048 && rc.values[6] == 0 && (rc.values[8] & 0xff00) == 0x0C00) {
		rc.values[0] = 1000 + (uint16_t)((float)rc.values[0] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[1] = 1000 + (uint16_t)((float)rc.values[1] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[2] = 1000 + (uint16_t)((float)rc.values[2] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[3] = 1000 + (uint16_t)((float)rc.values[3] / 4095.f * 1000.f); // 0..4095 -> 1000..2000
		rc.values[4] = 1000 + (uint16_t)((float)rc.values[4] / 4095.f * 1000.f); // 0..4095 -> 1000..2000 -> Aux 2
		rc.values[5] = 1000 + rc.values[7] * 500; // Switch toggles from 0 to 2, we map it from 1000 to 2000
		rc.values[6] = 1000 + (((rc.values[8] & 0x02) != 0) ? 1000 : 0); // Return button.
		rc.values[7] = 1000 + (((rc.values[8] & 0x01) != 0) ? 1000 : 0); // Video record button. -> Aux4
		rc.values[9] = 1000 + (((rc.values[8] & 0x10) != 0) ? 1000 : 0); // Photo record button. -> Aux3
		rc.values[8] = rc.values[8] & 0x00ff; // Remove magic identifier.
		// leave rc.values[10] as is 0..4095
		//
		// Note we are currently ignoring the stick buttons (sticks pressed down).
		// They are on rc.values[8] & 0x20 and rc.values[8] & 0x40.
	}

#endif

	// check how many channels are valid
	for (int i = 17; i >= 0; i--) {
		const bool ignore_max = rc.values[i] == UINT16_MAX; // ignore any channel with value UINT16_MAX
		const bool ignore_zero = (i > 7) && (rc.values[i] == 0); // ignore channel 8-18 if value is 0

		if (ignore_max || ignore_zero) {
			// set all ignored values to zero
			rc.values[i] = 0;

		} else {
			// first channel to not ignore -> set count considering zero-based index
			rc.channel_count = i + 1;
			break;
		}
	}

	rc.link_quality = -1;
	rc.rssi_dbm = NAN;

	// publish uORB message
	_rc_pub.publish(rc);
}

void
MavlinkReceiver::handle_message_manual_control(mavlink_message_t *msg)
{
	mavlink_manual_control_t mavlink_manual_control;
	mavlink_msg_manual_control_decode(msg, &mavlink_manual_control);

	// Check target
	if (mavlink_manual_control.target != 0 && mavlink_manual_control.target != _mavlink->get_system_id()) {
		return;
	}

	manual_control_setpoint_s manual_control_setpoint{};
	manual_control_setpoint.pitch = mavlink_manual_control.x / 1000.f;
	manual_control_setpoint.roll = mavlink_manual_control.y / 1000.f;
	// For backwards compatibility at the moment interpret throttle in range [0,1000]
	manual_control_setpoint.throttle = ((mavlink_manual_control.z / 1000.f) * 2.f) - 1.f;
	manual_control_setpoint.yaw = mavlink_manual_control.r / 1000.f;
	manual_control_setpoint.data_source = manual_control_setpoint_s::SOURCE_MAVLINK_0 + _mavlink->get_instance_id();
	manual_control_setpoint.timestamp = manual_control_setpoint.timestamp_sample = hrt_absolute_time();
	_manual_control_input_pub.publish(manual_control_setpoint);
}

void
MavlinkReceiver::handle_message_heartbeat(mavlink_message_t *msg)
{
	/* telemetry status supported only on first TELEMETRY_STATUS_ORB_ID_NUM mavlink channels */
	if (_mavlink->get_channel() < (mavlink_channel_t)ORB_MULTI_MAX_INSTANCES) {

		const hrt_abstime now = hrt_absolute_time();

		mavlink_heartbeat_t hb;
		mavlink_msg_heartbeat_decode(msg, &hb);

		const bool same_system = (msg->sysid == mavlink_system.sysid);

		if (same_system || hb.type == MAV_TYPE_GCS) {

			camera_status_s camera_status{};

			switch (hb.type) {
			case MAV_TYPE_ANTENNA_TRACKER:
				_heartbeat_type_antenna_tracker = now;
				break;

			case MAV_TYPE_GCS:
				_heartbeat_type_gcs = now;
				break;

			case MAV_TYPE_ONBOARD_CONTROLLER:
				_heartbeat_type_onboard_controller = now;
				break;

			case MAV_TYPE_GIMBAL:
				_heartbeat_type_gimbal = now;
				break;

			case MAV_TYPE_ADSB:
				_heartbeat_type_adsb = now;
				break;

			case MAV_TYPE_CAMERA:
				_heartbeat_type_camera = now;
				camera_status.timestamp = now;
				camera_status.active_comp_id = msg->compid;
				camera_status.active_sys_id = msg->sysid;
				_camera_status_pub.publish(camera_status);
				break;

			case MAV_TYPE_PARACHUTE:
				_heartbeat_type_parachute = now;
				_mavlink->telemetry_status().parachute_system_healthy =
					(hb.system_status == MAV_STATE_STANDBY) || (hb.system_status == MAV_STATE_ACTIVE);
				break;

			case MAV_TYPE_ODID:
				_heartbeat_type_open_drone_id = now;
				_mavlink->telemetry_status().open_drone_id_system_healthy =
					(hb.system_status == MAV_STATE_STANDBY) || (hb.system_status == MAV_STATE_ACTIVE);
				break;

			default:
				PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %" PRIu8 " from SYSID: %" PRIu8 ", COMPID: %" PRIu8, hb.type, msg->sysid,
					  msg->compid);
			}


			switch (msg->compid) {
			case MAV_COMP_ID_TELEMETRY_RADIO:
				_heartbeat_component_telemetry_radio = now;
				break;

			case MAV_COMP_ID_LOG:
				_heartbeat_component_log = now;
				break;

			case MAV_COMP_ID_OSD:
				_heartbeat_component_osd = now;
				break;

			case MAV_COMP_ID_OBSTACLE_AVOIDANCE:
				_heartbeat_component_obstacle_avoidance = now;
				_mavlink->telemetry_status().avoidance_system_healthy = (hb.system_status == MAV_STATE_ACTIVE);
				break;

			case MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY:
				_heartbeat_component_visual_inertial_odometry = now;
				break;

			case MAV_COMP_ID_PAIRING_MANAGER:
				_heartbeat_component_pairing_manager = now;
				break;

			case MAV_COMP_ID_UDP_BRIDGE:
				_heartbeat_component_udp_bridge = now;
				break;

			case MAV_COMP_ID_UART_BRIDGE:
				_heartbeat_component_uart_bridge = now;
				break;

			default:
				PX4_DEBUG("unhandled HEARTBEAT MAV_TYPE: %" PRIu8 " from SYSID: %" PRIu8 ", COMPID: %" PRIu8, hb.type, msg->sysid,
					  msg->compid);
			}

			CheckHeartbeats(now, true);
		}
	}
}

int
MavlinkReceiver::set_message_interval(int msgId, float interval, int data_rate)
{
	if (msgId == MAVLINK_MSG_ID_HEARTBEAT) {
		return PX4_ERROR;
	}

	if (data_rate > 0) {
		_mavlink->set_data_rate(data_rate);
	}

	// configure_stream wants a rate (msgs/second), so convert here.
	float rate = 0.f;

	if (interval < -0.00001f) {
		rate = 0.f; // stop the stream

	} else if (interval > 0.00001f) {
		rate = 1000000.0f / interval;

	} else {
		rate = -2.f; // set default rate
	}

	bool found_id = false;

	if (msgId != 0) {
		const char *stream_name = get_stream_name(msgId);

		if (stream_name != nullptr) {
			_mavlink->configure_stream_threadsafe(stream_name, rate);
			found_id = true;
		}
	}

	return (found_id ? PX4_OK : PX4_ERROR);
}

void
MavlinkReceiver::get_message_interval(int msgId)
{
	unsigned interval = 0;

	for (const auto &stream : _mavlink->get_streams()) {
		if (stream->get_id() == msgId) {
			interval = stream->get_interval();
			break;
		}
	}

	// send back this value...
	mavlink_msg_message_interval_send(_mavlink->get_channel(), msgId, interval);
}

void
MavlinkReceiver::handle_message_hil_sensor(mavlink_message_t *msg)
{
	mavlink_hil_sensor_t hil_sensor;
	mavlink_msg_hil_sensor_decode(msg, &hil_sensor);

	const uint64_t timestamp = hrt_absolute_time();

	// temperature only updated with baro
	float temperature = NAN;

	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		temperature = hil_sensor.temperature;
	}

	// gyro
	if ((hil_sensor.fields_updated & SensorSource::GYRO) == SensorSource::GYRO) {
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);
		}

		if (_px4_gyro != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_gyro->set_temperature(temperature);
			}

			_px4_gyro->update(timestamp, hil_sensor.xgyro, hil_sensor.ygyro, hil_sensor.zgyro);
		}
	}

	// accelerometer
	if ((hil_sensor.fields_updated & SensorSource::ACCEL) == SensorSource::ACCEL) {
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);
		}

		if (_px4_accel != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_accel->set_temperature(temperature);
			}

			_px4_accel->update(timestamp, hil_sensor.xacc, hil_sensor.yacc, hil_sensor.zacc);
		}
	}

	// magnetometer
	if ((hil_sensor.fields_updated & SensorSource::MAG) == SensorSource::MAG) {
		if (_px4_mag == nullptr) {
			// 197388: DRV_MAG_DEVTYPE_MAGSIM, BUS: 3, ADDR: 1, TYPE: SIMULATION
			_px4_mag = new PX4Magnetometer(197388);
		}

		if (_px4_mag != nullptr) {
			if (PX4_ISFINITE(temperature)) {
				_px4_mag->set_temperature(temperature);
			}

			_px4_mag->update(timestamp, hil_sensor.xmag, hil_sensor.ymag, hil_sensor.zmag);
		}
	}

	// baro
	if ((hil_sensor.fields_updated & SensorSource::BARO) == SensorSource::BARO) {
		// publish
		sensor_baro_s sensor_baro{};
		sensor_baro.timestamp_sample = timestamp;
		sensor_baro.device_id = 6620172; // 6620172: DRV_BARO_DEVTYPE_BAROSIM, BUS: 1, ADDR: 4, TYPE: SIMULATION
		sensor_baro.pressure = hil_sensor.abs_pressure * 100.0f; // hPa to Pa
		sensor_baro.temperature = hil_sensor.temperature;
		sensor_baro.error_count = 0;
		sensor_baro.timestamp = hrt_absolute_time();
		_sensor_baro_pub.publish(sensor_baro);
	}

	// differential pressure
	if ((hil_sensor.fields_updated & SensorSource::DIFF_PRESS) == SensorSource::DIFF_PRESS) {
		differential_pressure_s report{};
		report.timestamp_sample = timestamp;
		report.device_id = 1377548; // 1377548: DRV_DIFF_PRESS_DEVTYPE_SIM, BUS: 1, ADDR: 5, TYPE: SIMULATION
		report.temperature = hil_sensor.temperature;
		report.differential_pressure_pa = hil_sensor.diff_pressure * 100.0f; // hPa to Pa
		report.timestamp = hrt_absolute_time();
		_differential_pressure_pub.publish(report);
	}

	// battery status
	{
		battery_status_s hil_battery_status{};

		hil_battery_status.timestamp = timestamp;
		hil_battery_status.voltage_v = 16.0f;
		hil_battery_status.voltage_filtered_v = 16.0f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.connected = true;
		hil_battery_status.remaining = 0.70;
		hil_battery_status.time_remaining_s = NAN;

		_battery_pub.publish(hil_battery_status);
	}
}

void
MavlinkReceiver::handle_message_hil_gps(mavlink_message_t *msg)
{
	mavlink_hil_gps_t hil_gps;
	mavlink_msg_hil_gps_decode(msg, &hil_gps);

	sensor_gps_s gps{};

	device::Device::DeviceId device_id;
	device_id.devid_s.bus_type = device::Device::DeviceBusType::DeviceBusType_MAVLINK;
	device_id.devid_s.bus = _mavlink->get_instance_id();
	device_id.devid_s.address = msg->sysid;
	device_id.devid_s.devtype = DRV_GPS_DEVTYPE_SIM;

	gps.device_id = device_id.devid;

	gps.lat = hil_gps.lat;
	gps.lon = hil_gps.lon;
	gps.alt = hil_gps.alt;
	gps.alt_ellipsoid = hil_gps.alt;

	gps.s_variance_m_s = 0.25f;
	gps.c_variance_rad = 0.5f;
	gps.fix_type = hil_gps.fix_type;

	gps.eph = (float)hil_gps.eph * 1e-2f; // cm -> m
	gps.epv = (float)hil_gps.epv * 1e-2f; // cm -> m

	gps.hdop = 0; // TODO
	gps.vdop = 0; // TODO

	gps.noise_per_ms = 0;
	gps.automatic_gain_control = 0;
	gps.jamming_indicator = 0;
	gps.jamming_state = 0;
	gps.spoofing_state = 0;

	gps.vel_m_s = (float)(hil_gps.vel) / 100.0f; // cm/s -> m/s
	gps.vel_n_m_s = (float)(hil_gps.vn) / 100.0f; // cm/s -> m/s
	gps.vel_e_m_s = (float)(hil_gps.ve) / 100.0f; // cm/s -> m/s
	gps.vel_d_m_s = (float)(hil_gps.vd) / 100.0f; // cm/s -> m/s
	gps.cog_rad = ((hil_gps.cog == 65535) ? (float)NAN : matrix::wrap_2pi(math::radians(
				hil_gps.cog * 1e-2f))); // cdeg -> rad
	gps.vel_ned_valid = true;

	gps.timestamp_time_relative = 0;
	gps.time_utc_usec = hil_gps.time_usec;

	gps.satellites_used = hil_gps.satellites_visible;

	gps.heading = NAN;
	gps.heading_offset = NAN;

	gps.timestamp = hrt_absolute_time();

	_sensor_gps_pub.publish(gps);
}

void
MavlinkReceiver::handle_message_follow_target(mavlink_message_t *msg)
{
	mavlink_follow_target_t follow_target_msg;
	mavlink_msg_follow_target_decode(msg, &follow_target_msg);

	follow_target_s follow_target_topic{};

	follow_target_topic.timestamp = hrt_absolute_time();
	follow_target_topic.lat = follow_target_msg.lat * 1e-7;
	follow_target_topic.lon = follow_target_msg.lon * 1e-7;
	follow_target_topic.alt = follow_target_msg.alt;
	follow_target_topic.vx = follow_target_msg.vel[0];
	follow_target_topic.vy = follow_target_msg.vel[1];
	follow_target_topic.vz = follow_target_msg.vel[2];

	_follow_target_pub.publish(follow_target_topic);
}

void
MavlinkReceiver::handle_message_landing_target(mavlink_message_t *msg)
{
	mavlink_landing_target_t landing_target;
	mavlink_msg_landing_target_decode(msg, &landing_target);

	if (landing_target.position_valid && landing_target.frame == MAV_FRAME_LOCAL_NED) {
		landing_target_pose_s landing_target_pose{};

		landing_target_pose.timestamp = _mavlink_timesync.sync_stamp(landing_target.time_usec);
		landing_target_pose.abs_pos_valid = true;
		landing_target_pose.x_abs = landing_target.x;
		landing_target_pose.y_abs = landing_target.y;
		landing_target_pose.z_abs = landing_target.z;

		_landing_target_pose_pub.publish(landing_target_pose);

	} else if (landing_target.position_valid) {
		// We only support MAV_FRAME_LOCAL_NED. In this case, the frame was unsupported.
		mavlink_log_critical(&_mavlink_log_pub, "landing target: coordinate frame %" PRIu8 " unsupported\t",
				     landing_target.frame);
		events::send<uint8_t>(events::ID("mavlink_rcv_lnd_target_unsup_coord"), events::Log::Error,
				      "landing target: unsupported coordinate frame {1}", landing_target.frame);

	} else {
		irlock_report_s irlock_report{};

		irlock_report.timestamp = hrt_absolute_time();
		irlock_report.signature = landing_target.target_num;
		irlock_report.pos_x = landing_target.angle_x;
		irlock_report.pos_y = landing_target.angle_y;
		irlock_report.size_x = landing_target.size_x;
		irlock_report.size_y = landing_target.size_y;

		_irlock_report_pub.publish(irlock_report);
	}
}

void
MavlinkReceiver::handle_message_cellular_status(mavlink_message_t *msg)
{
	mavlink_cellular_status_t status;
	mavlink_msg_cellular_status_decode(msg, &status);

	cellular_status_s cellular_status{};

	cellular_status.timestamp = hrt_absolute_time();
	cellular_status.status = status.status;
	cellular_status.failure_reason = status.failure_reason;
	cellular_status.type = status.type;
	cellular_status.quality = status.quality;
	cellular_status.mcc = status.mcc;
	cellular_status.mnc = status.mnc;
	cellular_status.lac = status.lac;

	_cellular_status_pub.publish(cellular_status);
}

void
MavlinkReceiver::handle_message_adsb_vehicle(mavlink_message_t *msg)
{
	mavlink_adsb_vehicle_t adsb;
	mavlink_msg_adsb_vehicle_decode(msg, &adsb);

	transponder_report_s t{};

	t.timestamp = hrt_absolute_time();

	t.icao_address = adsb.ICAO_address;
	t.lat = adsb.lat * 1e-7;
	t.lon = adsb.lon * 1e-7;
	t.altitude_type = adsb.altitude_type;
	t.altitude = adsb.altitude / 1000.0f;
	t.heading = adsb.heading / 100.0f / 180.0f * M_PI_F - M_PI_F;
	t.hor_velocity = adsb.hor_velocity / 100.0f;
	t.ver_velocity = adsb.ver_velocity / 100.0f;
	memcpy(&t.callsign[0], &adsb.callsign[0], sizeof(t.callsign));
	t.emitter_type = adsb.emitter_type;
	t.tslc = adsb.tslc;
	t.squawk = adsb.squawk;

	t.flags = transponder_report_s::PX4_ADSB_FLAGS_RETRANSLATE;  //Unset in receiver already broadcast its messages

	if (adsb.flags & ADSB_FLAGS_VALID_COORDS) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS; }

	if (adsb.flags & ADSB_FLAGS_VALID_ALTITUDE) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE; }

	if (adsb.flags & ADSB_FLAGS_VALID_HEADING) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING; }

	if (adsb.flags & ADSB_FLAGS_VALID_VELOCITY) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY; }

	if (adsb.flags & ADSB_FLAGS_VALID_CALLSIGN) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_CALLSIGN; }

	if (adsb.flags & ADSB_FLAGS_VALID_SQUAWK) { t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_SQUAWK; }

	//PX4_INFO("code: %d callsign: %s, vel: %8.4f, tslc: %d", (int)t.ICAO_address, t.callsign, (double)t.hor_velocity, (int)t.tslc);

	_transponder_report_pub.publish(t);
}

void
MavlinkReceiver::handle_message_utm_global_position(mavlink_message_t *msg)
{
	mavlink_utm_global_position_t utm_pos;
	mavlink_msg_utm_global_position_decode(msg, &utm_pos);

	bool is_self_published = false;


#ifndef BOARD_HAS_NO_UUID
	px4_guid_t px4_guid;
	board_get_px4_guid(px4_guid);
	is_self_published = sizeof(px4_guid) == sizeof(utm_pos.uas_id)
			    && memcmp(px4_guid, utm_pos.uas_id, sizeof(px4_guid_t)) == 0;
#else

	is_self_published = msg->sysid == _mavlink->get_system_id();
#endif /* BOARD_HAS_NO_UUID */


	//Ignore selfpublished UTM messages
	if (is_self_published) {
		return;
	}

	// Convert cm/s to m/s
	float vx = utm_pos.vx / 100.0f;
	float vy = utm_pos.vy / 100.0f;
	float vz = utm_pos.vz / 100.0f;

	transponder_report_s t{};
	t.timestamp = hrt_absolute_time();
	mav_array_memcpy(t.uas_id, utm_pos.uas_id, PX4_GUID_BYTE_LENGTH);
	t.icao_address = msg->sysid;
	t.lat = utm_pos.lat * 1e-7;
	t.lon = utm_pos.lon * 1e-7;
	t.altitude = utm_pos.alt / 1000.0f;
	t.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
	// UTM_GLOBAL_POSIION uses NED (north, east, down) coordinates for velocity, in cm / s.
	t.heading = atan2f(vy, vx);
	t.hor_velocity = sqrtf(vy * vy + vx * vx);
	t.ver_velocity = -vz;
	// TODO: Callsign
	// For now, set it to all 0s. This is a null-terminated string, so not explicitly giving it a null
	// terminator could cause problems.
	memset(&t.callsign[0], 0, sizeof(t.callsign));
	t.emitter_type = ADSB_EMITTER_TYPE_UAV;  // TODO: Is this correct?x2?

	// The Mavlink docs do not specify what to do if tslc (time since last communication) is out of range of
	// an 8-bit int, or if this is the first communication.
	// Here, I assume that if this is the first communication, tslc = 0.
	// If tslc > 255, then tslc = 255.
	unsigned long time_passed = (t.timestamp - _last_utm_global_pos_com) / 1000000;

	if (_last_utm_global_pos_com == 0) {
		time_passed = 0;

	} else if (time_passed > UINT8_MAX) {
		time_passed = UINT8_MAX;
	}

	t.tslc = (uint8_t) time_passed;

	t.flags = 0;

	if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_COORDS;
	}

	if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_ALTITUDE;
	}

	if (utm_pos.flags & UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE) {
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_HEADING;
		t.flags |= transponder_report_s::PX4_ADSB_FLAGS_VALID_VELOCITY;
	}

	// Note: t.flags has deliberately NOT set VALID_CALLSIGN or VALID_SQUAWK, because UTM_GLOBAL_POSITION does not
	// provide these.
	_transponder_report_pub.publish(t);

	_last_utm_global_pos_com = t.timestamp;
}

void
MavlinkReceiver::handle_message_collision(mavlink_message_t *msg)
{
	mavlink_collision_t collision;
	mavlink_msg_collision_decode(msg, &collision);

	collision_report_s collision_report{};

	collision_report.timestamp = hrt_absolute_time();
	collision_report.src = collision.src;
	collision_report.id = collision.id;
	collision_report.action = collision.action;
	collision_report.threat_level = collision.threat_level;
	collision_report.time_to_minimum_delta = collision.time_to_minimum_delta;
	collision_report.altitude_minimum_delta = collision.altitude_minimum_delta;
	collision_report.horizontal_minimum_delta = collision.horizontal_minimum_delta;

	_collision_report_pub.publish(collision_report);
}

void
MavlinkReceiver::handle_message_gps_rtcm_data(mavlink_message_t *msg)
{
	mavlink_gps_rtcm_data_t gps_rtcm_data_msg;
	mavlink_msg_gps_rtcm_data_decode(msg, &gps_rtcm_data_msg);

	gps_inject_data_s gps_inject_data_topic{};

	gps_inject_data_topic.timestamp = hrt_absolute_time();

	gps_inject_data_topic.len = math::min((int)sizeof(gps_rtcm_data_msg.data),
					      (int)sizeof(uint8_t) * gps_rtcm_data_msg.len);
	gps_inject_data_topic.flags = gps_rtcm_data_msg.flags;
	memcpy(gps_inject_data_topic.data, gps_rtcm_data_msg.data,
	       math::min((int)sizeof(gps_inject_data_topic.data), (int)sizeof(uint8_t) * gps_inject_data_topic.len));

	gps_inject_data_topic.timestamp = hrt_absolute_time();
	_gps_inject_data_pub.publish(gps_inject_data_topic);
}

void
MavlinkReceiver::handle_message_hil_state_quaternion(mavlink_message_t *msg)
{
	mavlink_hil_state_quaternion_t hil_state;
	mavlink_msg_hil_state_quaternion_decode(msg, &hil_state);

	const uint64_t timestamp_sample = hrt_absolute_time();

	/* airspeed */
	{
		airspeed_s airspeed{};
		airspeed.timestamp_sample = timestamp_sample;
		airspeed.indicated_airspeed_m_s = hil_state.ind_airspeed * 1e-2f;
		airspeed.true_airspeed_m_s = hil_state.true_airspeed * 1e-2f;
		airspeed.air_temperature_celsius = 15.f;
		airspeed.timestamp = hrt_absolute_time();
		_airspeed_pub.publish(airspeed);
	}

	/* attitude */
	{
		vehicle_attitude_s hil_attitude{};
		hil_attitude.timestamp_sample = timestamp_sample;
		matrix::Quatf q(hil_state.attitude_quaternion);
		q.copyTo(hil_attitude.q);
		hil_attitude.timestamp = hrt_absolute_time();
		_attitude_pub.publish(hil_attitude);
	}

	/* global position */
	{
		vehicle_global_position_s hil_global_pos{};

		hil_global_pos.timestamp_sample = timestamp_sample;
		hil_global_pos.lat = hil_state.lat / ((double)1e7);
		hil_global_pos.lon = hil_state.lon / ((double)1e7);
		hil_global_pos.alt = hil_state.alt / 1000.0f;
		hil_global_pos.eph = 2.f;
		hil_global_pos.epv = 4.f;
		hil_global_pos.timestamp = hrt_absolute_time();
		_global_pos_pub.publish(hil_global_pos);
	}

	/* local position */
	{
		const double lat = hil_state.lat * 1e-7;
		const double lon = hil_state.lon * 1e-7;

		if (!_global_local_proj_ref.isInitialized() || !PX4_ISFINITE(_global_local_alt0)) {
			_global_local_proj_ref.initReference(lat, lon);
			_global_local_alt0 = hil_state.alt / 1000.f;
		}

		float x = 0.f;
		float y = 0.f;
		_global_local_proj_ref.project(lat, lon, x, y);

		vehicle_local_position_s hil_local_pos{};
		hil_local_pos.timestamp_sample = timestamp_sample;
		hil_local_pos.ref_timestamp = _global_local_proj_ref.getProjectionReferenceTimestamp();
		hil_local_pos.ref_lat = _global_local_proj_ref.getProjectionReferenceLat();
		hil_local_pos.ref_lon = _global_local_proj_ref.getProjectionReferenceLon();
		hil_local_pos.ref_alt = _global_local_alt0;
		hil_local_pos.xy_valid = true;
		hil_local_pos.z_valid = true;
		hil_local_pos.v_xy_valid = true;
		hil_local_pos.v_z_valid = true;
		hil_local_pos.x = x;
		hil_local_pos.y = y;
		hil_local_pos.z = _global_local_alt0 - hil_state.alt / 1000.f;
		hil_local_pos.vx = hil_state.vx / 100.f;
		hil_local_pos.vy = hil_state.vy / 100.f;
		hil_local_pos.vz = hil_state.vz / 100.f;

		matrix::Eulerf euler{matrix::Quatf(hil_state.attitude_quaternion)};
		hil_local_pos.heading = euler.psi();
		hil_local_pos.xy_global = true;
		hil_local_pos.z_global = true;
		hil_local_pos.vxy_max = INFINITY;
		hil_local_pos.vz_max = INFINITY;
		hil_local_pos.hagl_min = INFINITY;
		hil_local_pos.hagl_max = INFINITY;
		hil_local_pos.timestamp = hrt_absolute_time();
		_local_pos_pub.publish(hil_local_pos);
	}

	/* accelerometer */
	{
		if (_px4_accel == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_accel = new PX4Accelerometer(1310988);

			if (_px4_accel == nullptr) {
				PX4_ERR("PX4Accelerometer alloc failed");
			}
		}

		if (_px4_accel != nullptr) {
			// accel in mG
			_px4_accel->set_scale(CONSTANTS_ONE_G / 1000.0f);
			_px4_accel->update(timestamp_sample, hil_state.xacc, hil_state.yacc, hil_state.zacc);
		}
	}

	/* gyroscope */
	{
		if (_px4_gyro == nullptr) {
			// 1310988: DRV_IMU_DEVTYPE_SIM, BUS: 1, ADDR: 1, TYPE: SIMULATION
			_px4_gyro = new PX4Gyroscope(1310988);

			if (_px4_gyro == nullptr) {
				PX4_ERR("PX4Gyroscope alloc failed");
			}
		}

		if (_px4_gyro != nullptr) {
			_px4_gyro->update(timestamp_sample, hil_state.rollspeed, hil_state.pitchspeed, hil_state.yawspeed);
		}
	}

	/* battery status */
	{
		battery_status_s hil_battery_status{};
		hil_battery_status.voltage_v = 11.1f;
		hil_battery_status.voltage_filtered_v = 11.1f;
		hil_battery_status.current_a = 10.0f;
		hil_battery_status.discharged_mah = -1.0f;
		hil_battery_status.timestamp = hrt_absolute_time();
		hil_battery_status.time_remaining_s = NAN;
		_battery_pub.publish(hil_battery_status);
	}
}

#if !defined(CONSTRAINED_FLASH)
void
MavlinkReceiver::handle_message_named_value_float(mavlink_message_t *msg)
{
	mavlink_named_value_float_t debug_msg;
	mavlink_msg_named_value_float_decode(msg, &debug_msg);

	debug_key_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.key, debug_msg.name, sizeof(debug_topic.key));
	debug_topic.key[sizeof(debug_topic.key) - 1] = '\0'; // enforce null termination
	debug_topic.value = debug_msg.value;

	_debug_key_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug(mavlink_message_t *msg)
{
	mavlink_debug_t debug_msg;
	mavlink_msg_debug_decode(msg, &debug_msg);

	debug_value_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.ind = debug_msg.ind;
	debug_topic.value = debug_msg.value;

	_debug_value_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_vect(mavlink_message_t *msg)
{
	mavlink_debug_vect_t debug_msg;
	mavlink_msg_debug_vect_decode(msg, &debug_msg);

	debug_vect_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination
	debug_topic.x = debug_msg.x;
	debug_topic.y = debug_msg.y;
	debug_topic.z = debug_msg.z;

	_debug_vect_pub.publish(debug_topic);
}

void
MavlinkReceiver::handle_message_debug_float_array(mavlink_message_t *msg)
{
	mavlink_debug_float_array_t debug_msg;
	mavlink_msg_debug_float_array_decode(msg, &debug_msg);

	debug_array_s debug_topic{};

	debug_topic.timestamp = hrt_absolute_time();
	debug_topic.id = debug_msg.array_id;
	memcpy(debug_topic.name, debug_msg.name, sizeof(debug_topic.name));
	debug_topic.name[sizeof(debug_topic.name) - 1] = '\0'; // enforce null termination

	for (size_t i = 0; i < debug_array_s::ARRAY_SIZE; i++) {
		debug_topic.data[i] = debug_msg.data[i];
	}

	_debug_array_pub.publish(debug_topic);
}
#endif // !CONSTRAINED_FLASH

void
MavlinkReceiver::handle_message_onboard_computer_status(mavlink_message_t *msg)
{
	mavlink_onboard_computer_status_t status_msg;
	mavlink_msg_onboard_computer_status_decode(msg, &status_msg);

	onboard_computer_status_s onboard_computer_status_topic{};

	onboard_computer_status_topic.timestamp = hrt_absolute_time();
	onboard_computer_status_topic.uptime = status_msg.uptime;

	onboard_computer_status_topic.type = status_msg.type;

	memcpy(onboard_computer_status_topic.cpu_cores, status_msg.cpu_cores, sizeof(status_msg.cpu_cores));
	memcpy(onboard_computer_status_topic.cpu_combined, status_msg.cpu_combined, sizeof(status_msg.cpu_combined));
	memcpy(onboard_computer_status_topic.gpu_cores, status_msg.gpu_cores, sizeof(status_msg.gpu_cores));
	memcpy(onboard_computer_status_topic.gpu_combined, status_msg.gpu_combined, sizeof(status_msg.gpu_combined));
	onboard_computer_status_topic.temperature_board = status_msg.temperature_board;
	memcpy(onboard_computer_status_topic.temperature_core, status_msg.temperature_core,
	       sizeof(status_msg.temperature_core));
	memcpy(onboard_computer_status_topic.fan_speed, status_msg.fan_speed, sizeof(status_msg.fan_speed));
	onboard_computer_status_topic.ram_usage = status_msg.ram_usage;
	onboard_computer_status_topic.ram_total = status_msg.ram_total;
	memcpy(onboard_computer_status_topic.storage_type, status_msg.storage_type, sizeof(status_msg.storage_type));
	memcpy(onboard_computer_status_topic.storage_usage, status_msg.storage_usage, sizeof(status_msg.storage_usage));
	memcpy(onboard_computer_status_topic.storage_total, status_msg.storage_total, sizeof(status_msg.storage_total));
	memcpy(onboard_computer_status_topic.link_type, status_msg.link_type, sizeof(status_msg.link_type));
	memcpy(onboard_computer_status_topic.link_tx_rate, status_msg.link_tx_rate, sizeof(status_msg.link_tx_rate));
	memcpy(onboard_computer_status_topic.link_rx_rate, status_msg.link_rx_rate, sizeof(status_msg.link_rx_rate));
	memcpy(onboard_computer_status_topic.link_tx_max, status_msg.link_tx_max, sizeof(status_msg.link_tx_max));
	memcpy(onboard_computer_status_topic.link_rx_max, status_msg.link_rx_max, sizeof(status_msg.link_rx_max));

	_onboard_computer_status_pub.publish(onboard_computer_status_topic);
}

void MavlinkReceiver::handle_message_generator_status(mavlink_message_t *msg)
{
	mavlink_generator_status_t status_msg;
	mavlink_msg_generator_status_decode(msg, &status_msg);

	generator_status_s generator_status{};
	generator_status.timestamp = hrt_absolute_time();
	generator_status.status = status_msg.status;
	generator_status.battery_current = status_msg.battery_current;
	generator_status.load_current = status_msg.load_current;
	generator_status.power_generated = status_msg.power_generated;
	generator_status.bus_voltage = status_msg.bus_voltage;
	generator_status.bat_current_setpoint = status_msg.bat_current_setpoint;
	generator_status.runtime = status_msg.runtime;
	generator_status.time_until_maintenance = status_msg.time_until_maintenance;
	generator_status.generator_speed = status_msg.generator_speed;
	generator_status.rectifier_temperature = status_msg.rectifier_temperature;
	generator_status.generator_temperature = status_msg.generator_temperature;

	_generator_status_pub.publish(generator_status);
}

void MavlinkReceiver::handle_message_statustext(mavlink_message_t *msg)
{
	if (msg->sysid == mavlink_system.sysid) {
		// log message from the same system

		mavlink_statustext_t statustext;
		mavlink_msg_statustext_decode(msg, &statustext);

		if (_mavlink_statustext_handler.should_publish_previous(statustext)) {
			_log_message_pub.publish(_mavlink_statustext_handler.log_message());
		}

		if (_mavlink_statustext_handler.should_publish_current(statustext, hrt_absolute_time())) {
			_log_message_pub.publish(_mavlink_statustext_handler.log_message());
		}
	}
}

void MavlinkReceiver::CheckHeartbeats(const hrt_abstime &t, bool force)
{
	// check HEARTBEATs for timeout
	static constexpr uint64_t TIMEOUT = telemetry_status_s::HEARTBEAT_TIMEOUT_US;

	if (t <= TIMEOUT) {
		return;
	}

	if ((t >= _last_heartbeat_check + (TIMEOUT / 2)) || force) {
		telemetry_status_s &tstatus = _mavlink->telemetry_status();

		tstatus.heartbeat_type_antenna_tracker         = (t <= TIMEOUT + _heartbeat_type_antenna_tracker);
		tstatus.heartbeat_type_gcs                     = (t <= TIMEOUT + _heartbeat_type_gcs);
		tstatus.heartbeat_type_onboard_controller      = (t <= TIMEOUT + _heartbeat_type_onboard_controller);
		tstatus.heartbeat_type_gimbal                  = (t <= TIMEOUT + _heartbeat_type_gimbal);
		tstatus.heartbeat_type_adsb                    = (t <= TIMEOUT + _heartbeat_type_adsb);
		tstatus.heartbeat_type_camera                  = (t <= TIMEOUT + _heartbeat_type_camera);
		tstatus.heartbeat_type_parachute               = (t <= TIMEOUT + _heartbeat_type_parachute);
		tstatus.heartbeat_type_open_drone_id           = (t <= TIMEOUT + _heartbeat_type_open_drone_id);

		tstatus.heartbeat_component_telemetry_radio    = (t <= TIMEOUT + _heartbeat_component_telemetry_radio);
		tstatus.heartbeat_component_log                = (t <= TIMEOUT + _heartbeat_component_log);
		tstatus.heartbeat_component_osd                = (t <= TIMEOUT + _heartbeat_component_osd);
		tstatus.heartbeat_component_obstacle_avoidance = (t <= TIMEOUT + _heartbeat_component_obstacle_avoidance);
		tstatus.heartbeat_component_vio                = (t <= TIMEOUT + _heartbeat_component_visual_inertial_odometry);
		tstatus.heartbeat_component_pairing_manager    = (t <= TIMEOUT + _heartbeat_component_pairing_manager);
		tstatus.heartbeat_component_udp_bridge         = (t <= TIMEOUT + _heartbeat_component_udp_bridge);
		tstatus.heartbeat_component_uart_bridge        = (t <= TIMEOUT + _heartbeat_component_uart_bridge);

		_mavlink->telemetry_status_updated();
		_last_heartbeat_check = t;
	}
}

void MavlinkReceiver::handle_message_request_event(mavlink_message_t *msg)
{
	_mavlink->get_events_protocol().handle_request_event(*msg);
}

void
MavlinkReceiver::handle_message_gimbal_manager_set_manual_control(mavlink_message_t *msg)
{
	mavlink_gimbal_manager_set_manual_control_t set_manual_control_msg;
	mavlink_msg_gimbal_manager_set_manual_control_decode(msg, &set_manual_control_msg);

	gimbal_manager_set_manual_control_s set_manual_control{};
	set_manual_control.timestamp = hrt_absolute_time();
	set_manual_control.origin_sysid = msg->sysid;
	set_manual_control.origin_compid = msg->compid;
	set_manual_control.target_system = set_manual_control_msg.target_system;
	set_manual_control.target_component = set_manual_control_msg.target_component;
	set_manual_control.flags = set_manual_control_msg.flags;
	set_manual_control.gimbal_device_id = set_manual_control_msg.gimbal_device_id;

	set_manual_control.pitch = set_manual_control_msg.pitch;
	set_manual_control.yaw = set_manual_control_msg.yaw;
	set_manual_control.pitch_rate = set_manual_control_msg.pitch_rate;
	set_manual_control.yaw_rate = set_manual_control_msg.yaw_rate;

	_gimbal_manager_set_manual_control_pub.publish(set_manual_control);
}

void
MavlinkReceiver::handle_message_gimbal_manager_set_attitude(mavlink_message_t *msg)
{
	mavlink_gimbal_manager_set_attitude_t set_attitude_msg;
	mavlink_msg_gimbal_manager_set_attitude_decode(msg, &set_attitude_msg);

	gimbal_manager_set_attitude_s gimbal_attitude{};
	gimbal_attitude.timestamp = hrt_absolute_time();
	gimbal_attitude.origin_sysid = msg->sysid;
	gimbal_attitude.origin_compid = msg->compid;
	gimbal_attitude.target_system = set_attitude_msg.target_system;
	gimbal_attitude.target_component = set_attitude_msg.target_component;
	gimbal_attitude.flags = set_attitude_msg.flags;
	gimbal_attitude.gimbal_device_id = set_attitude_msg.gimbal_device_id;

	matrix::Quatf q(set_attitude_msg.q);
	q.copyTo(gimbal_attitude.q);

	gimbal_attitude.angular_velocity_x = set_attitude_msg.angular_velocity_x;
	gimbal_attitude.angular_velocity_y = set_attitude_msg.angular_velocity_y;
	gimbal_attitude.angular_velocity_z = set_attitude_msg.angular_velocity_z;

	_gimbal_manager_set_attitude_pub.publish(gimbal_attitude);
}

void
MavlinkReceiver::handle_message_gimbal_device_information(mavlink_message_t *msg)
{

	mavlink_gimbal_device_information_t gimbal_device_info_msg;
	mavlink_msg_gimbal_device_information_decode(msg, &gimbal_device_info_msg);

	gimbal_device_information_s gimbal_information{};
	gimbal_information.timestamp = hrt_absolute_time();

	static_assert(sizeof(gimbal_information.vendor_name) == sizeof(gimbal_device_info_msg.vendor_name),
		      "vendor_name length doesn't match");
	static_assert(sizeof(gimbal_information.model_name) == sizeof(gimbal_device_info_msg.model_name),
		      "model_name length doesn't match");
	static_assert(sizeof(gimbal_information.custom_name) == sizeof(gimbal_device_info_msg.custom_name),
		      "custom_name length doesn't match");
	memcpy(gimbal_information.vendor_name, gimbal_device_info_msg.vendor_name, sizeof(gimbal_information.vendor_name));
	memcpy(gimbal_information.model_name, gimbal_device_info_msg.model_name, sizeof(gimbal_information.model_name));
	memcpy(gimbal_information.custom_name, gimbal_device_info_msg.custom_name, sizeof(gimbal_information.custom_name));
	gimbal_device_info_msg.vendor_name[sizeof(gimbal_device_info_msg.vendor_name) - 1] = '\0';
	gimbal_device_info_msg.model_name[sizeof(gimbal_device_info_msg.model_name) - 1] = '\0';
	gimbal_device_info_msg.custom_name[sizeof(gimbal_device_info_msg.custom_name) - 1] = '\0';

	gimbal_information.firmware_version = gimbal_device_info_msg.firmware_version;
	gimbal_information.hardware_version = gimbal_device_info_msg.hardware_version;
	gimbal_information.cap_flags = gimbal_device_info_msg.cap_flags;
	gimbal_information.custom_cap_flags = gimbal_device_info_msg.custom_cap_flags;
	gimbal_information.uid = gimbal_device_info_msg.uid;

	gimbal_information.pitch_max = gimbal_device_info_msg.pitch_max;
	gimbal_information.pitch_min = gimbal_device_info_msg.pitch_min;

	gimbal_information.yaw_max = gimbal_device_info_msg.yaw_max;
	gimbal_information.yaw_min = gimbal_device_info_msg.yaw_min;

	gimbal_information.gimbal_device_compid = msg->compid;

	_gimbal_device_information_pub.publish(gimbal_information);
}

void
MavlinkReceiver::handle_message_gimbal_device_attitude_status(mavlink_message_t *msg)
{
	mavlink_gimbal_device_attitude_status_t gimbal_device_attitude_status_msg;
	mavlink_msg_gimbal_device_attitude_status_decode(msg, &gimbal_device_attitude_status_msg);

	gimbal_device_attitude_status_s gimbal_attitude_status{};
	gimbal_attitude_status.timestamp = hrt_absolute_time();
	gimbal_attitude_status.target_system = gimbal_device_attitude_status_msg.target_system;
	gimbal_attitude_status.target_component = gimbal_device_attitude_status_msg.target_component;
	gimbal_attitude_status.device_flags = gimbal_device_attitude_status_msg.flags;

	for (unsigned i = 0; i < 4; ++i) {
		gimbal_attitude_status.q[i] = gimbal_device_attitude_status_msg.q[i];
	}

	gimbal_attitude_status.angular_velocity_x = gimbal_device_attitude_status_msg.angular_velocity_x;
	gimbal_attitude_status.angular_velocity_y = gimbal_device_attitude_status_msg.angular_velocity_y;
	gimbal_attitude_status.angular_velocity_z = gimbal_device_attitude_status_msg.angular_velocity_z;
	gimbal_attitude_status.failure_flags = gimbal_device_attitude_status_msg.failure_flags;

	gimbal_attitude_status.received_from_mavlink = true;

	_gimbal_device_attitude_status_pub.publish(gimbal_attitude_status);
}

void
MavlinkReceiver::run()
{
	/* set thread name */
	{
		char thread_name[17];
		snprintf(thread_name, sizeof(thread_name), "mavlink_rcv_if%d", _mavlink->get_instance_id());
		px4_prctl(PR_SET_NAME, thread_name, px4_getpid());
	}

	// poll timeout in ms. Also defines the max update frequency of the mission & param manager, etc.
	const int timeout = 10;

#if defined(__PX4_POSIX)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1600 * 5];
#elif defined(CONFIG_NET)
	/* 1500 is the Wifi MTU, so we make sure to fit a full packet */
	uint8_t buf[1000];
#else
	/* the serial port buffers internally as well, we just need to fit a small chunk */
	uint8_t buf[64];
#endif
	mavlink_message_t msg;

	struct pollfd fds[1] = {};

	if (_mavlink->get_protocol() == Protocol::SERIAL) {
		fds[0].fd = _mavlink->get_uart_fd();
		fds[0].events = POLLIN;
	}

#if defined(MAVLINK_UDP)
	struct sockaddr_in srcaddr = {};
	socklen_t addrlen = sizeof(srcaddr);

	if (_mavlink->get_protocol() == Protocol::UDP) {
		fds[0].fd = _mavlink->get_socket_fd();
		fds[0].events = POLLIN;
	}

#endif // MAVLINK_UDP

	ssize_t nread = 0;
	hrt_abstime last_send_update = 0;

	while (!_mavlink->should_exit()) {

		// check for parameter updates
		if (_parameter_update_sub.updated()) {
			// clear update
			parameter_update_s pupdate;
			_parameter_update_sub.copy(&pupdate);

			// update parameters from storage
			updateParams();
		}

		int ret = poll(&fds[0], 1, timeout);

		if (ret > 0) {
			if (_mavlink->get_protocol() == Protocol::SERIAL) {
				/* non-blocking read. read may return negative values */
				nread = ::read(fds[0].fd, buf, sizeof(buf));

				if (nread == -1 && errno == ENOTCONN) { // Not connected (can happen for USB)
					usleep(100000);
				}
			}

#if defined(MAVLINK_UDP)

			else if (_mavlink->get_protocol() == Protocol::UDP) {
				if (fds[0].revents & POLLIN) {
					nread = recvfrom(_mavlink->get_socket_fd(), buf, sizeof(buf), 0, (struct sockaddr *)&srcaddr, &addrlen);
				}

				struct sockaddr_in &srcaddr_last = _mavlink->get_client_source_address();

				int localhost = (127 << 24) + 1;

				if (!_mavlink->get_client_source_initialized()) {

					// set the address either if localhost or if 3 seconds have passed
					// this ensures that a GCS running on localhost can get a hold of
					// the system within the first N seconds
					hrt_abstime stime = _mavlink->get_start_time();

					if ((stime != 0 && (hrt_elapsed_time(&stime) > 3_s))
					    || (srcaddr_last.sin_addr.s_addr == htonl(localhost))) {

						srcaddr_last.sin_addr.s_addr = srcaddr.sin_addr.s_addr;
						srcaddr_last.sin_port = srcaddr.sin_port;

						_mavlink->set_client_source_initialized();

						PX4_INFO("partner IP: %s", inet_ntoa(srcaddr.sin_addr));
					}
				}
			}

			// only start accepting messages on UDP once we're sure who we talk to
			if (_mavlink->get_protocol() != Protocol::UDP || _mavlink->get_client_source_initialized()) {
#endif // MAVLINK_UDP

				/* if read failed, this loop won't execute */
				for (ssize_t i = 0; i < nread; i++) {
					if (mavlink_parse_char(_mavlink->get_channel(), buf[i], &msg, &_status)) {

						/* check if we received version 2 and request a switch. */
						if (!(_mavlink->get_status()->flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1)) {
							/* this will only switch to proto version 2 if allowed in settings */
							_mavlink->set_proto_version(2);
						}

						/* handle generic messages and commands */
						handle_message(&msg);

						/* handle packet with mission manager */
						_mission_manager.handle_message(&msg);

						/* handle packet with parameter component */
						if (_mavlink->boot_complete()) {
							// make sure mavlink app has booted before we start processing parameter sync
							_parameters_manager.handle_message(&msg);

						} else {
							if (hrt_elapsed_time(&_mavlink->get_first_start_time()) > 20_s) {
								PX4_ERR("system boot did not complete in 20 seconds");
								_mavlink->set_boot_complete();
							}
						}

						if (_mavlink->ftp_enabled()) {
							/* handle packet with ftp component */
							_mavlink_ftp.handle_message(&msg);
						}

						/* handle packet with log component */
						_mavlink_log_handler.handle_message(&msg);

						/* handle packet with timesync component */
						_mavlink_timesync.handle_message(&msg);

						/* handle packet with parent object */
						_mavlink->handle_message(&msg);

						update_rx_stats(msg);

						if (_message_statistics_enabled) {
							update_message_statistics(msg);
						}
					}
				}

				/* count received bytes (nread will be -1 on read error) */
				if (nread > 0) {
					_mavlink->count_rxbytes(nread);

					telemetry_status_s &tstatus = _mavlink->telemetry_status();
					tstatus.rx_message_count = _total_received_counter;
					tstatus.rx_message_lost_count = _total_lost_counter;
					tstatus.rx_message_lost_rate = static_cast<float>(_total_lost_counter) / static_cast<float>(_total_received_counter);

					if (_mavlink_status_last_buffer_overrun != _status.buffer_overrun) {
						tstatus.rx_buffer_overruns++;
						_mavlink_status_last_buffer_overrun = _status.buffer_overrun;
					}

					if (_mavlink_status_last_parse_error != _status.parse_error) {
						tstatus.rx_parse_errors++;
						_mavlink_status_last_parse_error = _status.parse_error;
					}

					if (_mavlink_status_last_packet_rx_drop_count != _status.packet_rx_drop_count) {
						tstatus.rx_packet_drop_count++;
						_mavlink_status_last_packet_rx_drop_count = _status.packet_rx_drop_count;
					}
				}

#if defined(MAVLINK_UDP)
			}

#endif // MAVLINK_UDP

		} else if (ret == -1) {
			usleep(10000);
		}

		const hrt_abstime t = hrt_absolute_time();

		CheckHeartbeats(t);

		if (t - last_send_update > timeout * 1000) {
			_mission_manager.check_active_mission();
			_mission_manager.send();

			_parameters_manager.send();

			if (_mavlink->ftp_enabled()) {
				_mavlink_ftp.send();
			}

			_mavlink_log_handler.send();
			last_send_update = t;
		}

		if (_tune_publisher != nullptr) {
			_tune_publisher->publish_next_tune(t);
		}
	}
}

bool MavlinkReceiver::component_was_seen(int system_id, int component_id)
{
	// For system broadcast messages return true if at least one component was seen before
	if (system_id == 0) {
		return _component_states_count > 0;
	}

	for (unsigned i = 0; i < _component_states_count; ++i) {
		if (_component_states[i].system_id == system_id
		    && (component_id == 0 || _component_states[i].component_id == component_id)) {
			return true;
		}
	}

	return false;
}

void MavlinkReceiver::update_rx_stats(const mavlink_message_t &message)
{
	const bool component_states_has_still_space = [this, &message]() {
		for (unsigned i = 0; i < MAX_REMOTE_COMPONENTS; ++i) {
			if (_component_states[i].system_id == message.sysid && _component_states[i].component_id == message.compid) {

				int lost_messages = 0;
				const uint8_t expected_seq = _component_states[i].last_sequence + 1;

				// Account for overflow during packet loss
				if (message.seq < expected_seq) {
					lost_messages = (message.seq + 255) - expected_seq;

				} else {
					lost_messages = message.seq - expected_seq;
				}

				_component_states[i].missed_messages += lost_messages;

				++_component_states[i].received_messages;
				_component_states[i].last_sequence = message.seq;

				// Also update overall stats
				++_total_received_counter;
				_total_lost_counter += lost_messages;

				return true;

			} else if (_component_states[i].system_id == 0 && _component_states[i].component_id == 0) {
				_component_states[i].system_id = message.sysid;
				_component_states[i].component_id = message.compid;

				++_component_states[i].received_messages;
				_component_states[i].last_sequence = message.seq;

				_component_states_count = i + 1;

				// Also update overall stats
				++_total_received_counter;

				return true;
			}
		}

		return false;
	}();

	if (!component_states_has_still_space && !_warned_component_states_full_once) {
		PX4_WARN("Max remote components of %u used up", MAX_REMOTE_COMPONENTS);
		_warned_component_states_full_once = true;
	}
}

void MavlinkReceiver::update_message_statistics(const mavlink_message_t &message)
{
#if !defined(CONSTRAINED_FLASH)

	if (_received_msg_stats == nullptr) {
		_received_msg_stats = new ReceivedMessageStats[MAX_MSG_STAT_SLOTS];
	}

	if (_received_msg_stats) {
		const hrt_abstime now_ms = hrt_absolute_time() / 1000;

		int msg_stats_slot = -1;
		bool reset_stats = false;

		// find matching msg id
		for (int stat_slot = 0; stat_slot < MAX_MSG_STAT_SLOTS; stat_slot++) {
			if ((_received_msg_stats[stat_slot].msg_id == message.msgid)
			    && (_received_msg_stats[stat_slot].system_id == message.sysid)
			    && (_received_msg_stats[stat_slot].component_id == message.compid)) {

				msg_stats_slot = stat_slot;
				break;
			}
		}

		// otherwise find oldest or empty slot
		if (msg_stats_slot < 0) {
			uint32_t oldest_slot_time_ms = 0;

			for (int stat_slot = 0; stat_slot < MAX_MSG_STAT_SLOTS; stat_slot++) {
				if (_received_msg_stats[stat_slot].last_time_received_ms <= oldest_slot_time_ms) {
					oldest_slot_time_ms = _received_msg_stats[stat_slot].last_time_received_ms;
					msg_stats_slot = stat_slot;
				}
			}

			reset_stats = true;
		}

		if (msg_stats_slot >= 0) {
			if (!reset_stats) {
				if ((_received_msg_stats[msg_stats_slot].last_time_received_ms != 0)
				    && (now_ms > _received_msg_stats[msg_stats_slot].last_time_received_ms)) {

					float rate = 1000.f / (now_ms - _received_msg_stats[msg_stats_slot].last_time_received_ms);

					if (PX4_ISFINITE(_received_msg_stats[msg_stats_slot].avg_rate_hz)) {
						_received_msg_stats[msg_stats_slot].avg_rate_hz = 0.9f * _received_msg_stats[msg_stats_slot].avg_rate_hz + 0.1f * rate;

					} else {
						_received_msg_stats[msg_stats_slot].avg_rate_hz = rate;
					}

				} else {
					_received_msg_stats[msg_stats_slot].avg_rate_hz = 0.f;
				}

			} else {
				_received_msg_stats[msg_stats_slot].avg_rate_hz = NAN;
			}

			_received_msg_stats[msg_stats_slot].last_time_received_ms = now_ms;
			_received_msg_stats[msg_stats_slot].msg_id = message.msgid;
			_received_msg_stats[msg_stats_slot].system_id = message.sysid;
			_received_msg_stats[msg_stats_slot].component_id = message.compid;
		}
	}

#endif // !CONSTRAINED_FLASH
}

void MavlinkReceiver::print_detailed_rx_stats() const
{
	// TODO: add mutex around shared data.
	if (_component_states_count > 0) {
		printf("\tReceived Messages:\n");

		for (const auto &comp_stat : _component_states) {
			if (comp_stat.received_messages > 0) {
				printf("\t  sysid:%3" PRIu8 ", compid:%3" PRIu8 ", Total: %" PRIu32 " (lost: %" PRIu32 ")\n",
				       comp_stat.system_id, comp_stat.component_id,
				       comp_stat.received_messages, comp_stat.missed_messages);

#if !defined(CONSTRAINED_FLASH)

				if (_message_statistics_enabled && _received_msg_stats) {
					for (int i = 0; i < MAX_MSG_STAT_SLOTS; i++) {
						const ReceivedMessageStats &msg_stat = _received_msg_stats[i];

						const uint32_t now_ms = hrt_absolute_time() / 1000;

						// valid messages received within the last 10 seconds
						if ((msg_stat.system_id == comp_stat.system_id)
						    && (msg_stat.component_id == comp_stat.component_id)
						    && (msg_stat.last_time_received_ms != 0)
						    && (now_ms - msg_stat.last_time_received_ms < 10'000)) {

							const float elapsed_s = (now_ms - msg_stat.last_time_received_ms) / 1000.f;

							printf("\t    msgid:%5" PRIu16 ", Rate:%5.1f Hz, last %.2fs ago\n",
							       msg_stat.msg_id, (double)msg_stat.avg_rate_hz, (double)elapsed_s);
						}
					}
				}

#endif // !CONSTRAINED_FLASH
			}
		}
	}
}

void MavlinkReceiver::start()
{
	pthread_attr_t receiveloop_attr;
	pthread_attr_init(&receiveloop_attr);

	struct sched_param param;
	(void)pthread_attr_getschedparam(&receiveloop_attr, &param);
	param.sched_priority = SCHED_PRIORITY_MAX - 80;
	(void)pthread_attr_setschedparam(&receiveloop_attr, &param);

	pthread_attr_setstacksize(&receiveloop_attr,
				  PX4_STACK_ADJUSTED(sizeof(MavlinkReceiver) + 2840 + MAVLINK_RECEIVER_NET_ADDED_STACK));

	pthread_create(&_thread, &receiveloop_attr, MavlinkReceiver::start_trampoline, (void *)this);

	pthread_attr_destroy(&receiveloop_attr);
}

void
MavlinkReceiver::updateParams()
{
	// update parameters from storage
	ModuleParams::updateParams();
}

void *MavlinkReceiver::start_trampoline(void *context)
{
	MavlinkReceiver *self = reinterpret_cast<MavlinkReceiver *>(context);
	self->run();
	return nullptr;
}

void MavlinkReceiver::stop()
{
	_should_exit.store(true);
	pthread_join(_thread, nullptr);
}
/****************************************************************************
 *
 *   Copyright (c) 2016-2022 PX4 Development Team. All rights reserved.
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
 * @file mavlink_shell.cpp
 * A shell to be used via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "mavlink_shell.h"
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>

#ifdef __PX4_NUTTX
#include <nshlib/nshlib.h>
#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX
#include "../../../platforms/posix/src/px4/common/px4_daemon/pxh.h"
#endif /* __PX4_POSIX */

#ifdef __PX4_CYGWIN
#include <asm/socket.h>
#endif

MavlinkShell::~MavlinkShell()
{
	//closing the pipes will stop the thread as well
	if (_to_shell_fd >= 0) {
		PX4_INFO("Stopping mavlink shell");
		close(_to_shell_fd);
	}

	if (_from_shell_fd >= 0) {
		close(_from_shell_fd);
	}
}

int MavlinkShell::start()
{
	//this currently only works for NuttX & POSIX
#if !defined(__PX4_NUTTX) && !defined(__PX4_POSIX)
	return -1;
#endif

	PX4_INFO("Starting mavlink shell");

	int p1[2], p2[2];

	/* Create the shell task and redirect its stdin & stdout. If we used pthread, we would redirect
	 * stdin/out of the calling process as well, so we need px4_task_spawn_cmd. However NuttX only
	 * keeps (duplicates) the first 3 fd's when creating a new task, all others are not inherited.
	 * This means we need to temporarily change the first 3 fd's of the current task (or at least
	 * the first 2 if stdout=stderr).
	 */

	if (pipe(p1) != 0) {
		return -errno;
	}

	if (pipe(p2) != 0) {
		close(p1[0]);
		close(p1[1]);
		return -errno;
	}

	int ret = 0;

	_from_shell_fd  = p1[0];
	_to_shell_fd = p2[1];
	_shell_fds[0]  = p2[0];
	_shell_fds[1] = p1[1];

	/*
	 * Ensure that during the temporary phase no other thread from the same task writes to
	 * stdout (as it would end up in the pipe).
	 */
#ifdef __PX4_NUTTX
	sched_lock();
#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX
	int remote_in_fd = dup(_shell_fds[0]);	// Input file descriptor for the remote shell
	int remote_out_fd = dup(_shell_fds[1]); // Output file descriptor for the remote shell

	char r_in[32];
	char r_out[32];
	sprintf(r_in, "%d", remote_in_fd);
	sprintf(r_out, "%d", remote_out_fd);
	char *const argv[3] = {r_in, r_out, nullptr};

#else
	int fd_backups[2]; //we don't touch stderr, we will redirect it to stdout in the startup of the shell task

	for (int i = 0; i < 2; ++i) {
		fd_backups[i] = dup(i);

		if (fd_backups[i] == -1) {
			ret = -errno;
		}
	}

	dup2(_shell_fds[0], 0);
	dup2(_shell_fds[1], 1);
#endif

	if (ret == 0) {
		_task = px4_task_spawn_cmd("mavlink_shell",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_DEFAULT,
					   2048,
					   &MavlinkShell::shell_start_thread,
#ifdef __PX4_POSIX
					   argv);
#else
					   nullptr);
#endif

		if (_task < 0) {
			ret = -1;
		}
	}

#if !defined(__PX4_POSIX)

	//restore fd's
	for (int i = 0; i < 2; ++i) {
		if (dup2(fd_backups[i], i) == -1) {
			ret = -errno;
		}

		close(fd_backups[i]);
	}

#endif

	//close unused pipe fd's
	close(_shell_fds[0]);
	close(_shell_fds[1]);

#ifdef __PX4_NUTTX
	sched_unlock();
#endif /* __PX4_NUTTX */

	return ret;
}

int MavlinkShell::shell_start_thread(int argc, char *argv[])
{
#ifdef __PX4_NUTTX
	dup2(1, 2); //redirect stderror to stdout

	nsh_consolemain(0, NULL);
#endif /* __PX4_NUTTX */

#ifdef __PX4_POSIX

	if (argc != 3) {
		PX4_ERR("Mavlink shell bug");
		return -1;
	}

	int remote_in_fd = atoi(argv[1]);
	int remote_out_fd = atoi(argv[2]);

	px4_daemon::Pxh pxh;
	pxh.run_remote_pxh(remote_in_fd, remote_out_fd);
#endif

	return 0;
}

size_t MavlinkShell::write(uint8_t *buffer, size_t len)
{
	return ::write(_to_shell_fd, buffer, len);
}

size_t MavlinkShell::read(uint8_t *buffer, size_t len)
{
	return ::read(_from_shell_fd, buffer, len);
}

size_t MavlinkShell::available()
{
	int ret = 0;

	if (ioctl(_from_shell_fd, FIONREAD, (unsigned long)&ret) == OK) {
		return ret;
	}

	return 0;
}
/****************************************************************************
 *
 *   Copyright (c) 2015-2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_simple_analyzer.cpp
 *
 * @author Achermann Florian <acfloria@ethz.ch>
 */

#include "mavlink_simple_analyzer.h"

#include <float.h>

#include <px4_platform_common/log.h>
#include <px4_platform_common/defines.h>

SimpleAnalyzer::SimpleAnalyzer(Mode mode, float window) :
	_window(window),
	_mode(mode)
{
	reset();
}

void SimpleAnalyzer::reset()
{
	_n = 0;

	switch (_mode) {
	case AVERAGE:
		_result = 0.0f;

		break;

	case MIN:
		_result = FLT_MAX;

		break;

	case MAX:
		_result = FLT_MIN;

		break;

	default:
		PX4_ERR("SimpleAnalyzer: Unknown mode.");
	}
}

void SimpleAnalyzer::add_value(float val, float update_rate)
{
	switch (_mode) {
	case AVERAGE:
		_result = (_result * _n + val) / (_n + 1u);

		break;

	case MIN:
		if (val < _result) {
			_result = val;
		}

		break;

	case MAX:
		if (val > _result) {
			_result = val;
		}

		break;
	}

	// if we get more measurements than n_max so the exponential moving average
	// is computed
	if ((_n < update_rate * _window) && (update_rate > 1.0f)) {
		_n++;
	}

	// value sanity checks
	if (!PX4_ISFINITE(_result)) {
		PX4_DEBUG("SimpleAnalyzer: Result is not finite, reset the analyzer.");
		reset();
	}
}

bool SimpleAnalyzer::valid() const
{
	return _n > 0u;
}

float SimpleAnalyzer::get() const
{
	return _result;
}

float SimpleAnalyzer::get_scaled(float scalingfactor) const
{
	return get() * scalingfactor;
}

void SimpleAnalyzer::check_limits(float &x, float min, float max) const
{
	if (x > max) {
		x = max;

	} else if (x < min) {
		x = min;
	}
}

void SimpleAnalyzer::int_round(float &x) const
{
	if (x < 0) {
		x -= 0.5f;

	} else {
		x += 0.5f;
	}
}

void convert_limit_safe(float in, uint16_t &out)
{
	if (in > UINT16_MAX) {
		out = UINT16_MAX;

	} else if (in < 0) {
		out = 0;

	} else {
		out = static_cast<uint16_t>(in);
	}
}

void convert_limit_safe(float in, int16_t &out)
{
	if (in > INT16_MAX) {
		out = INT16_MAX;

	} else if (in < INT16_MIN) {
		out = INT16_MIN;

	} else {
		out = static_cast<int16_t>(in);
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file mavlink_stream.cpp
 * Mavlink messages stream implementation.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include <stdlib.h>

#include "mavlink_stream.h"
#include "mavlink_main.h"

MavlinkStream::MavlinkStream(Mavlink *mavlink) :
	_mavlink(mavlink)
{
	_last_sent = hrt_absolute_time();
}

/**
 * Update subscriptions and send message if necessary
 */
int
MavlinkStream::update(const hrt_abstime &t)
{
	update_data();

	// If the message has never been sent before we want
	// to send it immediately and can return right away
	if (_last_sent == 0) {
		// this will give different messages on the same run a different
		// initial timestamp which will help spacing them out
		// on the link scheduling
		if (send()) {
			_last_sent = hrt_absolute_time();

			if (!_first_message_sent) {
				_first_message_sent = true;
			}
		}

		return 0;
	}

	// One of the previous iterations sent the update
	// already before the deadline
	if (_last_sent > t) {
		return -1;
	}

	int64_t dt = t - _last_sent;
	int interval = _interval;

	if (!const_rate()) {
		interval /= _mavlink->get_rate_mult();
	}

	// We don't need to send anything if the inverval is 0. send() will be called manually.
	if (interval == 0) {
		return 0;
	}

	const bool unlimited_rate = interval < 0;

	// Send the message if it is due or
	// if it will overrun the next scheduled send interval
	// by 30% of the interval time. This helps to avoid
	// sending a scheduled message on average slower than
	// scheduled. Doing this at 50% would risk sending
	// the message too often as the loop runtime of the app
	// needs to be accounted for as well.
	// This method is not theoretically optimal but a suitable
	// stopgap as it hits its deadlines well (0.5 Hz, 50 Hz and 250 Hz)

	if (unlimited_rate || (dt > (interval - (_mavlink->get_main_loop_delay() / 10) * 3))) {
		// interval expired, send message

		// If the interval is non-zero and dt is smaller than 1.5 times the interval
		// do not use the actual time but increment at a fixed rate, so that processing delays do not
		// distort the average rate. The check of the maximum interval is done to ensure that after a
		// long time not sending anything, sending multiple messages in a short time is avoided.
		if (send()) {
			_last_sent = ((interval > 0) && ((int64_t)(1.5f * interval) > dt)) ? _last_sent + interval : t;

			if (!_first_message_sent) {
				_first_message_sent = true;
			}

			return 0;

		} else {
			return -1;
		}
	}

	return -1;
}
/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * @file mavlink_timesync.cpp
 * Mavlink timesync implementation.
 *
 * @author Mohammed Kabir <mhkabir98@gmail.com>
 */

#include "mavlink_timesync.h"
#include "mavlink_main.h"

#include <stdlib.h>

MavlinkTimesync::MavlinkTimesync(Mavlink *mavlink) :
	_mavlink(mavlink)
{
}

void
MavlinkTimesync::handle_message(const mavlink_message_t *msg)
{
	switch (msg->msgid) {
	case MAVLINK_MSG_ID_TIMESYNC: {

			mavlink_timesync_t tsync = {};
			mavlink_msg_timesync_decode(msg, &tsync);

			const uint64_t now = hrt_absolute_time();

			if (tsync.tc1 == 0) {			// Message originating from remote system, timestamp and return it

				mavlink_timesync_t rsync;

				rsync.tc1 = now * 1000ULL;
				rsync.ts1 = tsync.ts1;

				mavlink_msg_timesync_send_struct(_mavlink->get_channel(), &rsync);

				return;

			} else if (tsync.tc1 > 0) {		// Message originating from this system, compute time offset from it

				_timesync.update(now, tsync.tc1, tsync.ts1);
			}

			break;
		}

	case MAVLINK_MSG_ID_SYSTEM_TIME: {

			mavlink_system_time_t time;
			mavlink_msg_system_time_decode(msg, &time);

			timespec tv = {};
			px4_clock_gettime(CLOCK_REALTIME, &tv);

			// date -d @1234567890: Sat Feb 14 02:31:30 MSK 2009
			bool onb_unix_valid = (unsigned long long)tv.tv_sec > PX4_EPOCH_SECS;
			bool ofb_unix_valid = time.time_unix_usec > PX4_EPOCH_SECS * 1000000ULL;

			if (!onb_unix_valid && ofb_unix_valid) {
				tv.tv_sec = time.time_unix_usec / 1000000ULL;
				tv.tv_nsec = (time.time_unix_usec % 1000000ULL) * 1000ULL;

				if (px4_clock_settime(CLOCK_REALTIME, &tv)) {
					PX4_ERR("[timesync] Failed setting realtime clock");
				}
			}

			break;
		}

	default:
		break;
	}
}
/****************************************************************************
 *
 *   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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
 * @file mavlink_ulog.cpp
 * ULog data streaming via MAVLink
 *
 * @author Beat Küng <beat-kueng@gmx.net>
 */

#include "mavlink_ulog.h"
#include <px4_platform_common/log.h>
#include <errno.h>
#include <mathlib/mathlib.h>

bool MavlinkULog::_init = false;
MavlinkULog *MavlinkULog::_instance = nullptr;
px4_sem_t MavlinkULog::_lock;


MavlinkULog::MavlinkULog(int datarate, float max_rate_factor, uint8_t target_system, uint8_t target_component)
	: _target_system(target_system), _target_component(target_component),
	  _max_rate_factor(max_rate_factor),
	  _max_num_messages(math::max(1, (int)ceilf((_rate_calculation_delta_t / 1e6f) * _max_rate_factor * datarate /
				      (MAVLINK_MSG_ID_LOGGING_DATA_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES)))),
	  _current_rate_factor(max_rate_factor)
{
	// make sure we won't read any old messages
	while (_ulog_stream_sub.update()) {

	}

	_waiting_for_initial_ack = true;
	_last_sent_time = hrt_absolute_time(); //(ab)use this timestamp during initialization
	_next_rate_check = _last_sent_time + _rate_calculation_delta_t;
}

MavlinkULog::~MavlinkULog()
{
	perf_free(_msg_missed_ulog_stream_perf);
}

void MavlinkULog::start_ack_received()
{
	if (_waiting_for_initial_ack) {
		_last_sent_time = 0;
		_waiting_for_initial_ack = false;
		PX4_DEBUG("got logger ack");
	}
}

int MavlinkULog::handle_update(mavlink_channel_t channel)
{
	static_assert(sizeof(ulog_stream_s::data) == MAVLINK_MSG_LOGGING_DATA_FIELD_DATA_LEN,
		      "Invalid uorb ulog_stream.data length");
	static_assert(sizeof(ulog_stream_s::data) == MAVLINK_MSG_LOGGING_DATA_ACKED_FIELD_DATA_LEN,
		      "Invalid uorb ulog_stream.data length");

	if (_waiting_for_initial_ack) {
		if (hrt_elapsed_time(&_last_sent_time) > 3e5) {
			PX4_WARN("no ack from logger (is it running?)");
			return -1;
		}

		return 0;
	}

	// check if we're waiting for an ACK
	if (_last_sent_time) {
		bool check_for_updates = false;

		if (_ack_received) {
			_last_sent_time = 0;
			check_for_updates = true;

		} else {

			if (hrt_elapsed_time(&_last_sent_time) > ulog_stream_ack_s::ACK_TIMEOUT * 1000) {
				if (++_sent_tries > ulog_stream_ack_s::ACK_MAX_TRIES) {
					return -ETIMEDOUT;

				} else {
					PX4_DEBUG("re-sending ulog mavlink message (try=%i)", _sent_tries);
					_last_sent_time = hrt_absolute_time();

					const ulog_stream_s &ulog_data = _ulog_stream_sub.get();

					mavlink_logging_data_acked_t msg;
					msg.sequence = ulog_data.msg_sequence;
					msg.length = ulog_data.length;
					msg.first_message_offset = ulog_data.first_message_offset;
					msg.target_system = _target_system;
					msg.target_component = _target_component;
					memcpy(msg.data, ulog_data.data, sizeof(msg.data));
					mavlink_msg_logging_data_acked_send_struct(channel, &msg);
				}
			}
		}

		if (!check_for_updates) {
			return 0;
		}
	}


	while ((_current_num_msgs < _max_num_messages) && _ulog_stream_sub.updated()) {
		const unsigned last_generation = _ulog_stream_sub.get_last_generation();
		_ulog_stream_sub.update();

		if (_ulog_stream_sub.get_last_generation() != last_generation + 1) {
			perf_count(_msg_missed_ulog_stream_perf);
		}

		const ulog_stream_s &ulog_data = _ulog_stream_sub.get();

		if (ulog_data.timestamp > 0) {
			if (ulog_data.flags & ulog_stream_s::FLAGS_NEED_ACK) {
				_sent_tries = 1;
				_last_sent_time = hrt_absolute_time();
				lock();
				_wait_for_ack_sequence = ulog_data.msg_sequence;
				_ack_received = false;
				unlock();

				mavlink_logging_data_acked_t msg;
				msg.sequence = ulog_data.msg_sequence;
				msg.length = ulog_data.length;
				msg.first_message_offset = ulog_data.first_message_offset;
				msg.target_system = _target_system;
				msg.target_component = _target_component;
				memcpy(msg.data, ulog_data.data, sizeof(msg.data));
				mavlink_msg_logging_data_acked_send_struct(channel, &msg);

			} else {
				mavlink_logging_data_t msg;
				msg.sequence = ulog_data.msg_sequence;
				msg.length = ulog_data.length;
				msg.first_message_offset = ulog_data.first_message_offset;
				msg.target_system = _target_system;
				msg.target_component = _target_component;
				memcpy(msg.data, ulog_data.data, sizeof(msg.data));
				mavlink_msg_logging_data_send_struct(channel, &msg);
			}
		}

		++_current_num_msgs;
	}

	//need to update the rate?
	hrt_abstime t = hrt_absolute_time();

	if (t > _next_rate_check) {
		if (_current_num_msgs < _max_num_messages) {
			_current_rate_factor = _max_rate_factor * (float)_current_num_msgs / _max_num_messages;

		} else {
			_current_rate_factor = _max_rate_factor;
		}

		_current_num_msgs = 0;
		_next_rate_check = t + _rate_calculation_delta_t;
		PX4_DEBUG("current rate=%.3f (max=%i msgs in %.3fs)", (double)_current_rate_factor, _max_num_messages,
			  (double)(_rate_calculation_delta_t / 1e6));
	}

	return 0;
}

void MavlinkULog::initialize()
{
	if (_init) {
		return;
	}

	px4_sem_init(&_lock, 1, 1);
	_init = true;
}

MavlinkULog *MavlinkULog::try_start(int datarate, float max_rate_factor, uint8_t target_system,
				    uint8_t target_component)
{
	MavlinkULog *ret = nullptr;
	bool failed = false;
	lock();

	if (!_instance) {
		ret = _instance = new MavlinkULog(datarate, max_rate_factor, target_system, target_component);

		if (!_instance) {
			failed = true;
		}
	}

	unlock();

	if (failed) {
		PX4_ERR("alloc failed");
	}

	return ret;
}

void MavlinkULog::stop()
{
	lock();

	if (_instance) {
		delete _instance;
		_instance = nullptr;
	}

	unlock();
}

void MavlinkULog::handle_ack(mavlink_logging_ack_t ack)
{
	lock();

	if (_instance) { // make sure stop() was not called right before
		if (_wait_for_ack_sequence == ack.sequence) {
			_ack_received = true;
			publish_ack(ack.sequence);
		}
	}

	unlock();
}

void MavlinkULog::publish_ack(uint16_t sequence)
{
	ulog_stream_ack_s ack;
	ack.timestamp = hrt_absolute_time();
	ack.msg_sequence = sequence;

	_ulog_stream_ack_pub.publish(ack);
}
/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "MavlinkStatustextHandler.hpp"
#include <mathlib/mathlib.h>

bool MavlinkStatustextHandler::should_publish_previous(const mavlink_statustext_t &msg_statustext)
{
	// Check if the previous message has not been published yet. This can
	// happen if the last chunk has been dropped, let's publish what we have.
	if (_last_log_id > 0 && msg_statustext.id != _last_log_id) {
		// We add a note that a part is missing at the end.
		const size_t offset = strlen(_log_msg.text);
		strncpy(_log_msg.text + offset, "[ missing ... ]",
			sizeof(_log_msg.text) - offset);
		_log_msg.text[sizeof(_log_msg.text) - 1] = '\0';

		return true;
	}

	return false;
}

bool MavlinkStatustextHandler::should_publish_current(const mavlink_statustext_t &msg_statustext, const uint64_t &now)
{
	if (msg_statustext.id > 0) {
		// Multi statustext message.
		if (_last_log_id != msg_statustext.id) {
			// On the first one arriving, we save the timestamp and severity.
			_log_msg.timestamp = now;
			_log_msg.severity = msg_statustext.severity;
		}

		if (msg_statustext.chunk_seq == 0) {
			// We start from 0 with a new message.
			strncpy(_log_msg.text, msg_statustext.text,
				math::min(sizeof(_log_msg.text), sizeof(msg_statustext.text)));
			_log_msg.text[sizeof(msg_statustext.text)] = '\0';

		} else {
			if (msg_statustext.chunk_seq != _last_log_chunk_seq + 1) {
				const size_t offset = strlen(_log_msg.text);
				strncpy(_log_msg.text + offset, "[ missing ... ]",
					sizeof(_log_msg.text) - offset);
				_log_msg.text[sizeof(_log_msg.text) - offset - 1] = '\0';
			}

			// We add a consecutive chunk.
			const size_t offset = strlen(_log_msg.text);
			const size_t max_to_add = math::min(sizeof(_log_msg.text) - offset - 1, sizeof(msg_statustext.text));
			strncpy(_log_msg.text + offset, msg_statustext.text, max_to_add);
			_log_msg.text[math::min(offset + max_to_add, sizeof(_log_msg.text) - 1)] = '\0';
		}

		_last_log_chunk_seq = msg_statustext.chunk_seq;

		const bool publication_message_is_full = sizeof(_log_msg.text) - 1 == strlen(_log_msg.text);
		const bool found_zero_termination = strnlen(msg_statustext.text,
						    sizeof(msg_statustext.text)) < sizeof(msg_statustext.text);

		if (publication_message_is_full || found_zero_termination) {
			_last_log_id = 0;
			_last_log_chunk_seq = -1;
			return true;

		} else {
			_last_log_id = msg_statustext.id;
			return false;
		}

	} else {
		// Single statustext message.
		_log_msg.timestamp = now;
		_log_msg.severity = msg_statustext.severity;

		static_assert(sizeof(_log_msg.text) > sizeof(msg_statustext.text),
			      "_log_msg.text not big enough to hold msg_statustext.text");

		strncpy(_log_msg.text, msg_statustext.text,
			math::min(sizeof(_log_msg.text),
				  sizeof(msg_statustext.text)));

		// We need to 0-terminate after the copied text which does not have to be
		// 0-terminated on the wire.
		_log_msg.text[sizeof(msg_statustext.text) - 1] = '\0';

		_last_log_id = 0;
		return true;

	}
}
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


#include "tune_publisher.h"
#include "string.h"
#include <px4_platform_common/log.h>

void TunePublisher::set_tune_string(const char *tune, const hrt_abstime &now)
{
	// The tune string needs to be 0 terminated.
	const unsigned tune_len = strlen(tune);

	// We don't expect the tune string to be longer than what can come in via MAVLink including 0 termination.
	if (tune_len >= MAX_TUNE_LEN) {
		PX4_ERR("Tune string too long.");
		return;
	}

	strncpy(_tune_buffer, tune, MAX_TUNE_LEN);

	_tunes.set_string(_tune_buffer, tune_control_s::VOLUME_LEVEL_DEFAULT);

	_next_publish_time = now;
}


void TunePublisher::publish_next_tune(const hrt_abstime now)
{
	if (_next_publish_time == 0) {
		// Nothing to play.
		return;
	}

	if (now < _next_publish_time) {
		// Too early, try again later.
		return;
	}

	unsigned frequency;
	unsigned duration;
	unsigned silence;
	uint8_t volume;

	if (_tunes.get_next_note(frequency, duration, silence, volume) == Tunes::Status::Continue) {
		tune_control_s tune_control{};
		tune_control.frequency = static_cast<uint16_t>(frequency);
		tune_control.duration = static_cast<uint32_t>(duration);
		tune_control.silence = static_cast<uint32_t>(silence);
		tune_control.volume = static_cast<uint8_t>(volume);
		tune_control.timestamp = hrt_absolute_time();
		_tune_control_pub.publish(tune_control);

		_next_publish_time = now + duration + silence;

	} else {
		// We're done, let's reset.
		_next_publish_time = 0;
	}
}
