/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

#include "rplidar.hpp"

#include <inttypes.h>
#include <string.h>

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <ObstacleMath.hpp>

using namespace rplidar;

RPLidar::RPLidar(const char *port) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_uart(port, 256000),
	_revolution_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": revolution")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": com_err")),
	_resync_perf(perf_alloc(PC_COUNT, MODULE_NAME": resync"))
{
	_obstacle_distance.frame = obstacle_distance_s::MAV_FRAME_BODY_FRD;
	_obstacle_distance.sensor_type = obstacle_distance_s::MAV_DISTANCE_SENSOR_LASER;
	_obstacle_distance.increment = BIN_WIDTH_DEG;
	_obstacle_distance.angle_offset = 0.f;

	for (int i = 0; i < BIN_COUNT; i++) {
		_obstacle_distance.distances[i] = UINT16_MAX;
		_bin_min_cm[i] = UINT16_MAX;
		_bin_seen[i] = false;
	}
}

RPLidar::~RPLidar()
{
	ScheduleClear();

	if (_uart.isOpen()) {
		// leave the sensor idle instead of streaming into a closed port
		sendCommand(CMD_STOP);
		_uart.close();
	}

	perf_free(_revolution_perf);
	perf_free(_comms_errors);
	perf_free(_resync_perf);
}

int RPLidar::init()
{
	int32_t baudrate = 256000;
	int32_t orientation = 0;
	float yaw_offset_deg = 0.f;
	float min_distance_m = 0.2f;
	float max_distance_m = 12.f;

	param_get(param_find("RPL_BAUD"), &baudrate);
	param_get(param_find("RPL_ORIENT_CFG"), &orientation);
	param_get(param_find("RPL_YAW_OFF"), &yaw_offset_deg);
	param_get(param_find("RPL_MIN_DIST"), &min_distance_m);
	param_get(param_find("RPL_MAX_DIST"), &max_distance_m);

	_upside_down = (orientation == 1);

	// The legacy scan sample encodes at most 16.38 m.
	_min_distance_cm = (uint16_t)math::constrain((int)lroundf(min_distance_m * 100.f), 1, 1500);
	_max_distance_cm = (uint16_t)math::constrain((int)lroundf(max_distance_m * 100.f), (int)_min_distance_cm + 1, 1600);

	_obstacle_distance.min_distance = _min_distance_cm;
	_obstacle_distance.max_distance = _max_distance_cm;
	_obstacle_distance.angle_offset = ObstacleMath::wrap_360(yaw_offset_deg);

	if (baudrate <= 0 || !_uart.setBaudrate((uint32_t)baudrate)) {
		PX4_ERR("invalid baudrate %" PRId32, baudrate);
		return PX4_ERROR;
	}

	enterState(State::Stop, hrt_absolute_time());
	ScheduleNow();
	return PX4_OK;
}

void RPLidar::enterState(State state, hrt_abstime now)
{
	_state = state;
	_state_since = now;
}

const char *RPLidar::stateName(State state) const
{
	switch (state) {
	case State::Stop: return "stop";

	case State::WaitStop: return "wait stop";

	case State::WaitHealth: return "wait health";

	case State::WaitReset: return "wait reset";

	case State::WaitInfo: return "wait info";

	case State::WaitScan: return "wait scan";

	case State::Streaming: return "streaming";
	}

	return "?";
}

bool RPLidar::sendCommand(uint8_t command)
{
	const uint8_t packet[2] = {SYNC_A, command};

	if (_uart.write(packet, sizeof(packet)) != (ssize_t)sizeof(packet)) {
		perf_count(_comms_errors);
		return false;
	}

	_tx_bytes += sizeof(packet);
	return true;
}

void RPLidar::readSerial()
{
	const size_t space = sizeof(_buf) - _buf_len;

	if (space == 0) {
		// The parser always consumes or discards data, so a full buffer means garbage.
		_buf_len = 0;
		perf_count(_comms_errors);
		return;
	}

	const ssize_t ret = _uart.read(&_buf[_buf_len], space);

	if (ret > 0) {
		_rx_bytes += (uint32_t)ret;
		_rx_last_len = math::min((size_t)ret, sizeof(_rx_last));
		memcpy(_rx_last, &_buf[_buf_len], _rx_last_len);
		_buf_len += (size_t)ret;
	}
}

void RPLidar::consume(size_t count)
{
	if (count >= _buf_len) {
		_buf_len = 0;

	} else if (count > 0) {
		memmove(&_buf[0], &_buf[count], _buf_len - count);
		_buf_len -= count;
	}
}

bool RPLidar::takeResponse(uint8_t data_type, uint8_t *payload, size_t payload_len)
{
	size_t i = 0;

	while (_buf_len - i >= DESCRIPTOR_LEN) {
		Descriptor descriptor;

		if (!parse_descriptor(&_buf[i], descriptor) || descriptor.data_type != data_type) {
			i++;
			continue;
		}

		if (_buf_len - i < DESCRIPTOR_LEN + payload_len) {
			// descriptor found, payload still incoming
			break;
		}

		if (payload != nullptr && payload_len > 0) {
			memcpy(payload, &_buf[i + DESCRIPTOR_LEN], payload_len);
		}

		consume(i + DESCRIPTOR_LEN + payload_len);
		return true;
	}

	// drop everything in front of a possible descriptor start
	consume(i);
	return false;
}

void RPLidar::resetRevolution(hrt_abstime now)
{
	for (int i = 0; i < BIN_COUNT; i++) {
		_bin_min_cm[i] = UINT16_MAX;
		_bin_seen[i] = false;
	}

	_rev_nodes = 0;
	_rev_start = now;
}

void RPLidar::publishRevolution(hrt_abstime now)
{
	if (_vehicle_attitude_sub.updated()) {
		vehicle_attitude_s vehicle_attitude;

		if (_vehicle_attitude_sub.copy(&vehicle_attitude)) {
			_vehicle_attitude = matrix::Quatf(vehicle_attitude.q);
		}
	}

	for (int i = 0; i < BIN_COUNT; i++) {
		if (_bin_min_cm[i] != UINT16_MAX) {
			// Scale the range measured in the tilted body plane onto the horizontal plane.
			float distance_cm = (float)_bin_min_cm[i];
			const float yaw_frd_rad = math::radians(ObstacleMath::wrap_360(i * BIN_WIDTH_DEG + _obstacle_distance.angle_offset));
			ObstacleMath::project_distance_on_horizontal_plane(distance_cm, yaw_frd_rad, _vehicle_attitude);
			_obstacle_distance.distances[i] = (uint16_t)math::constrain(distance_cm, 0.f, (float)_max_distance_cm);

		} else if (_bin_seen[i]) {
			// The beam swept this bin without a usable return: nothing within range.
			_obstacle_distance.distances[i] = (uint16_t)(_max_distance_cm + 1);

		} else {
			// No sample at all for this direction: unknown.
			_obstacle_distance.distances[i] = UINT16_MAX;
		}
	}

	_obstacle_distance.timestamp = now;
	_obstacle_distance_pub.publish(_obstacle_distance);

	_last_rev_nodes = _rev_nodes;
	_last_rev_duration = now - _rev_start;
	perf_count(_revolution_perf);
}

void RPLidar::handleScanNode(const ScanNode &node, hrt_abstime now)
{
	// A start flag that follows the previous one too quickly comes from corrupted bytes.
	// Ignoring it keeps the revolution intact instead of publishing a half-filled one.
	if (node.start && (now - _rev_start) >= MIN_REVOLUTION_TIME) {
		if (_rev_nodes >= MIN_NODES_PER_REVOLUTION) {
			publishRevolution(now);
		}

		resetRevolution(now);
	}

	_rev_nodes++;

	float angle_deg = node.angle_deg();

	if (_upside_down) {
		// mirrored about the roll axis: clockwise becomes counter-clockwise
		angle_deg = 360.f - angle_deg;
	}

	const int bin = bin_for_angle(angle_deg, BIN_WIDTH_DEG, BIN_COUNT);

	if (!node.valid()) {
		_bin_seen[bin] = true;
		return;
	}

	const float distance_cm = node.distance_cm();

	if (distance_cm < (float)_min_distance_cm) {
		// Closer than the configured minimum (typically part of the airframe): ignore the
		// sample completely so that it neither blocks nor clears this direction.
		return;
	}

	_bin_seen[bin] = true;

	if (distance_cm > (float)_max_distance_cm) {
		return;
	}

	const uint16_t distance = (uint16_t)distance_cm;

	if (distance < _bin_min_cm[bin]) {
		_bin_min_cm[bin] = distance;
	}
}

void RPLidar::parseScanNodes(hrt_abstime now)
{
	size_t i = 0;

	while (_buf_len - i >= SCAN_NODE_LEN) {
		ScanNode node;

		if (!_stream_synced) {
			// The check bits of a single sample are weak. Only lock on once two
			// consecutive samples look valid.
			if (_buf_len - i < 2 * SCAN_NODE_LEN) {
				break;
			}

			ScanNode next;

			if (parse_scan_node(&_buf[i], node) && parse_scan_node(&_buf[i + SCAN_NODE_LEN], next)) {
				_stream_synced = true;

			} else {
				i++;
				continue;
			}

		} else if (!parse_scan_node(&_buf[i], node)) {
			_stream_synced = false;
			perf_count(_resync_perf);
			i++;
			continue;
		}

		i += SCAN_NODE_LEN;
		_last_node_time = now;
		handleScanNode(node, now);
	}

	consume(i);

	// Do not hold back data forever if the start flag is never seen.
	if (_rev_nodes >= MIN_NODES_PER_REVOLUTION && (now - _rev_start) > REVOLUTION_TIMEOUT) {
		publishRevolution(now);
		resetRevolution(now);
	}
}

void RPLidar::Run()
{
	if (!_uart.isOpen()) {
		if (!_uart.open()) {
			PX4_ERR("failed to open %s", _uart.getPort());
			ScheduleDelayed(1_s);
			return;
		}
	}

	const hrt_abstime now = hrt_absolute_time();

	readSerial();

	switch (_state) {
	case State::Stop:
		_buf_len = 0;
		sendCommand(CMD_STOP);
		enterState(State::WaitStop, now);
		break;

	case State::WaitStop:
		// discard whatever was still streaming
		_buf_len = 0;

		if (now - _state_since > STOP_SETTLE_TIME) {
			sendCommand(CMD_GET_HEALTH);
			enterState(State::WaitHealth, now);
		}

		break;

	case State::WaitHealth: {
			uint8_t payload[HEALTH_LEN];

			if (takeResponse(ANS_TYPE_HEALTH, payload, sizeof(payload))) {
				_health = payload[0];
				_health_error_code = (uint16_t)(payload[1] | (payload[2] << 8));

				if (_health == HEALTH_ERROR) {
					PX4_WARN("sensor reports error 0x%04x, resetting it", _health_error_code);
					sendCommand(CMD_RESET);
					enterState(State::WaitReset, now);

				} else {
					sendCommand(CMD_GET_INFO);
					enterState(State::WaitInfo, now);
				}
			}

			break;
		}

	case State::WaitReset:
		// the sensor prints a text banner while rebooting
		_buf_len = 0;

		if (now - _state_since > RESET_SETTLE_TIME) {
			enterState(State::Stop, now);
		}

		break;

	case State::WaitInfo: {
			uint8_t payload[INFO_LEN];

			if (takeResponse(ANS_TYPE_INFO, payload, sizeof(payload))) {
				_model = payload[0];
				_firmware_minor = payload[1];
				_firmware_major = payload[2];
				_hardware = payload[3];

				if (!_info_valid) {
					PX4_INFO("RPLidar %s (model 0x%02x) fw %u.%02u hw %u on %s", model_name(_model), _model,
						 _firmware_major, _firmware_minor, _hardware, _uart.getPort());
				}

				_info_valid = true;
				sendCommand(CMD_SCAN);
				enterState(State::WaitScan, now);
			}

			break;
		}

	case State::WaitScan:
		if (takeResponse(ANS_TYPE_SCAN, nullptr, 0)) {
			_stream_synced = false;
			_last_node_time = now;
			resetRevolution(now);
			enterState(State::Streaming, now);
			// samples may already be in the buffer
			parseScanNodes(now);
		}

		break;

	case State::Streaming:
		parseScanNodes(now);

		if (now - _last_node_time > STREAM_TIMEOUT) {
			// Usually the motor is not spinning (MOTOCTL low) or the supply collapsed.
			perf_count(_comms_errors);
			_restarts++;
			enterState(State::Stop, now);
		}

		break;
	}

	// common timeout of the request/response states
	if ((_state == State::WaitHealth || _state == State::WaitInfo || _state == State::WaitScan)
	    && (now - _state_since > RESPONSE_TIMEOUT)) {
		perf_count(_comms_errors);
		_restarts++;
		enterState(State::Stop, now);
	}

	ScheduleDelayed(INTERVAL_US);
}

void RPLidar::print_info()
{
	PX4_INFO("port: %s, baudrate: %" PRIu32, _uart.getPort(), _uart.getBaudrate());
	PX4_INFO("state: %s, restarts: %" PRIu32, stateName(_state), _restarts);
	PX4_INFO("bytes sent: %" PRIu32 ", bytes received: %" PRIu32, _tx_bytes, _rx_bytes);

	if (_rx_last_len > 0) {
		char hex[3 * sizeof(_rx_last) + 1] {};

		for (size_t k = 0; k < _rx_last_len; k++) {
			snprintf(&hex[3 * k], 4, "%02x ", _rx_last[k]);
		}

		PX4_INFO("last bytes received: %s", hex);
	}

	if (_info_valid) {
		PX4_INFO("sensor: RPLidar %s (model 0x%02x) fw %u.%02u hw %u, health %u (error 0x%04x)", model_name(_model), _model,
			 _firmware_major, _firmware_minor, _hardware, _health, _health_error_code);

	} else {
		PX4_INFO("sensor: no response yet (check wiring, baudrate and power)");
	}

	if (_state == State::Streaming && _last_rev_duration > 0) {
		PX4_INFO("scan: %.1f Hz, %u samples per revolution", (double)(1e6f / (float)_last_rev_duration), _last_rev_nodes);

	} else {
		PX4_INFO("scan: no complete revolution (is the motor spinning? MOTOCTL must be tied high)");
	}

	PX4_INFO("range: %u to %u cm, yaw offset %.1f deg, %s", _min_distance_cm, _max_distance_cm,
		 (double)_obstacle_distance.angle_offset, _upside_down ? "upside down" : "upright");

	perf_print_counter(_revolution_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_resync_perf);
}
