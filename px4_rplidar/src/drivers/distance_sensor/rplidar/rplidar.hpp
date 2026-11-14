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

/**
 * @file rplidar.hpp
 *
 * Driver for Slamtec RPLidar 360 degree laser scanners (developed on the A2M12)
 * connected directly to a flight controller UART.
 *
 * The scan is reduced to 72 bins of 5 degrees and published as obstacle_distance
 * for Collision Prevention.
 *
 * The scanner motor is NOT controlled by this driver: the MOTOCTL pin must be
 * tied high (3.3 V to 5 V) so that the motor spins whenever the unit is powered.
 */

#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix/math.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/obstacle_distance.h>
#include <uORB/topics/vehicle_attitude.h>

#include "rplidar_protocol.hpp"

using namespace time_literals;

class RPLidar : public px4::ScheduledWorkItem
{
public:
	RPLidar(const char *port);
	~RPLidar() override;

	int init();
	void print_info();

private:
	enum class State : uint8_t {
		Stop = 0,          ///< send STOP so that the sensor leaves any running scan
		WaitStop,          ///< let the sensor settle and drop stale bytes
		WaitHealth,        ///< waiting for the GET_HEALTH response
		WaitReset,         ///< sensor reported an error and was reset, wait for reboot
		WaitInfo,          ///< waiting for the GET_INFO response
		WaitScan,          ///< waiting for the SCAN response descriptor
		Streaming          ///< receiving scan samples
	};

	static constexpr int BIN_COUNT = sizeof(obstacle_distance_s::distances) / sizeof(obstacle_distance_s::distances[0]);
	static constexpr float BIN_WIDTH_DEG = 360.f / BIN_COUNT;

	static constexpr uint32_t INTERVAL_US = 5_ms;           ///< read period
	static constexpr hrt_abstime STOP_SETTLE_TIME = 100_ms;
	static constexpr hrt_abstime RESET_SETTLE_TIME = 2_s;
	static constexpr hrt_abstime RESPONSE_TIMEOUT = 1_s;
	static constexpr hrt_abstime STREAM_TIMEOUT = 1_s;       ///< restart if no valid sample for this long
	static constexpr hrt_abstime REVOLUTION_TIMEOUT = 400_ms; ///< publish even if the start flag was missed
	static constexpr hrt_abstime MIN_REVOLUTION_TIME = 40_ms; ///< a start flag earlier than this is corrupt data (15 Hz is 67 ms)
	static constexpr unsigned MIN_NODES_PER_REVOLUTION = 36;

	void Run() override;

	void enterState(State state, hrt_abstime now);
	const char *stateName(State state) const;

	bool sendCommand(uint8_t command);
	void readSerial();
	void consume(size_t count);

	/**
	 * Look for a response of the given type in the receive buffer.
	 * @param payload destination for payload_len bytes, may be nullptr if payload_len is 0
	 * @return true once the complete response was found and removed from the buffer
	 */
	bool takeResponse(uint8_t data_type, uint8_t *payload, size_t payload_len);

	void parseScanNodes(hrt_abstime now);
	void handleScanNode(const rplidar::ScanNode &node, hrt_abstime now);
	void resetRevolution(hrt_abstime now);
	void publishRevolution(hrt_abstime now);

	device::Serial _uart;

	State _state{State::Stop};
	hrt_abstime _state_since{0};

	uint8_t _buf[256] {};
	size_t _buf_len{0};

	// diagnostics shown by "rplidar status"
	uint32_t _tx_bytes{0};
	uint32_t _rx_bytes{0};
	uint8_t _rx_last[8] {};
	size_t _rx_last_len{0};
	bool _stream_synced{false};
	hrt_abstime _last_node_time{0};

	// accumulation over one revolution
	uint16_t _bin_min_cm[BIN_COUNT] {};
	bool _bin_seen[BIN_COUNT] {};
	unsigned _rev_nodes{0};
	hrt_abstime _rev_start{0};

	// configuration
	bool _upside_down{false};
	uint16_t _min_distance_cm{20};
	uint16_t _max_distance_cm{1200};

	// sensor information for status output
	uint8_t _model{0};
	uint8_t _firmware_major{0};
	uint8_t _firmware_minor{0};
	uint8_t _hardware{0};
	uint8_t _health{0};
	uint16_t _health_error_code{0};
	bool _info_valid{false};
	unsigned _last_rev_nodes{0};
	hrt_abstime _last_rev_duration{0};
	uint32_t _restarts{0};

	obstacle_distance_s _obstacle_distance{};
	uORB::Publication<obstacle_distance_s> _obstacle_distance_pub{ORB_ID(obstacle_distance)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	matrix::Quatf _vehicle_attitude{};

	perf_counter_t _revolution_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _resync_perf;
};
