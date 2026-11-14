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
 * @file rplidar_protocol.hpp
 *
 * Slamtec RPLidar serial protocol: constants and stateless decoding helpers.
 *
 * This header has no PX4 dependencies so that it can be unit tested on a host.
 *
 * Only the legacy (standard) scan mode is used. It is supported by every
 * RPLidar model and its 5 byte samples fit comfortably in the UART budget.
 */

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>

namespace rplidar
{

// Every request starts with SYNC_A. Every response descriptor starts with SYNC_A, SYNC_B.
static constexpr uint8_t SYNC_A = 0xA5;
static constexpr uint8_t SYNC_B = 0x5A;

// Requests without payload: {SYNC_A, command}
static constexpr uint8_t CMD_STOP       = 0x25;
static constexpr uint8_t CMD_RESET      = 0x40;
static constexpr uint8_t CMD_SCAN       = 0x20;
static constexpr uint8_t CMD_GET_INFO   = 0x50;
static constexpr uint8_t CMD_GET_HEALTH = 0x52;

// Response data types (last byte of the descriptor)
static constexpr uint8_t ANS_TYPE_INFO   = 0x04;
static constexpr uint8_t ANS_TYPE_HEALTH = 0x06;
static constexpr uint8_t ANS_TYPE_SCAN   = 0x81;

static constexpr size_t DESCRIPTOR_LEN = 7;
static constexpr size_t INFO_LEN       = 20;
static constexpr size_t HEALTH_LEN     = 3;
static constexpr size_t SCAN_NODE_LEN  = 5;

static constexpr uint8_t HEALTH_GOOD    = 0;
static constexpr uint8_t HEALTH_WARNING = 1;
static constexpr uint8_t HEALTH_ERROR   = 2;

// Angles are Q6 fixed point degrees. Allow a little more than 360 because the
// sensor's angle compensation can report values slightly past a full turn.
static constexpr uint16_t ANGLE_Q6_MAX = (360 + 10) * 64;

struct Descriptor {
	uint32_t length;    ///< length of one data response in bytes
	uint8_t send_mode;  ///< 0: single response, 1: multiple responses
	uint8_t data_type;  ///< one of ANS_TYPE_*
};

/**
 * Decode a 7 byte response descriptor.
 * @return false if the two sync bytes are not present
 */
inline bool parse_descriptor(const uint8_t *b, Descriptor &d)
{
	if (b[0] != SYNC_A || b[1] != SYNC_B) {
		return false;
	}

	const uint32_t v = (uint32_t)b[2] | ((uint32_t)b[3] << 8) | ((uint32_t)b[4] << 16) | ((uint32_t)b[5] << 24);
	d.length = v & 0x3FFFFFFFu;
	d.send_mode = (uint8_t)(v >> 30);
	d.data_type = b[6];
	return true;
}

struct ScanNode {
	bool start;            ///< true on the first sample of a new revolution
	uint8_t quality;       ///< 0 means the measurement is invalid
	uint16_t angle_q6;     ///< degrees * 64, clockwise seen from above
	uint16_t distance_q2;  ///< millimetres * 4, 0 means no valid return

	float angle_deg() const { return angle_q6 / 64.f; }
	float distance_cm() const { return distance_q2 / 40.f; }
	bool valid() const { return quality > 0 && distance_q2 > 0; }
};

/**
 * Decode one 5 byte sample of the legacy scan mode.
 *
 *   byte 0: quality[7:2] | !S[1] | S[0]
 *   byte 1: angle_q6[6:0] << 1 | C[0]   (C is always 1)
 *   byte 2: angle_q6[14:7]
 *   byte 3: distance_q2[7:0]
 *   byte 4: distance_q2[15:8]
 *
 * @return false if the check bits or the angle are implausible (stream out of sync)
 */
inline bool parse_scan_node(const uint8_t *b, ScanNode &n)
{
	const bool s = (b[0] & 0x01) != 0;
	const bool not_s = (b[0] & 0x02) != 0;

	if (s == not_s) {
		return false;
	}

	if ((b[1] & 0x01) == 0) {
		return false;
	}

	const uint16_t angle_q6 = (uint16_t)(((uint16_t)b[2] << 7) | (b[1] >> 1));

	if (angle_q6 > ANGLE_Q6_MAX) {
		return false;
	}

	n.start = s;
	n.quality = b[0] >> 2;
	n.angle_q6 = angle_q6;
	n.distance_q2 = (uint16_t)((uint16_t)b[3] | ((uint16_t)b[4] << 8));
	return true;
}

/**
 * Index of the bin whose centre is closest to the given angle.
 * Bin 0 is centred on 0 degrees, matching ObstacleMath::get_bin_at_angle().
 */
inline int bin_for_angle(float angle_deg, float bin_width_deg, int bin_count)
{
	int bin = (int)lroundf(angle_deg / bin_width_deg) % bin_count;

	if (bin < 0) {
		bin += bin_count;
	}

	return bin;
}

/**
 * Human readable model name from the model byte of the device info response.
 * The high nibble is the model family; the mapping below is best effort.
 */
inline const char *model_name(uint8_t model)
{
	switch (model) {
	case 0x18: return "A1";

	case 0x28: return "A2";

	case 0x2C: return "A2M12";

	case 0x31: return "A3";

	case 0x41: return "C1";

	case 0x61: return "S1";

	case 0x71: return "S2";

	default: return "unknown";
	}
}

} // namespace rplidar
