// Host test for rplidar_protocol.hpp. No PX4 needed.
//
// Build and run:
//   g++ -std=c++14 -Wall -Wextra -I../src/drivers/distance_sensor/rplidar test_protocol.cpp -o test_protocol && ./test_protocol

#include <cstdio>
#include <cstdlib>
#include <vector>

#include "rplidar_protocol.hpp"

using namespace rplidar;

static int failures = 0;

#define CHECK(cond) do { if (!(cond)) { printf("FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)

// Encode a legacy scan sample exactly as the sensor does.
static void encode_node(std::vector<uint8_t> &out, bool start, uint8_t quality, float angle_deg, float distance_mm)
{
	const uint16_t angle_q6 = (uint16_t)(angle_deg * 64.f);
	const uint16_t distance_q2 = (uint16_t)(distance_mm * 4.f);
	out.push_back((uint8_t)((quality << 2) | (start ? 0x01 : 0x02)));
	out.push_back((uint8_t)(((angle_q6 & 0x7F) << 1) | 0x01));
	out.push_back((uint8_t)(angle_q6 >> 7));
	out.push_back((uint8_t)(distance_q2 & 0xFF));
	out.push_back((uint8_t)(distance_q2 >> 8));
}

int main()
{
	// --- response descriptors (byte values from the Slamtec protocol specification) ---
	{
		const uint8_t info[7] = {0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04};
		Descriptor d{};
		CHECK(parse_descriptor(info, d));
		CHECK(d.length == INFO_LEN);
		CHECK(d.send_mode == 0);
		CHECK(d.data_type == ANS_TYPE_INFO);

		const uint8_t health[7] = {0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06};
		CHECK(parse_descriptor(health, d));
		CHECK(d.length == HEALTH_LEN);
		CHECK(d.data_type == ANS_TYPE_HEALTH);

		const uint8_t scan[7] = {0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81};
		CHECK(parse_descriptor(scan, d));
		CHECK(d.length == SCAN_NODE_LEN);
		CHECK(d.send_mode == 1);
		CHECK(d.data_type == ANS_TYPE_SCAN);

		const uint8_t bad[7] = {0xA5, 0x00, 0x05, 0x00, 0x00, 0x40, 0x81};
		CHECK(!parse_descriptor(bad, d));
	}

	// --- scan samples ---
	{
		std::vector<uint8_t> b;
		encode_node(b, true, 47, 0.f, 1234.f);
		encode_node(b, false, 20, 123.5f, 250.f);
		encode_node(b, false, 0, 359.9f, 0.f);

		ScanNode n{};
		CHECK(parse_scan_node(&b[0], n));
		CHECK(n.start);
		CHECK(n.quality == 47);
		CHECK(n.angle_q6 == 0);
		CHECK(fabsf(n.distance_cm() - 123.4f) < 0.05f);
		CHECK(n.valid());

		CHECK(parse_scan_node(&b[5], n));
		CHECK(!n.start);
		CHECK(fabsf(n.angle_deg() - 123.5f) < 0.02f);
		CHECK(fabsf(n.distance_cm() - 25.f) < 0.05f);

		CHECK(parse_scan_node(&b[10], n));
		CHECK(!n.valid());

		// a sample shifted by one byte must not be accepted together with its successor
		int accepted_pairs = 0;

		for (int shift = 1; shift < 5; shift++) {
			ScanNode a{}, c{};

			if (parse_scan_node(&b[shift], a) && parse_scan_node(&b[shift + 5], c)) {
				accepted_pairs++;
			}
		}

		CHECK(accepted_pairs == 0);

		// both check bit violations
		uint8_t s_equal[5] = {0x03, 0x01, 0x00, 0x00, 0x00};
		CHECK(!parse_scan_node(s_equal, n));
		uint8_t no_c[5] = {0x01, 0x00, 0x00, 0x00, 0x00};
		CHECK(!parse_scan_node(no_c, n));
		// angle far beyond one turn
		uint8_t big_angle[5] = {0x01, 0xFF, 0xFF, 0x00, 0x00};
		CHECK(!parse_scan_node(big_angle, n));
	}

	// --- bins: 72 bins of 5 degrees, bin 0 centred on 0 ---
	{
		CHECK(bin_for_angle(0.f, 5.f, 72) == 0);
		CHECK(bin_for_angle(2.4f, 5.f, 72) == 0);
		CHECK(bin_for_angle(2.6f, 5.f, 72) == 1);
		CHECK(bin_for_angle(90.f, 5.f, 72) == 18);
		CHECK(bin_for_angle(180.f, 5.f, 72) == 36);
		CHECK(bin_for_angle(357.4f, 5.f, 72) == 71);
		CHECK(bin_for_angle(357.6f, 5.f, 72) == 0);
		CHECK(bin_for_angle(360.f, 5.f, 72) == 0);
		CHECK(bin_for_angle(365.f, 5.f, 72) == 1);
		CHECK(bin_for_angle(-5.f, 5.f, 72) == 71);
	}

	if (failures == 0) {
		printf("all protocol tests passed\n");
		return EXIT_SUCCESS;
	}

	printf("%d check(s) failed\n", failures);
	return EXIT_FAILURE;
}
