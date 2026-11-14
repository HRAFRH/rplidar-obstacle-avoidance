// Host test bench: runs the real rplidar.cpp against a simulated RPLidar.
// See run_host_sim.sh for how to build it.

#include <cstdio>
#include <cstdlib>
#include <deque>
#include <functional>
#include <vector>

#include "host_px4.hpp"
#include "rplidar.hpp"

// ---- globals declared in host_px4.hpp ---------------------------------------------------------
bool g_host_log_enabled = true;
hrt_abstime g_host_time_us = 0;
std::map<std::string, double> g_host_params;
HostSerialPeer *g_host_serial_peer = nullptr;
obstacle_distance_s g_host_last_obstacle_distance{};
unsigned g_host_obstacle_distance_count = 0;
vehicle_attitude_s g_host_vehicle_attitude{};
bool g_host_vehicle_attitude_updated = false;

static int failures = 0;
#define CHECK(cond) do { if (!(cond)) { printf("    FAIL line %d: %s\n", __LINE__, #cond); failures++; } } while (0)
#define CHECK_NEAR(value, expected, tolerance) do { \
		const double v_ = (double)(value); \
		if (fabs(v_ - (double)(expected)) > (double)(tolerance)) { \
			printf("    FAIL line %d: %s = %.1f, expected %.1f +/- %.1f\n", __LINE__, #value, v_, (double)(expected), (double)(tolerance)); \
			failures++; } } while (0)

// ---- simulated RPLidar ------------------------------------------------------------------------
struct FakeLidar : HostSerialPeer {
	// behaviour switches
	bool responds = true;       // false: nothing connected
	bool motor_on = true;       // false: MOTOCTL low, the sensor answers but never produces samples
	uint8_t health = 0;         // 2: error until RESET
	size_t max_chunk = 256;     // bytes handed to the driver per read
	double samples_per_sec = 4000.0;
	double rev_hz = 10.0;
	std::function<float(float)> scene;  // angle in degrees -> distance in mm, 0 = no return

	// state
	bool streaming = false;
	double angle = 123.0;       // start mid revolution like a real unit
	double sample_credit = 0.0;
	hrt_abstime last_time = 0;
	bool expect_command = false;
	std::deque<uint8_t> out;
	std::vector<uint8_t> commands;
	unsigned stops = 0, resets = 0, scans = 0;

	void push(std::initializer_list<uint8_t> bytes) { for (uint8_t b : bytes) { out.push_back(b); } }

	void handle(uint8_t command)
	{
		commands.push_back(command);

		if (!responds) { return; }

		switch (command) {
		case 0x25: // STOP
			stops++;
			streaming = false;
			break;

		case 0x40: // RESET
			resets++;
			streaming = false;
			health = 0;

			for (const char *c = "RP LIDAR System.\r\nFirmware Ver 1.32\r\n"; *c; c++) { out.push_back((uint8_t)*c); }

			break;

		case 0x52: // GET_HEALTH
			push({0xA5, 0x5A, 0x03, 0x00, 0x00, 0x00, 0x06, health, 0x34, 0x12});
			break;

		case 0x50: // GET_INFO
			push({0xA5, 0x5A, 0x14, 0x00, 0x00, 0x00, 0x04, 0x2C, 0x20, 0x01, 0x06});

			for (int i = 0; i < 16; i++) { out.push_back((uint8_t)(0xA0 + i)); }

			break;

		case 0x20: // SCAN
			scans++;

			if (health == 2) { break; } // protection stop: no scan

			push({0xA5, 0x5A, 0x05, 0x00, 0x00, 0x40, 0x81});
			streaming = true;
			sample_credit = 0.0;
			last_time = g_host_time_us;
			break;

		default:
			break;
		}
	}

	void from_host(const uint8_t *data, size_t len) override
	{
		for (size_t i = 0; i < len; i++) {
			if (expect_command) {
				expect_command = false;
				handle(data[i]);

			} else if (data[i] == 0xA5) {
				expect_command = true;
			}
		}
	}

	size_t to_host(uint8_t *data, size_t max_len) override
	{
		size_t n = 0;

		while (n < max_len && n < max_chunk && !out.empty()) {
			data[n++] = out.front();
			out.pop_front();
		}

		return n;
	}

	void emit_sample()
	{
		bool start = false;
		angle += 360.0 * rev_hz / samples_per_sec;

		if (angle >= 360.0) {
			angle -= 360.0;
			start = true;
		}

		const float distance_mm = scene ? scene((float)angle) : 0.f;
		const uint8_t quality = distance_mm > 0.f ? 47 : 0;
		const uint16_t angle_q6 = (uint16_t)(angle * 64.0);
		const uint16_t distance_q2 = (uint16_t)(distance_mm * 4.f);
		push({(uint8_t)((quality << 2) | (start ? 0x01 : 0x02)),
		      (uint8_t)(((angle_q6 & 0x7F) << 1) | 0x01),
		      (uint8_t)(angle_q6 >> 7),
		      (uint8_t)(distance_q2 & 0xFF),
		      (uint8_t)(distance_q2 >> 8)});
	}

	void advance(hrt_abstime now)
	{
		if (streaming && motor_on) {
			sample_credit += (double)(now - last_time) * 1e-6 * samples_per_sec;

			while (sample_credit >= 1.0) {
				sample_credit -= 1.0;
				emit_sample();
			}
		}

		last_time = now;
	}

	void inject_garbage(unsigned count)
	{
		static uint32_t seed = 12345;

		for (unsigned i = 0; i < count; i++) {
			seed = seed * 1664525u + 1013904223u;
			out.push_back((uint8_t)(seed >> 24));
		}
	}
};

// A wall 2 m in front (seen within +/-20 deg), an object 0.5 m to the right,
// a part of the airframe 0.1 m behind, and open space everywhere else.
static float default_scene(float angle_deg)
{
	if (angle_deg <= 20.f || angle_deg >= 340.f) {
		return 2000.f / cosf(angle_deg * 3.14159265f / 180.f);
	}

	if (angle_deg >= 85.f && angle_deg <= 95.f) {
		return 500.f;
	}

	if (angle_deg >= 175.f && angle_deg <= 185.f) {
		return 100.f;
	}

	return 0.f;
}

static constexpr uint16_t FREE = 1201;      // max_distance + 1
static constexpr uint16_t UNKNOWN = UINT16_MAX;

struct Bench {
	FakeLidar lidar;
	RPLidar *dev{nullptr};

	Bench()
	{
		lidar.scene = default_scene;
		g_host_serial_peer = &lidar;
		g_host_params = {{"RPL_BAUD", 256000}, {"RPL_ORIENT_CFG", 0}, {"RPL_YAW_OFF", 0}, {"RPL_MIN_DIST", 0.2}, {"RPL_MAX_DIST", 12}};
		g_host_obstacle_distance_count = 0;
		g_host_last_obstacle_distance = {};
		g_host_vehicle_attitude = {};
		g_host_vehicle_attitude.q[0] = 1.f;
		g_host_vehicle_attitude_updated = false;
		g_host_time_us += 1000000;
	}

	~Bench()
	{
		delete dev;
		g_host_serial_peer = nullptr;
	}

	void start()
	{
		dev = new RPLidar("/dev/ttyS4");
		CHECK(dev->init() == PX4_OK);
	}

	void run_ms(unsigned ms)
	{
		for (unsigned t = 0; t < ms; t += 5) {
			g_host_time_us += 5000;
			lidar.advance(g_host_time_us);
			dev->host_run();
		}
	}

	const uint16_t *d() const { return g_host_last_obstacle_distance.distances; }

	unsigned count_bins(uint16_t value) const
	{
		unsigned n = 0;

		for (int i = 0; i < 72; i++) { if (d()[i] == value) { n++; } }

		return n;
	}

	void check_default_map()
	{
		CHECK_NEAR(d()[0], 200, 2);     // wall straight ahead
		CHECK_NEAR(d()[3], 204, 3);     // 15 deg right of the nose, wall is further along the ray
		CHECK_NEAR(d()[69], 204, 3);    // 15 deg left
		CHECK_NEAR(d()[18], 50, 1);     // object to the right
		CHECK(d()[27] == FREE);         // swept, nothing there
		CHECK(d()[35] == FREE);         // partly airframe, partly open
		CHECK(d()[36] == UNKNOWN);      // only airframe returns below RPL_MIN_DIST: masked
		CHECK(d()[54] == FREE);         // left
		CHECK(count_bins(UNKNOWN) == 1);
	}
};

static void scenario(const char *name) { printf("\n== %s\n", name); }

int main()
{
	scenario("1. normal start-up and scan");
	{
		Bench b;
		b.start();
		b.run_ms(1500);
		const obstacle_distance_s &m = g_host_last_obstacle_distance;
		printf("    published %u messages in 1.5 s\n", g_host_obstacle_distance_count);
		CHECK(b.lidar.commands.size() >= 4);

		if (b.lidar.commands.size() >= 4) {
			CHECK(b.lidar.commands[0] == 0x25 && b.lidar.commands[1] == 0x52);
			CHECK(b.lidar.commands[2] == 0x50 && b.lidar.commands[3] == 0x20);
		}

		CHECK(g_host_obstacle_distance_count >= 10 && g_host_obstacle_distance_count <= 14);
		CHECK(m.frame == obstacle_distance_s::MAV_FRAME_BODY_FRD);
		CHECK_NEAR(m.increment, 5, 0.001);
		CHECK_NEAR(m.angle_offset, 0, 0.001);
		CHECK(m.min_distance == 20 && m.max_distance == 1200);
		CHECK(m.timestamp > 0);
		b.check_default_map();
		b.dev->print_info();
	}

	scenario("2. motor not spinning (MOTOCTL low), then fixed");
	{
		Bench b;
		b.lidar.motor_on = false;
		b.start();
		g_host_log_enabled = false;
		b.run_ms(3500);
		g_host_log_enabled = true;
		printf("    while stopped: %u messages, %u scan attempts\n", g_host_obstacle_distance_count, b.lidar.scans);
		CHECK(g_host_obstacle_distance_count == 0);
		CHECK(b.lidar.scans >= 3);
		b.dev->print_info();
		b.lidar.motor_on = true;
		b.run_ms(2500);
		printf("    after the motor starts: %u messages\n", g_host_obstacle_distance_count);
		CHECK(g_host_obstacle_distance_count >= 10);
		b.check_default_map();
	}

	scenario("3. corrupted bytes in the stream");
	{
		Bench b;
		b.start();
		b.run_ms(1000);
		const unsigned before = g_host_obstacle_distance_count;

		for (int i = 0; i < 10; i++) {
			b.lidar.inject_garbage(37);
			b.run_ms(100);
		}

		printf("    %u messages during 1 s with 10 bursts of garbage\n", g_host_obstacle_distance_count - before);
		CHECK(g_host_obstacle_distance_count - before >= 7);
		b.run_ms(500);
		b.check_default_map();
		b.dev->print_info();
	}

	scenario("4. sensor reports a health error");
	{
		Bench b;
		b.lidar.health = 2;
		b.start();
		b.run_ms(5000);
		printf("    resets sent: %u, messages: %u\n", b.lidar.resets, g_host_obstacle_distance_count);
		CHECK(b.lidar.resets == 1);
		CHECK(g_host_obstacle_distance_count >= 10);
		b.check_default_map();
	}

	scenario("5. mounted upside down");
	{
		Bench b;
		g_host_params["RPL_ORIENT_CFG"] = 1;
		b.start();
		b.run_ms(1500);
		CHECK_NEAR(b.d()[0], 200, 2);
		CHECK_NEAR(b.d()[54], 50, 1);   // the object now appears on the left
		CHECK(b.d()[18] == FREE);
	}

	scenario("6. yaw offset 90 deg");
	{
		Bench b;
		g_host_params["RPL_YAW_OFF"] = 90;
		b.start();
		b.run_ms(1500);
		CHECK_NEAR(g_host_last_obstacle_distance.angle_offset, 90, 0.001);
		CHECK_NEAR(b.d()[0], 200, 2);   // bins stay in the sensor frame; the offset is applied by the consumer
		CHECK_NEAR(b.d()[18], 50, 1);
	}

	scenario("7. nothing connected");
	{
		Bench b;
		b.lidar.responds = false;
		b.start();
		g_host_log_enabled = false;
		b.run_ms(4000);
		g_host_log_enabled = true;
		printf("    messages: %u, commands sent: %u\n", g_host_obstacle_distance_count, (unsigned)b.lidar.commands.size());
		CHECK(g_host_obstacle_distance_count == 0);
		CHECK(b.lidar.commands.size() >= 6);
		b.dev->print_info();
	}

	scenario("8. samples split across reads");
	{
		Bench b;
		b.lidar.max_chunk = 103;
		b.start();
		b.run_ms(2000);
		CHECK(g_host_obstacle_distance_count >= 14);
		b.check_default_map();
	}

	scenario("9. vehicle pitched 30 deg");
	{
		Bench b;
		g_host_vehicle_attitude.q[0] = cosf(15.f * 3.14159265f / 180.f);
		g_host_vehicle_attitude.q[2] = sinf(15.f * 3.14159265f / 180.f);
		g_host_vehicle_attitude_updated = true;
		b.start();
		b.run_ms(1500);
		printf("    front %u cm (200 cm along the tilted beam), right %u cm\n", b.d()[0], b.d()[18]);
		CHECK_NEAR(b.d()[0], 173, 3);   // 200 * cos(30 deg)
		CHECK_NEAR(b.d()[18], 50, 1);   // along the pitch axis: unchanged
	}

	scenario("10. slower, denser scan (5 Hz, 8000 samples/s)");
	{
		Bench b;
		b.lidar.rev_hz = 5.0;
		b.lidar.samples_per_sec = 8000.0;
		b.lidar.max_chunk = 256;
		b.start();
		b.run_ms(2000);
		printf("    messages: %u\n", g_host_obstacle_distance_count);
		CHECK(g_host_obstacle_distance_count >= 7 && g_host_obstacle_distance_count <= 10);
		b.check_default_map();
	}

	printf("\n%s\n", failures == 0 ? "ALL SCENARIOS PASSED" : "SOME CHECKS FAILED");
	return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
