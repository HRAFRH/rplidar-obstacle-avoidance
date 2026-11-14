// Minimal stand-ins for the PX4 APIs used by the RPLidar driver, so that the real
// rplidar.cpp can be compiled and run on a PC against a simulated sensor.
// Test code only. Nothing here is part of the firmware.

#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <map>
#include <string>

// ---- defines --------------------------------------------------------------------------------
#define PX4_OK 0
#define PX4_ERROR (-1)
#define __EXPORT
#define MODULE_NAME "rplidar"

#ifndef M_PI_F
#define M_PI_F 3.14159265f
#endif
#ifndef M_TWOPI_F
#define M_TWOPI_F (M_PI_F * 2.0f)
#endif
#ifndef M_PI_2_F
#define M_PI_2_F (M_PI_F / 2.0f)
#endif
#define PX4_ISFINITE(x) std::isfinite(x)

// ---- logging --------------------------------------------------------------------------------
extern bool g_host_log_enabled;
#define HOST_LOG(level, ...) do { if (g_host_log_enabled) { printf("    [" level "] " __VA_ARGS__); printf("\n"); } } while (0)
#define PX4_INFO(...) HOST_LOG("info", __VA_ARGS__)
#define PX4_WARN(...) HOST_LOG("warn", __VA_ARGS__)
#define PX4_ERR(...) HOST_LOG("err ", __VA_ARGS__)
#define PX4_DEBUG(...) do { } while (0)

// ---- time -----------------------------------------------------------------------------------
typedef uint64_t hrt_abstime;
extern hrt_abstime g_host_time_us;
static inline hrt_abstime hrt_absolute_time() { return g_host_time_us; }

namespace time_literals
{
constexpr hrt_abstime operator ""_s(unsigned long long seconds) { return hrt_abstime(seconds * 1000000ULL); }
constexpr hrt_abstime operator ""_ms(unsigned long long milliseconds) { return hrt_abstime(milliseconds * 1000ULL); }
constexpr hrt_abstime operator ""_us(unsigned long long microseconds) { return hrt_abstime(microseconds); }
}

// ---- parameters -----------------------------------------------------------------------------
typedef const char *param_t;
extern std::map<std::string, double> g_host_params;
static inline param_t param_find(const char *name) { return name; }
static inline int param_get(param_t param, int32_t *value)
{
	auto it = g_host_params.find(param);

	if (it == g_host_params.end()) { return -1; }

	*value = (int32_t)it->second;
	return 0;
}
static inline int param_get(param_t param, float *value)
{
	auto it = g_host_params.find(param);

	if (it == g_host_params.end()) { return -1; }

	*value = (float)it->second;
	return 0;
}

// ---- perf counters --------------------------------------------------------------------------
enum perf_counter_type { PC_COUNT, PC_ELAPSED, PC_INTERVAL };
struct host_perf_counter { const char *name; uint64_t count; };
typedef host_perf_counter *perf_counter_t;
static inline perf_counter_t perf_alloc(perf_counter_type, const char *name) { return new host_perf_counter{name, 0}; }
static inline void perf_free(perf_counter_t counter) { delete counter; }
static inline void perf_count(perf_counter_t counter) { counter->count++; }
static inline void perf_print_counter(perf_counter_t counter)
{
	PX4_INFO("%s: %llu events", counter->name, (unsigned long long)counter->count);
}

// ---- work queue -----------------------------------------------------------------------------
namespace px4
{
struct wq_config_t { const char *name; };
static inline const wq_config_t &serial_port_to_wq(const char *)
{
	static wq_config_t config{"host"};
	return config;
}

class ScheduledWorkItem
{
public:
	ScheduledWorkItem(const char *, const wq_config_t &) {}
	virtual ~ScheduledWorkItem() = default;

	void ScheduleNow() { scheduled = true; delay_us = 0; }
	void ScheduleDelayed(uint32_t delay) { scheduled = true; delay_us = delay; }
	void ScheduleClear() { scheduled = false; }

	// test hook: execute one work item cycle
	void host_run() { Run(); }

	bool scheduled{false};
	uint32_t delay_us{0};

protected:
	virtual void Run() = 0;
};
}

// ---- serial port ----------------------------------------------------------------------------
// Whatever is "connected" to the UART. The simulated lidar implements this.
struct HostSerialPeer {
	virtual ~HostSerialPeer() = default;
	virtual void from_host(const uint8_t *data, size_t len) = 0;
	virtual size_t to_host(uint8_t *data, size_t max_len) = 0;
};
extern HostSerialPeer *g_host_serial_peer;

namespace device
{
class Serial
{
public:
	Serial(const char *port, uint32_t baudrate = 57600) : _baudrate(baudrate)
	{
		strncpy(_port, port, sizeof(_port) - 1);
	}

	bool open() { _open = true; return true; }
	bool isOpen() const { return _open; }
	bool close() { _open = false; return true; }
	void flush() {}

	ssize_t read(uint8_t *buffer, size_t buffer_size)
	{
		return (_open && g_host_serial_peer) ? (ssize_t)g_host_serial_peer->to_host(buffer, buffer_size) : -1;
	}

	ssize_t write(const void *buffer, size_t buffer_size)
	{
		if (!_open || !g_host_serial_peer) { return -1; }

		g_host_serial_peer->from_host((const uint8_t *)buffer, buffer_size);
		return (ssize_t)buffer_size;
	}

	uint32_t getBaudrate() const { return _baudrate; }
	bool setBaudrate(uint32_t baudrate) { _baudrate = baudrate; return true; }
	const char *getPort() const { return _port; }

private:
	char _port[32] {};
	uint32_t _baudrate;
	bool _open{false};
};
}

// ---- uORB -----------------------------------------------------------------------------------
struct obstacle_distance_s {
	uint64_t timestamp;
	float increment;
	float angle_offset;
	uint16_t distances[72];
	uint16_t min_distance;
	uint16_t max_distance;
	uint8_t frame;
	uint8_t sensor_type;

	static constexpr uint8_t MAV_FRAME_GLOBAL = 0;
	static constexpr uint8_t MAV_FRAME_LOCAL_NED = 1;
	static constexpr uint8_t MAV_FRAME_BODY_FRD = 12;
	static constexpr uint8_t MAV_DISTANCE_SENSOR_LASER = 0;
};

struct vehicle_attitude_s {
	uint64_t timestamp;
	uint64_t timestamp_sample;
	float q[4];
	float delta_q_reset[4];
	uint8_t quat_reset_counter;
};

extern obstacle_distance_s g_host_last_obstacle_distance;
extern unsigned g_host_obstacle_distance_count;
extern vehicle_attitude_s g_host_vehicle_attitude;
extern bool g_host_vehicle_attitude_updated;

#define ORB_ID(name) #name

namespace uORB
{
template<typename T>
class Publication
{
public:
	Publication(const char *) {}
	bool publish(const T &data);
};

template<>
inline bool Publication<obstacle_distance_s>::publish(const obstacle_distance_s &data)
{
	g_host_last_obstacle_distance = data;
	g_host_obstacle_distance_count++;
	return true;
}

class Subscription
{
public:
	Subscription(const char *) {}
	bool updated() { return g_host_vehicle_attitude_updated; }
	bool copy(void *dst)
	{
		memcpy(dst, &g_host_vehicle_attitude, sizeof(g_host_vehicle_attitude));
		g_host_vehicle_attitude_updated = false;
		return true;
	}
};
}
