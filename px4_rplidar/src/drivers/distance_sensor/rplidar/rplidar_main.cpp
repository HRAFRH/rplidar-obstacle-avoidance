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

#include <string.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

namespace rplidar_driver
{

RPLidar *g_dev{nullptr};

static int start(const char *port)
{
	if (g_dev != nullptr) {
		PX4_WARN("already started");
		return -1;
	}

	if (port == nullptr) {
		PX4_ERR("no device specified");
		return -1;
	}

	g_dev = new RPLidar(port);

	if (g_dev == nullptr) {
		PX4_ERR("alloc failed");
		return -1;
	}

	if (g_dev->init() != PX4_OK) {
		delete g_dev;
		g_dev = nullptr;
		return -1;
	}

	return 0;
}

static int stop()
{
	if (g_dev == nullptr) {
		return -1;
	}

	delete g_dev;
	g_dev = nullptr;
	return 0;
}

static int status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return -1;
	}

	g_dev->print_info();
	return 0;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for Slamtec RPLidar 360 degree laser scanners (developed on the A2M12).

The scan is published as `obstacle_distance` (72 bins of 5 degrees) for Collision Prevention.

The scanner motor is not controlled by the driver: the MOTOCTL pin of the sensor
must be tied high (3.3 V to 5 V).

Most boards do not include this driver by default. It is enabled in the board
configuration with DRIVERS_DISTANCE_SENSOR_RPLIDAR and started through SENS_RPLIDAR_CFG.

### Examples

Start the driver on a specified serial device
$ rplidar start -d /dev/ttyS4
Show the sensor and scan status
$ rplidar status
Stop the driver
$ rplidar stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("rplidar", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	return PX4_OK;
}

} // namespace rplidar_driver

extern "C" __EXPORT int rplidar_main(int argc, char *argv[])
{
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_path = myoptarg;
			break;

		default:
			rplidar_driver::usage();
			return -1;
		}
	}

	if (myoptind >= argc) {
		rplidar_driver::usage();
		return -1;
	}

	if (!strcmp(argv[myoptind], "start")) {
		return rplidar_driver::start(device_path);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return rplidar_driver::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return rplidar_driver::status();
	}

	rplidar_driver::usage();
	return -1;
}
