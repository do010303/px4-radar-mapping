/****************************************************************************
 *
 *   Ainstein US-D1 radar altimeter as a distance sensor: command line.
 *
 ****************************************************************************/

#include "UsD1Radar.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

namespace us_d1_radar
{

static constexpr int MAX_INSTANCES = 3;
UsD1Radar *g_dev[MAX_INSTANCES] {};

static int start(const char *port, uint8_t rotation)
{
	for (auto *dev : g_dev) {
		if (dev != nullptr && strcmp(dev->port(), port) == 0) {
			PX4_WARN("%s is already in use", port);
			return -1;
		}
	}

	for (int i = 0; i < MAX_INSTANCES; i++) {
		if (g_dev[i] == nullptr) {
			g_dev[i] = new UsD1Radar(port, rotation, i);

			if (g_dev[i] == nullptr) {
				return -1;
			}

			if (g_dev[i]->init() != PX4_OK) {
				delete g_dev[i];
				g_dev[i] = nullptr;
				return -1;
			}

			return 0;
		}
	}

	PX4_ERR("at most %d sensors", MAX_INSTANCES);
	return -1;
}

static int stop()
{
	bool stopped = false;

	for (auto &dev : g_dev) {
		if (dev != nullptr) {
			delete dev;
			dev = nullptr;
			stopped = true;
		}
	}

	return stopped ? 0 : -1;
}

static int status()
{
	bool any = false;

	for (auto *dev : g_dev) {
		if (dev != nullptr) {
			dev->print_info();
			any = true;
		}
	}

	if (!any) {
		PX4_ERR("not running");
	}

	return any ? 0 : -1;
}

static int usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Serial driver for the Ainstein US-D1 radar altimeter, used as a distance sensor in any direction
(collision prevention reads every direction but up and down). Up to three sensors, one `start` each.
An altitude whose SNR is 13 dB or lower is reported as "no obstacle" (manual 2.4).

### Examples

Three sensors, 45 deg left, forward and 45 deg right:
$ us_d1_radar start -d /dev/ttyS1 -R 7
$ us_d1_radar start -d /dev/ttyS2 -R 0
$ us_d1_radar start -d /dev/ttyS3 -R 1
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("us_d1_radar", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start one sensor");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 25, "Sensor rotation (MAV_SENSOR_ROTATION): 0 forward, 1 yaw 45, 7 yaw 315", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop all sensors");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print the counters of all sensors");
	return PX4_OK;
}

} // namespace us_d1_radar

extern "C" __EXPORT int us_d1_radar_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_FORWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R':
			rotation = (uint8_t)atoi(myoptarg);
			break;

		case 'd':
			device_path = myoptarg;
			break;

		default:
			return us_d1_radar::usage();
		}
	}

	if (myoptind >= argc) {
		return us_d1_radar::usage();
	}

	if (!strcmp(argv[myoptind], "start")) {
		if (device_path == nullptr) {
			PX4_ERR("serial device required");
			return -1;
		}

		return us_d1_radar::start(device_path, rotation);

	} else if (!strcmp(argv[myoptind], "stop")) {
		return us_d1_radar::stop();

	} else if (!strcmp(argv[myoptind], "status")) {
		return us_d1_radar::status();
	}

	return us_d1_radar::usage();
}
