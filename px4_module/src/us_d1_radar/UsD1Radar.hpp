/****************************************************************************
 *
 *   Ainstein US-D1 radar altimeter as a distance sensor.
 *   Packet format and rules: US-D1 user manual D00.02.05 (2023-04-03), 5.1 and 2.4.
 *
 ****************************************************************************/

#pragma once

#include <drivers/drv_hrt.h>
#include <lib/drivers/rangefinder/PX4Rangefinder.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <termios.h>

using namespace time_literals;

class UsD1Radar : public px4::ScheduledWorkItem
{
public:
	// Manual, Table 1: 0.5 to 50 m, field of view 43 x 30 deg.
	static constexpr float MIN_DISTANCE_M = 0.5f;
	static constexpr float MAX_DISTANCE_M = 50.0f;
	static constexpr float HFOV_DEG = 43.0f;
	static constexpr float VFOV_DEG = 30.0f;

	// Manual 2.4: an altitude with an SNR of 13 dB or lower is erroneous.
	static constexpr uint8_t SNR_MIN_DB = 13;

	/**
	 * @param port serial device to read the sensor from
	 * @param rotation sensor rotation relative to the vehicle body (distance_sensor_s::ROTATION_*)
	 * @param instance 0..2, tells the sensors apart in the device id
	 */
	UsD1Radar(const char *port, uint8_t rotation, uint8_t instance);
	~UsD1Radar() override;

	int init();
	void print_info();

	const char *port() const { return _port; }
	uint8_t rotation() const { return _rotation; }

private:
	void Run() override;
	int open_serial_port();

	/** Reads what has arrived and returns true with the newest valid packet in cm and dB. */
	bool read_packet(int &distance_cm, uint8_t &snr);

	PX4Rangefinder _px4_rangefinder;

	char _port[64] {};
	uint8_t _rotation;
	int _fd{-1};

	// Bytes received and not yet used: a packet can arrive in two reads.
	uint8_t _rx[96] {};
	unsigned _rx_len{0};

	uint32_t _valid{0};
	uint32_t _low_snr{0};

	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
};
