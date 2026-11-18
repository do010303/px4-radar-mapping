/****************************************************************************
 *
 *   Ainstein US-D1 radar altimeter as a distance sensor.
 *
 ****************************************************************************/

#include "UsD1Radar.hpp"

#include <fcntl.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/log.h>
#include <string.h>
#include <unistd.h>

// Manual 5.1: 0xFE, version 0x02, altitude LSB, altitude MSB (cm), SNR, checksum.
static constexpr uint8_t PACKET_HEADER = 0xFE;
static constexpr uint8_t PACKET_VERSION = 0x02;
static constexpr unsigned PACKET_LENGTH = 6;

// The sensor sends at 100 Hz; poll faster than that so no packet is left waiting.
static constexpr hrt_abstime POLL_INTERVAL = 5_ms;

UsD1Radar::UsD1Radar(const char *port, uint8_t rotation, uint8_t instance) :
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_px4_rangefinder(0, rotation),
	_rotation(rotation)
{
	strncpy(_port, port, sizeof(_port) - 1);

	device::Device::DeviceId device_id{};
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;
	device_id.devid_s.bus = instance;
	device_id.devid_s.address = instance + 1;
	device_id.devid_s.devtype = DRV_DIST_DEVTYPE_ULANDING;

	_px4_rangefinder.set_device_id(device_id.devid);
	_px4_rangefinder.set_device_type(DRV_DIST_DEVTYPE_ULANDING);
	_px4_rangefinder.set_rangefinder_type(distance_sensor_s::MAV_DISTANCE_SENSOR_RADAR);
	_px4_rangefinder.set_min_distance(MIN_DISTANCE_M);
	_px4_rangefinder.set_max_distance(MAX_DISTANCE_M);
	_px4_rangefinder.set_hfov(math::radians(HFOV_DEG));
	_px4_rangefinder.set_vfov(math::radians(VFOV_DEG));
}

UsD1Radar::~UsD1Radar()
{
	ScheduleClear();

	if (_fd >= 0) {
		::close(_fd);
	}

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int UsD1Radar::init()
{
	ScheduleOnInterval(POLL_INTERVAL);
	return PX4_OK;
}

int UsD1Radar::open_serial_port()
{
	if (_fd >= 0) {
		return PX4_OK;
	}

	_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("open %s failed (%i)", _port, errno);
		return PX4_ERROR;
	}

	termios uart_config{};
	tcgetattr(_fd, &uart_config);

	// Raw 115200 8N1 (manual 5.1).
	cfmakeraw(&uart_config);
	uart_config.c_cflag &= ~(CSTOPB | PARENB);
	cfsetispeed(&uart_config, B115200);
	cfsetospeed(&uart_config, B115200);

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("configuring %s failed (%i)", _port, errno);
		::close(_fd);
		_fd = -1;
		return PX4_ERROR;
	}

	PX4_INFO("opened %s, rotation %u", _port, _rotation);
	return PX4_OK;
}

bool UsD1Radar::read_packet(int &distance_cm, uint8_t &snr)
{
	uint8_t chunk[64];
	ssize_t n;

	while ((n = ::read(_fd, chunk, sizeof(chunk))) > 0) {
		const unsigned count = (unsigned)n;

		// Keep the newest bytes if more arrive than the buffer holds.
		if (_rx_len + count > sizeof(_rx)) {
			const unsigned drop = _rx_len + count - sizeof(_rx);
			const unsigned keep = _rx_len > drop ? _rx_len - drop : 0;
			memmove(_rx, _rx + (_rx_len - keep), keep);
			_rx_len = keep;
		}

		memcpy(_rx + _rx_len, chunk, count);
		_rx_len += count;
	}

	// Newest valid packet first. The checksum is the sum of version, altitude and SNR.
	for (int i = (int)_rx_len - (int)PACKET_LENGTH; i >= 0; i--) {
		if (_rx[i] == PACKET_HEADER && _rx[i + 1] == PACKET_VERSION
		    && _rx[i + 5] == (uint8_t)(_rx[i + 1] + _rx[i + 2] + _rx[i + 3] + _rx[i + 4])) {
			distance_cm = (_rx[i + 3] << 8) | _rx[i + 2];
			snr = _rx[i + 4];

			const unsigned used = i + PACKET_LENGTH;
			memmove(_rx, _rx + used, _rx_len - used);
			_rx_len -= used;
			return true;
		}
	}

	// Nothing usable: keep only what could still be the start of a packet.
	if (_rx_len >= PACKET_LENGTH) {
		memmove(_rx, _rx + _rx_len - (PACKET_LENGTH - 1), PACKET_LENGTH - 1);
		_rx_len = PACKET_LENGTH - 1;
	}

	return false;
}

void UsD1Radar::Run()
{
	if (open_serial_port() != PX4_OK) {
		return;
	}

	perf_begin(_sample_perf);
	const hrt_abstime timestamp_sample = hrt_absolute_time();
	int distance_cm = 0;
	uint8_t snr = 0;

	if (read_packet(distance_cm, snr)) {
		if (snr <= SNR_MIN_DB) {
			// Erroneous by the manual (2.4): no obstacle seen, not an obstacle at this distance.
			// Collision prevention reads distance -1 with quality 0 as "free up to the maximum range".
			_px4_rangefinder.update(timestamp_sample, -1.f, 0);
			_low_snr++;

		} else {
			// 1 at the limit up to 100 at 50 dB: only the order matters.
			const int quality = math::constrain((int)(snr - SNR_MIN_DB) * 100 / 37, 1, 100);
			_px4_rangefinder.update(timestamp_sample, distance_cm / 100.f, quality);
			_valid++;
		}

	}

	perf_end(_sample_perf);
}

void UsD1Radar::print_info()
{
	PX4_INFO("%s rotation %u: %u valid, %u at or below %u dB", _port, _rotation, (unsigned)_valid, (unsigned)_low_snr,
		 (unsigned)SNR_MIN_DB);
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
}
