// Standard headers
#include <math.h>
#include <poll.h>
#include <termios.h>

// Project headers
#include "sbg_gps.h"

namespace px4
{
namespace sbg
{

typedef SbgErrorCode (*ReadCallback) (void *p_buffer, size_t read_bytes, void *p_user_arg);

class Serial
{
	public:
	Serial(int serial_id, ReadCallback read_callback, void *p_arg);

	~Serial(void);

	SbgErrorCode init(unsigned baudrate);

	SbgErrorCode flush(void);

	int									 _id;
	ReadCallback						 read;
	void								*p_user_arg;
};

Serial::Serial(int serial_id, ReadCallback read_callback, void *p_arg)
:	_id(serial_id),
	read(read_callback),
	p_user_arg(p_arg)
{
}

SbgErrorCode Serial::init(uint32_t baudrate)
{
	SbgErrorCode						 error_code;
	struct termios						 config;

	if (tcgetattr((_id), &config) != -1)
	{
		config.c_cflag |=  (CLOCAL | CREAD);
		config.c_cflag &= ~(PARENB|CSTOPB|CSIZE);
		config.c_cflag |= CS8;
		config.c_cflag &= ~CRTSCTS;

		config.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|IXON);

		config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
		config.c_oflag = 0;

		config.c_cc[VMIN] = 0;
		config.c_cc[VTIME] = 0;

		if ((cfsetispeed(&config, baudrate) >= 0) && (cfsetospeed(&config, baudrate) >= 0))
		{
			if (tcsetattr(_id, TCSANOW, &config) == 0)
			{
				error_code = SBG_NO_ERROR;
			}
			else
			{
				error_code = SBG_ERROR;
				SBG_LOG_ERROR(error_code, "unable to set attributes");
			}
		}
		else
		{
			error_code = SBG_ERROR;
			SBG_LOG_ERROR(error_code, "unable to set baudrate");
		}
	}
	else
	{
		error_code = SBG_ERROR;
		SBG_LOG_ERROR(error_code, "unale to get attributes");
	}

	return error_code;
}

SbgErrorCode Serial::flush(void)
{
	SbgErrorCode						 error_code;

	if (tcflush(_id, TCIOFLUSH) == 0)
	{
	  error_code = SBG_NO_ERROR;
	}
	else
	{
		error_code = SBG_ERROR;
		SBG_LOG_ERROR(error_code, "unable to flush");
	}

	return error_code;
}

class Interface
{
	public:
	SbgErrorCode open(SbgInterface *p_interface, int serial_id, unsigned baudrate, ReadCallback read_callback, void *p_user_arg);

	private:
	uint32_t getBaudRateConst(unsigned baudRate);

	static SbgErrorCode serialRead(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read);
};

uint32_t Interface::getBaudRateConst(unsigned baudRate)
{
	uint32_t							 baudrate_const;

	switch (baudRate)
	{
		case 9600:
			baudrate_const = B9600;
			break;
		case 19200:
			baudrate_const = B19200;
			break;
		#ifdef B38400
		case 38400:
			baudrate_const = B38400;
			break;
		#endif
		#ifdef B57600
		case 57600:
			baudrate_const = B57600;
			break;
		#endif
		#ifdef B115200
		case 115200:
			baudrate_const = B115200;
			break;
		#endif
		#ifdef B230400
		case 230400:
			baudrate_const = B230400;
			break;
		#endif
		#ifdef B460800
		case 460800:
			baudrate_const = B460800;
			break;
		#endif
		#ifdef B921600
		case 921600:
			baudrate_const = B921600;
			break;
		#endif // B921600
		default:
			baudrate_const = baudRate;
	}

	return baudrate_const;
}

SbgErrorCode Interface::serialRead(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read)
{
	SbgErrorCode						 error_code;
	Serial								*p_serial;
	pollfd								 fds[1];
	int									 result;

	assert(p_interface);
	assert(p_read_bytes);
	assert(p_interface->type == SBG_IF_TYPE_SERIAL);

	p_serial = (Serial *)p_interface->handle;

	fds[0].fd = p_serial->_id;
	fds[0].events = POLLIN;

	result = poll(fds, sizeof(fds) / sizeof(fds[0]), 0);

	if (result > 0)
	{
		ssize_t							 bytes_read;

		bytes_read = ::read(p_serial->_id, p_buffer, bytes_to_read);

		if (bytes_read >= 0)
		{
			*p_read_bytes = (size_t)bytes_read;

			error_code = p_serial->read(p_buffer, *p_read_bytes, p_serial->p_user_arg);
		}
		else
		{
			*p_read_bytes = 0;

			error_code = SBG_READ_ERROR;
		}
	}
	else
	{
		*p_read_bytes = 0;

		error_code = SBG_NOT_READY;
	}

	return error_code;
}

SbgErrorCode Interface::open(SbgInterface *p_interface, int serial_id, unsigned baudrate, ReadCallback read_callback, void *p_user_arg)
{
	SbgErrorCode						 error_code;
	Serial								*p_serial;

	p_serial = new Serial(serial_id, read_callback, p_user_arg);

	if (p_serial)
	{
		uint32_t						 baudrate_const;

		baudrate_const = getBaudRateConst(baudrate);

		error_code = p_serial->init(baudrate_const);

		if (error_code == SBG_NO_ERROR)
		{
			error_code = p_serial->flush();

			if (error_code == SBG_NO_ERROR)
			{
				p_interface->handle			= p_serial;
				p_interface->type			= SBG_IF_TYPE_SERIAL;
				snprintf(p_interface->name, sizeof(p_interface->name), "uart%u", serial_id);
				p_interface->pReadFunc		= serialRead;
			}
		}
	}
	else
	{
		error_code = SBG_MALLOC_FAILED;
		SBG_LOG_ERROR(error_code, "malloc fails");
	}

	return error_code;
}

GPSDriver::GPSDriver(GPSCallbackPtr callback, void *callback_user, struct sensor_gps_s *gps_position, float heading_offset, int serial_id)
:	GPSBaseStationSupport(callback, callback_user),
	p_gps_position(gps_position),
	_heading_offset(heading_offset),
	_serial_id(serial_id)
{
	_pos_received = false;
	_vel_received = false;
	_utc_timestamp = 0;
	_bin.bytes_read = 0;

	_sbg_bin_pub.advertise();
}

GPSDriver::~GPSDriver(void)
{
	sbgEComClose(&_com_handle);
}

int GPSDriver::reset(GPSRestartType restart_type)
{
	return -1;
}

int GPSDriver::configure(unsigned &baudrate, OutputMode output_mode)
{
	int									 result;

	result = -1;

	if (output_mode == OutputMode::GPS)
	{
		SbgErrorCode					 error_code;
		sbg::Interface					 interface;

		error_code = interface.open(&_sbg_interface, _serial_id, baudrate, onReadCallback, this);

		if (error_code == SBG_NO_ERROR)
		{
			error_code = sbgEComInit(&_com_handle, &_sbg_interface);

			if (error_code == SBG_NO_ERROR)
			{
				sbgEComSetReceiveLogCallback(&_com_handle, onLogReceivedCallback, this);

				result = 0;
			}
			else
			{
				PX4_ERR("couldn't init sbgECom");
			}
		}
		else
		{
			PX4_ERR("couldn't init interface");
		}
	}
	else
	{
		PX4_ERR("unsupported Output Mode %i", (int)output_mode);
	}

	return result;
}

int GPSDriver::receive(unsigned timeout)
{
	int									 result;
	uint64_t							 start_time;

	start_time = gps_absolute_time();

	for (;;)
	{
		sbgEComHandleOneLog(&_com_handle);

		if (_pos_received && _vel_received)
		{
			_rate_count_vel++;
			_rate_count_lat_lon++;

			result = 1;

			_pos_received = false;
			_vel_received = false;

			break;
		}
		else if (isTimeout(start_time, timeout))
		{
			result = -1;

			_pos_received = false;
			_vel_received = false;

			PX4_ERR("timeout");
			break;
		}

		px4_usleep(1000);
	}

	return result;
}

void GPSDriver::onRead(void *p_buffer, size_t bytes_received)
{
	size_t								 bytes_remaining;
	char								*p_head;

	bytes_remaining = bytes_received;

	p_head = (char *)p_buffer;

	while (bytes_remaining != 0)
	{
		bool							 published;
		size_t							 bytes_available;

		bytes_available = sizeof(_bin.data) - _bin.bytes_read;

		if (bytes_remaining <= bytes_available)
		{
			memcpy(&_bin.data[_bin.bytes_read], p_head, bytes_remaining);

			_bin.bytes_read	+= bytes_remaining;

			p_head = &p_head[bytes_remaining];

			bytes_remaining = 0;
		}
		else
		{
			memcpy(&_bin.data[_bin.bytes_read], p_head, bytes_available);

			p_head = &p_head[bytes_available];

			bytes_remaining -= bytes_available;

			_bin.timestamp = hrt_absolute_time();

			_bin.bytes_read	+= bytes_available;

			published = _sbg_bin_pub.publish(_bin);

			if (published)
			{
				_bin.bytes_read = 0;
			}
			else
			{
				PX4_ERR("unable to publish sbg bin data");
			}
		}
	}
}

SbgErrorCode GPSDriver::onReadCallback(void *p_buffer, size_t read_bytes, void *p_user_arg)
{
	GPSDriver						*p_sbg_driver;

	assert(p_user_arg);

	p_sbg_driver = (GPSDriver *)p_user_arg;

	p_sbg_driver->onRead(p_buffer, read_bytes);

	return SBG_NO_ERROR;
}

void GPSDriver::onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData &ref_sbg_data, uint64_t system_timestamp)
{
	if (msg_class == SBG_ECOM_CLASS_LOG_ECOM_0)
	{
		switch (msg)
		{
		case SBG_ECOM_LOG_UTC_TIME:
			processUtc(&ref_sbg_data.utcData);
			break;
		case SBG_ECOM_LOG_GPS1_VEL:
		case SBG_ECOM_LOG_GPS2_VEL:
			processGpsVel(&ref_sbg_data.gpsVelData);
			break;
		case SBG_ECOM_LOG_GPS1_POS:
		case SBG_ECOM_LOG_GPS2_POS:
			processGpsPos(&ref_sbg_data.gpsPosData, system_timestamp);
			break;
		case SBG_ECOM_LOG_GPS1_HDT:
		case SBG_ECOM_LOG_GPS2_HDT:
			processGpsHdt(&ref_sbg_data.gpsHdtData);
			break;
		default:
			break;
		}
	}
}

SbgErrorCode GPSDriver::onLogReceivedCallback(SbgEComHandle *p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData *p_log_data, void* p_user_arg)
{
	uint64_t							 system_timestamp;
	GPSDriver							*p_sbg_driver;

	assert(p_user_arg);
	SBG_UNUSED_PARAMETER(p_handle);

	system_timestamp = hrt_absolute_time();

	p_sbg_driver = (GPSDriver *)p_user_arg;

	p_sbg_driver->onLogReceived(msg_class, msg, *p_log_data, system_timestamp);

	return SBG_NO_ERROR;
}

bool GPSDriver::isTimeout(uint64_t start_time, uint64_t timeout)
{
	bool								 is_timeout;

	if ((start_time + timeout * 1000) < gps_absolute_time())
	{
		is_timeout = true;
	}
	else
	{
		is_timeout = false;
	}

	return is_timeout;
}

void GPSDriver::processGpsVel(const SbgLogGpsVel *p_vel)
{
	p_gps_position->vel_m_s = sqrtf(powf(p_vel->velocity[0], 2) + powf(p_vel->velocity[1], 2));
	p_gps_position->vel_n_m_s = p_vel->velocity[0];
	p_gps_position->vel_e_m_s = p_vel->velocity[1];
	p_gps_position->vel_d_m_s = p_vel->velocity[2];

	p_gps_position->s_variance_m_s = sqrtf(powf(p_vel->velocityAcc[0], 2) + powf(p_vel->velocityAcc[1], 2) + powf(p_vel->velocityAcc[2], 2));

	p_gps_position->vel_ned_valid = true;

	p_gps_position->cog_rad = sbgDegToRadF(p_vel->course);

	p_gps_position->c_variance_rad = sbgDegToRadF(p_vel->courseAcc);

	_vel_received = true;
}

void GPSDriver::processGpsPos(const SbgLogGpsPos *p_pos, uint64_t system_timestamp)
{
	uint8_t								 type;

	p_gps_position->timestamp = system_timestamp;

	p_gps_position->timestamp_time_relative = (int32_t)(_utc_timestamp - p_pos->timeStamp);

	p_gps_position->lat = (int32_t)(p_pos->latitude * 10000000);
	p_gps_position->lon =(int32_t)(p_pos->longitude * 10000000);
	p_gps_position->alt =(int32_t)(p_pos->altitude * 1000);

	p_gps_position->alt_ellipsoid = (int32_t)((p_pos->altitude + (double)p_pos->undulation) * 1000);

	p_gps_position->eph = sqrtf(powf(p_pos->longitudeAccuracy, 2) + powf(p_pos->latitudeAccuracy, 2));
	p_gps_position->epv = p_pos->altitudeAccuracy;

	 type = sbgEComLogGpsPosGetType(p_pos->status);

	switch (type)
	{
		case SBG_ECOM_POS_NO_SOLUTION:
		p_gps_position->fix_type = 0;
		break;
		case SBG_ECOM_POS_RTK_FLOAT:
		p_gps_position->fix_type = 5;
		break;
		case SBG_ECOM_POS_RTK_INT:
		p_gps_position->fix_type = 6;
		break;
		default:
		p_gps_position->fix_type = 3;
		break;
	}

	p_gps_position->satellites_used = p_pos->numSvUsed;

	_pos_received = true;
}

void GPSDriver::processGpsHdt(const SbgLogGpsHdt *p_gps_hdt)
{
	p_gps_position->heading = sbgDegToRadF(p_gps_hdt->heading);

	p_gps_position->heading_offset = _heading_offset;
}

void GPSDriver::processUtc(const SbgLogUtcData *p_utc)
{
	bool								 clock_stable;
	bool								 utc_sync;
	uint8_t								 clock_status;

	clock_stable = ((p_utc->status & SBG_ECOM_CLOCK_STABLE_INPUT) != 0);

	utc_sync = ((p_utc->status & SBG_ECOM_CLOCK_UTC_SYNC) != 0);

	clock_status = sbgEComLogUtcGetClockStatus(p_utc->status);

	if (clock_stable && utc_sync && clock_status == SBG_ECOM_CLOCK_VALID)
	{
		time_t							 epoch;
		tm								 timeinfo{};

		timeinfo.tm_year	= p_utc->year - 1900;
		timeinfo.tm_mon		= p_utc->month - 1;
		timeinfo.tm_mday	= p_utc->day;
		timeinfo.tm_hour	= p_utc->hour;
		timeinfo.tm_min		= p_utc->minute;
		timeinfo.tm_sec		= p_utc->second;

#ifndef NO_MKTIME
		epoch = mktime(&timeinfo);

		if (epoch > GPS_EPOCH_SECS)
		{
			timespec					 ts{};

			ts.tv_sec = epoch;
			ts.tv_nsec = p_utc->nanoSecond;

			setClock(ts);

			p_gps_position->time_utc_usec = (uint64_t)epoch * 1000000ULL;
			p_gps_position->time_utc_usec += (uint64_t)p_utc->nanoSecond / 1000;

			_utc_timestamp = p_utc->timeStamp;
		}
		else
		{
			PX4_DEBUG("unable to get epoch time in seconds");
			p_gps_position->time_utc_usec = 0;
		}
#else

		PX4_DEBUG("mktime function not define");
		p_gps_position->time_utc_usec = 0;
#endif
	}
	else
	{
		PX4_DEBUG("UTC clock not ready");
		p_gps_position->time_utc_usec = 0;
	}
}
}
}
