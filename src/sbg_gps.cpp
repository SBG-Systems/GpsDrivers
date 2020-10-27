/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file sbg_gps.cpp
 *
 * @author Brice Saussay <brice.saussay@sbg-systems.com>
 */

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
GPSDriver::GPSDriver(GPSCallbackPtr callback, void *callback_user, struct sensor_gps_s *gps_position, float heading_offset)
:	GPSBaseStationSupport(callback, callback_user),
	p_gps_position(gps_position),
	_heading_offset(heading_offset)
{
	_pos_received = false;
	_vel_received = false;
	_utc_timestamp = 0;
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

		result = setBaudrate(baudrate);

		if (result == 0)
		{
			// Initialise the interface
			_sbg_interface.handle		= this;
			_sbg_interface.type			= SBG_IF_TYPE_SERIAL;
			_sbg_interface.pReadFunc	= readCallback;

			// Initialise the sbgECom handler
			error_code = sbgEComInit(&_com_handle, &_sbg_interface);

			if (error_code == SBG_NO_ERROR)
			{
				// Attach the callback that handle received log
				sbgEComSetReceiveLogCallback(&_com_handle, onLogReceivedCallback, this);

				result = 0;
			}
			else
			{
				SBG_LOG_DEBUG("couldn't init sbgECom");
			}
		}
	}
	else
	{
		SBG_LOG_ERROR(SBG_INVALID_PARAMETER, "unsupported Output Mode %i", (int)output_mode);
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
		// Handle incoming log
		sbgEComHandleOneLog(&_com_handle);

		// Check if GPS position and GPS velocity logs have been received
		if (_pos_received && _vel_received)
		{
			_rate_count_vel++;
			_rate_count_lat_lon++;

			result = 1;

			_pos_received = false;
			_vel_received = false;

			break;
		}
		// Check if a timeout ocurred
		else if (isTimeout(start_time, timeout * 1000))
		{
			result = -1;

			_pos_received = false;
			_vel_received = false;

			SBG_LOG_ERROR(SBG_TIME_OUT, "timeout");
			break;
		}

		px4_usleep(1000);
	}

	return result;
}

SbgErrorCode GPSDriver::readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read)
{
	SbgErrorCode					 error_code;
	GPSDriver						*p_sbg_driver;
	int								 result;

	assert(p_interface);

	p_sbg_driver = (GPSDriver *)p_interface->handle;

	// Call GPS helper read method
	result = p_sbg_driver->read((uint8_t *)p_buffer, bytes_to_read, 0);

	if (result >= 0)
	{
		*p_read_bytes = result;
		 error_code = SBG_NO_ERROR;
	}
	else
	{
		error_code = SBG_NOT_READY;
	}

	return error_code;
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

	if ((start_time + timeout) < gps_absolute_time())
	{
		is_timeout = true;
	}
	else
	{
		is_timeout = false;
	}

	return is_timeout;
}

void GPSDriver::processGpsVel(const SbgLogGpsVel *p_gps_vel)
{
	p_gps_position->vel_m_s = sqrtf(powf(p_gps_vel->velocity[0], 2) + powf(p_gps_vel->velocity[1], 2));
	p_gps_position->vel_n_m_s = p_gps_vel->velocity[0];
	p_gps_position->vel_e_m_s = p_gps_vel->velocity[1];
	p_gps_position->vel_d_m_s = p_gps_vel->velocity[2];

	p_gps_position->s_variance_m_s = sqrtf(powf(p_gps_vel->velocityAcc[0], 2) + powf(p_gps_vel->velocityAcc[1], 2) + powf(p_gps_vel->velocityAcc[2], 2));

	p_gps_position->vel_ned_valid = true;

	p_gps_position->cog_rad = sbgDegToRadF(p_gps_vel->course);

	p_gps_position->c_variance_rad = sbgDegToRadF(p_gps_vel->courseAcc);

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
