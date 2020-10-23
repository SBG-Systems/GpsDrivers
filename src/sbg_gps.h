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
 * @file sbg_gps.h
 *
 * @author Brice Saussay <brice.saussay@sbg-systems.com>
 */

#pragma once

// uORB headers
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>

// sbgCommonLib headers
#include <sbgECom/src/sbgEComLib.h>
#include <sbgECom/common/interfaces/sbgInterface.h>

// Project headers
#include "gps_helper.h"
#include "base_station.h"
#include "../../definitions.h"

namespace px4
{
	namespace sbg
	{

		class GPSDriver : public GPSBaseStationSupport
		{
		public:
			GPSDriver(GPSCallbackPtr callback, void* callback_user, struct sensor_gps_s *gps_position, float heading_offset);
			~GPSDriver(void);

			int configure(unsigned &baud, OutputMode output_mode) override;

			int receive(unsigned int timeout) override;

			int reset(GPSRestartType restart_type) override;

		private:
			/**
			 * Read callback
			 * @param p_interface pointer on an initialized interface
			 * @param p_buffer pointer on an allocated buffer that can hold at least bytesToRead bytes of data
			 * @param p_read_bytes returns the number of bytes actually read (can be zero and up to bytesToRead)
			 * @param p_read_bytes maximum number of bytes to try to read on the interface
			 * @return: SBG_NO_ERROR if zero or some bytes have been read successfully
			 */
			static SbgErrorCode readCallback(SbgInterface *p_interface, void *p_buffer, size_t *p_read_bytes, size_t bytes_to_read);

			/**
			 * Callback called each time a new log is received
			 * @param p_handle Valid handle on the sbgECom instance that has called this callback.
			 * @param msg_class Class of the message we have received
			 * @param msg Message ID of the log received.
			 * @param p_log_data Contains the received log data as an union.
			 * @param p_user_arg Optional user supplied argument.
			 * @return: SBG_NO_ERROR if the received log has been used successfully.
			 */
			static SbgErrorCode onLogReceivedCallback(SbgEComHandle *p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData *p_log_data, void *p_user_arg);

			/**
			 * Callback called each time a new log is received
			 * @param msgClass Class of the message we have received
			 * @param msg Message ID of the log received.
			 * @param p_log_data Contains the received log data as an union.
			 * @param p_user_arg Optional user supplied argument.
			 * @return: SBG_NO_ERROR if the received log has been used successfully.
			 */
			void onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData &ref_sbg_data, uint64_t system_timestamp);

			/**
			 * Process GPS velocity log
			 * @param p_gps_vel pointer on a structure that contains the GPS velocity log to process
			 */
			void processGpsVel(const SbgLogGpsVel *p_gps_vel);

			/**
			 * Process GPS position log
			 * @param p_pos pointer on a structure that contains the GPS position log to process
			 * @param system_timestamp timestamp from the system start, in microseconds
			 */
			void processGpsPos(const SbgLogGpsPos *p_gps_pos, uint64_t system_timestamp);

			/**
			 * Process GPS heading log
			 * @param p_gps_hdt pointer on a structure that contains the GPS heading log to process
			 */
			void processGpsHdt(const SbgLogGpsHdt *p_gps_hdt);

			/**
			 * Process UTC data log
			 * @param p_vel pointer on a structure that contains the UTC data log to process
			 */
			void processUtc(const SbgLogUtcData *p_utc);

			/**
			 * Check if the driver is timeout
			 * @param start_time in ms
			 * @param timeout in ms
			 */
			bool isTimeout(uint64_t start_time, uint64_t timeout);

			struct sensor_gps_s					*p_gps_position;

			float								 _heading_offset;

			SbgInterface						 _sbg_interface;
			SbgEComHandle						 _com_handle;

			uint64_t							 _utc_timestamp;

			bool								 _pos_received;
			bool								 _vel_received;
		};
	}
}
