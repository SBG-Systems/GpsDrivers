#pragma once

// uORB headers
#include <uORB/uORB.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sbg_bin.h>

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
			GPSDriver(GPSCallbackPtr callback, void* callback_user, struct sensor_gps_s *gps_position, float heading_offset, int serial_id);
			~GPSDriver(void);

			int configure(unsigned &baud, OutputMode output_mode) override;

			int receive(unsigned int timeout) override;

			int reset(GPSRestartType restart_type) override;

		private:
			static SbgErrorCode onReadCallback(void *p_buffer, size_t read_bytes, void *user_arg);
			void onRead(void *p_buffer, size_t bytes_received);

			static SbgErrorCode onLogReceivedCallback(SbgEComHandle *p_handle, SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData *p_log_data, void *p_user_arg);
			void onLogReceived(SbgEComClass msg_class, SbgEComMsgId msg, const SbgBinaryLogData &ref_sbg_data, uint64_t system_timestamp);

			void processGpsVel(const SbgLogGpsVel *p_vel);
			void processGpsPos(const SbgLogGpsPos *p_gps_pos, uint64_t system_timestamp);
			void processGpsHdt(const SbgLogGpsHdt *p_gps_hdt);
			void processUtc(const SbgLogUtcData *p_utc);

			bool isTimeout(uint64_t start_time, uint64_t timeout);

			struct sensor_gps_s					*p_gps_position;

			float								 _heading_offset;
			int									 _serial_id;

			SbgInterface						 _sbg_interface;
			SbgEComHandle						 _com_handle;

			uint64_t							 _utc_timestamp;

			bool								 _pos_received;
			bool								 _vel_received;

			sbg_bin_s							 _bin;
			uORB::Publication<sbg_bin_s>		 _sbg_bin_pub{ORB_ID(sbg_bin)};
		};
	}
}
