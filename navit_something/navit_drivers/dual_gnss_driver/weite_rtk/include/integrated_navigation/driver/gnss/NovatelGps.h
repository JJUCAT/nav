#ifndef NOVATEL_OEM628_NOVATEL_GPS_H_
#define NOVATEL_OEM628_NOVATEL_GPS_H_

#include <boost/optional/optional.hpp>
#include <integrated_navigation/library/GnssParserLib/novatel_message_extractor.h>
#include <integrated_navigation/library/GnssParserLib/parsers/bestpos.h>
#include <integrated_navigation/library/GnssParserLib/parsers/bestvel.h>
#include <integrated_navigation/library/GnssParserLib/parsers/bestxyz.h>
#include <integrated_navigation/library/GnssParserLib/parsers/dual_antenna_heading.h>

#include <integrated_navigation/library/GnssParserLib/parsers/gpchc.h>
#include <integrated_navigation/library/GnssParserLib/parsers/gpgga.h>
#include <integrated_navigation/library/GnssParserLib/parsers/gprmc.h>
#include <integrated_navigation/library/GnssParserLib/parsers/refstation.h>
#include <integrated_navigation/library/GnssParserLib/parsers/time.h>

#include <navit_msgs/Gpchc.h>
#include <navit_msgs/Gpgga.h>
#include <navit_msgs/Gprmc.h>
#include <navit_msgs/NovatelDualAntennaHeading.h>
#include <navit_msgs/NovatelPosVelHeading.h>
#include <navit_msgs/NovatelPosition.h>
#include <navit_msgs/NovatelVelocity.h>
#include <navit_msgs/NovatelXYZ.h>
#include <navit_msgs/Time.h>

#include <sensor_msgs/NavSatFix.h>

#include <boost/asio.hpp>
#include <boost/circular_buffer.hpp>
#include <map>
#include <queue>
#include <string>
#include <vector>

#include "integrated_navigation/driver/io_common/serial_common.h"

namespace integrated_navigation {
    class NovatelGps {
        public:
            NovatelGps();
            ~NovatelGps();

            std::string ErrorMsg() const { return error_msg_; }
            void GetSyncedPosVelHeading(std::vector<navit_msgs::NovatelPosVelHeadingPtr>& pos_vel_heading);
            void GetGpchcMessages(std::vector<navit_msgs::GpchcPtr>& gpchc_msgs);
            void GetGpggaMessages(std::vector<navit_msgs::GpggaPtr>& gpgga_messages);
            void GetGprmcMessages(std::vector<navit_msgs::GprmcPtr>& gprmc_messages);
            void GetFixMessages(std::vector<sensor_msgs::NavSatFixPtr> &fix);
            void GetNovatelDualAntennaHeadingMessages(
              std::vector<navit_msgs::NovatelDualAntennaHeadingPtr>& headings);
            void GetNovatelPositions(std::vector<navit_msgs::NovatelPositionPtr>& positions);
            void GetNovatelXYZPositions(std::vector<navit_msgs::NovatelXYZPtr>& positions);
            void GetNovatelVelocities(std::vector<navit_msgs::NovatelVelocityPtr>& velocities);
            void GetTimeMessages(std::vector<navit_msgs::TimePtr>& time_messages);
            std::string GetBoardType();

            serial_common::ReadResult ProcessData(std::vector<uint8_t> &data_buffer,std::string data_str=std::string(""));

            double gpsfix_sync_tol_;  // seconds
            bool wait_for_sync_;      // wait until a bestvel has arrived before publishing bestpos

        private:
            serial_common::ReadResult ParseNmeaSentence(const NmeaSentence& sentence, const ros::Time& stamp,
                                                    double most_recent_utc_time) noexcept(false);
            serial_common::ReadResult ParseNovatelSentence(const NovatelSentence& sentence, const ros::Time& stamp) noexcept(false);
            serial_common::ReadResult ParseBinaryMessage(const BinaryMessage& msg, const ros::Time& stamp) noexcept(false);

            double handleCov(const double &x) {
              return std::pow(-0.00285714285714284 + -0.214285714285712 * x + 207.142857142857 * x * x, 2);
            }

            static constexpr uint16_t DEFAULT_TCP_PORT = 3001;
            static constexpr uint16_t DEFAULT_UDP_PORT = 3002;
            static constexpr size_t MAX_BUFFER_SIZE = 100;
            static constexpr size_t SYNC_BUFFER_SIZE = 10;
            static constexpr uint32_t SECONDS_PER_WEEK = 604800;
            static constexpr double IMU_TOLERANCE_S = 0.0002;
            static constexpr double DEGREES_TO_RADIANS = M_PI / 180.0;

            double utc_offset_;
            std::shared_ptr<serial_common> serial_;
            std::string error_msg_;

            /// Buffer containing incomplete data from message parsing
            std::string nmea_buffer_;

            /// Used to extract messages from the incoming data stream
            NovatelMessageExtractor extractor_;

            // Message parsers
            BestposParser bestpos_parser_;
            BestxyzParser bestxyz_parser_;
            BestvelParser bestvel_parser_;
            DualAntennaHeadingParser dual_antenna_heading_parser_;
            RefStationParser refstation_parser_;
            TimeParser time_parser_;

            GpchcParser gpchc_parser_;
            GpggaParser gpgga_parser_;
            GprmcParser gprmc_parser_;
        

            // Message buffers
            boost::circular_buffer<navit_msgs::NovatelPositionPtr> novatel_positions_;
            boost::circular_buffer<navit_msgs::NovatelVelocityPtr> novatel_velocities_;
            boost::circular_buffer<navit_msgs::NovatelXYZPtr> novatel_xyz_positions_;
            boost::circular_buffer<navit_msgs::NovatelDualAntennaHeadingPtr> dual_antenna_heading_msgs_;
            boost::circular_buffer<navit_msgs::GpchcPtr> gpchc_msgs_;
            boost::circular_buffer<navit_msgs::GpggaPtr> gpgga_msgs_;
            boost::circular_buffer<navit_msgs::GprmcPtr> gprmc_msgs_;
      
            boost::circular_buffer<navit_msgs::NovatelRefStationPtr> ref_station_msgs_;
            boost::circular_buffer<navit_msgs::NovatelPositionPtr> bestpos_fix_buffer_;
            boost::circular_buffer<navit_msgs::NovatelPositionPtr> bestpos_sync_buffer_;
            boost::circular_buffer<navit_msgs::NovatelVelocityPtr> bestvel_sync_buffer_;
            boost::circular_buffer<navit_msgs::NovatelXYZPtr> bestxyz_sync_buffer_;
            boost::circular_buffer<navit_msgs::NovatelDualAntennaHeadingPtr> heading_sync_buffer_;
            boost::circular_buffer<navit_msgs::GpggaPtr> gpgga_sync_buffer_;
            boost::circular_buffer<navit_msgs::NovatelRefStationPtr> ref_station_sync_buffer_;
            boost::circular_buffer<navit_msgs::TimePtr> time_msgs_;

            bool heading_482_;
            boost::optional<std::string> board_type_;
    };
}  // namespace integrated_navigation
#endif  // NOVATEL_OEM628_NOVATEL_GPS_H_