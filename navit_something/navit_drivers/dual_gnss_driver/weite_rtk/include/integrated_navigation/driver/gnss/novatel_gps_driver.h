#ifndef SRC_NOVATEL_GPS_H
#define SRC_NOVATEL_GPS_H

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/publisher.h>
#include <std_msgs/String.h>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/max.hpp>
#include <boost/accumulators/statistics/mean.hpp>
#include <boost/accumulators/statistics/min.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/variance.hpp>
#include <boost/circular_buffer.hpp>
#include <exception>
#include <string>

#include <navit_msgs/NovatelPosVelHeading.h>
#include <navit_msgs/NovatelPosition.h>
#include <navit_msgs/NovatelVelocity.h>
#include <navit_msgs/NovatelXYZ.h>
#include <navit_msgs/NovatelDualAntennaHeading.h>
#include <navit_msgs/Gpgga.h>
#include <navit_msgs/Time.h>
#include "integrated_navigation/driver/io_common/serial_common.h"
#include "integrated_navigation/driver/gnss/NovatelGps.h"
#include "integrated_navigation/publisher/gpchc_publisher.h"

#include "integrated_navigation/log/logging.h"
#include "ros/publisher.h"
#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>

namespace stats = boost::accumulators;

namespace integrated_navigation {
    class novatel_gps_driver {
    public:
        novatel_gps_driver(ros::NodeHandle &nh, std::shared_ptr<io_common> &serial);
        ~novatel_gps_driver();

        void novatel_data_callback(std::vector<uint8_t> &data_buffer,std::string data_str=std::string(""));

    private:
        void CalculateTimeSync();
        void FixDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);
        void SyncDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);
        void DeviceDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);
        void GpsDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);
        void DataDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);
        void RateDiagnostic(diagnostic_updater::DiagnosticStatusWrapper &status);

        std::string frame_id_;

        bool publish_novatel_pose_vel_;
        bool publish_novatel_positions_;
        bool publish_novatel_xyz_positions_;
        bool publish_novatel_velocity_;
        bool publish_novatel_dual_antenna_heading_;
        bool publish_gpchc_;
        bool publish_gpgga_;
        bool publish_gprmc_;
        bool publish_navsatfix_;

        ros::Publisher fusion_pub_;
        ros::Publisher novatel_position_pub_;
        ros::Publisher novatel_xyz_position_pub_;
        ros::Publisher novatel_velocity_pub_;
        ros::Publisher novatel_dual_antenna_heading_pub_;
        ros::Publisher gpchc_pub_;
        ros::Publisher gpgga_pub_;
        ros::Publisher gprmc_pub_;
        ros::Publisher fix_data_pub_;
        ros::Publisher navsatfix_pub_;
        ros::Publisher rtk_health_status_pub_;
        diagnostic_updater::Updater diag_updater_;

        std::shared_ptr<io_common> serial_;
        std::shared_ptr<NovatelGps> gps_;
        std::shared_ptr<GpchcPublisher> gpchc_;

        boost::mutex mutex_;

        ros::Time last_sync_;
        /// Buffer of sync message time stamps
        boost::circular_buffer<ros::Time> sync_times_;
        /// Buffer of gps message time stamps
        boost::circular_buffer<ros::Time> msg_times_;
        /// Stats on time offset
        stats::accumulator_set<float, stats::stats<stats::tag::max, stats::tag::min, stats::tag::mean, stats::tag::variance> >
                offset_stats_;

        unsigned int gpchc_count_;
        double expected_rate_;
        int32_t device_timeouts_;
        int32_t device_interrupts_;
        int32_t device_errors_;
        int32_t gps_parse_failures_;
        int32_t gps_insufficient_data_warnings_;
        int32_t publish_rate_warnings_;
        int32_t measurement_count_;
        ros::Time last_published_;
        // ROS diagnostics
        std::string error_msg_;
        diagnostic_updater::Updater diagnostic_updater_;
        navit_msgs::NovatelPositionPtr last_novatel_position_;
    };
}
#endif //SRC_NOVATEL_GPS_H