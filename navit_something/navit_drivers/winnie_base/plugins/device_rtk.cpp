//
// Created by fan on 23-6-25.
//

#include <GnssParserLib/parsers/parsing_utils.h>
#include <navit_msgs/NovatelPosVelHeading.h>

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static navit_msgs::NovatelPosVelHeading::Ptr to_ros_msg(const winnie_base::cmd_rtk_info& msg_sdk) {
    std::map<uint8_t, std::string> GPS_TIME_STATUS = {
        {20, "UNKNOWN"},        {60, "APPROXIMATE"},     {80, "COARSEADJUSTING"},
        {100, "COARSE"},        {120, "COARSESTEERING"}, {130, "FREEWHEELING"},
        {140, "FINEADJUSTING"}, {160, "FINE"},           {170, "FINEBACKUPSTEERING"},
        {180, "FINESTEERING"},  {200, "SATTIME"}};

    navit_msgs::NovatelPosVelHeading::Ptr msg_ros_novatel(
        new navit_msgs::NovatelPosVelHeading());
    // Ros Custom Msg Header
    msg_ros_novatel->novatel_msg_header.port              = integrated_navigation::PORT_IDENTIFIERS[msg_sdk.port];
    msg_ros_novatel->novatel_msg_header.sequence_num      = msg_sdk.sequence_num;
    msg_ros_novatel->novatel_msg_header.percent_idle_time = msg_sdk.percent_idle_time;
    msg_ros_novatel->novatel_msg_header.gps_time_status   = GPS_TIME_STATUS[msg_sdk.gps_time_status];
    msg_ros_novatel->novatel_msg_header.gps_week_num      = msg_sdk.gps_week_num;
    msg_ros_novatel->novatel_msg_header.gps_seconds       = msg_sdk.gps_seconds;
    integrated_navigation::GetNovatelReceiverStatusMessage(msg_sdk.receiver_status,
                                                           msg_ros_novatel->novatel_msg_header.receiver_status);
    msg_ros_novatel->novatel_msg_header.receiver_software_version = msg_sdk.receiver_software_version;

    msg_ros_novatel->gps_seconds1 = msg_sdk.gps_seconds1;
    msg_ros_novatel->gps_seconds2 = msg_sdk.gps_seconds2;
    msg_ros_novatel->gps_seconds3 = msg_sdk.gps_seconds3;

    // Position
    msg_ros_novatel->latitude               = msg_sdk.latitude;
    msg_ros_novatel->longitude              = msg_sdk.longitude;
    msg_ros_novatel->altitude               = msg_sdk.altitude;
    msg_ros_novatel->undulation             = msg_sdk.undulation;
    msg_ros_novatel->lon_sigma              = msg_sdk.lon_sigma;
    msg_ros_novatel->lat_sigma              = msg_sdk.lat_sigma;
    msg_ros_novatel->height_sigma           = msg_sdk.height_sigma;
    msg_ros_novatel->position_covariance[0] = msg_sdk.lon_sigma * msg_sdk.lon_sigma;
    msg_ros_novatel->position_covariance[4] = msg_sdk.lat_sigma * msg_sdk.lat_sigma;
    msg_ros_novatel->position_covariance[8] = msg_sdk.height_sigma * msg_sdk.height_sigma;

    // Velocity
    msg_ros_novatel->x_vel                  = msg_sdk.x_vel;
    msg_ros_novatel->y_vel                  = msg_sdk.y_vel;
    msg_ros_novatel->z_vel                  = msg_sdk.z_vel;
    msg_ros_novatel->v_latency              = msg_sdk.v_latency;
    msg_ros_novatel->x_vel_sigma            = msg_sdk.x_vel_sigma;
    msg_ros_novatel->y_vel_sigma            = msg_sdk.y_vel_sigma;
    msg_ros_novatel->z_vel_sigma            = msg_sdk.z_vel_sigma;
    msg_ros_novatel->velocity_covariance[0] = msg_sdk.x_vel_sigma * msg_sdk.x_vel_sigma;
    msg_ros_novatel->velocity_covariance[4] = msg_sdk.y_vel_sigma * msg_sdk.y_vel_sigma;
    msg_ros_novatel->velocity_covariance[8] = msg_sdk.z_vel_sigma * msg_sdk.z_vel_sigma;

    // Heading
    msg_ros_novatel->heading            = msg_sdk.heading;
    msg_ros_novatel->pitch              = msg_sdk.pitch;
    msg_ros_novatel->heading_sigma      = msg_sdk.heading_sigma;
    msg_ros_novatel->pitch_sigma        = msg_sdk.pitch_sigma;
    msg_ros_novatel->heading_covariance = msg_sdk.heading_sigma * msg_sdk.heading_sigma;
    msg_ros_novatel->pitch_covariance   = msg_sdk.pitch_sigma * msg_sdk.pitch_sigma;

    // Rover Status
    msg_ros_novatel->solution_status = integrated_navigation::SOLUTION_STATUSES[msg_sdk.solution_status];
    msg_ros_novatel->position_type   = integrated_navigation::POSITION_TYPES[msg_sdk.position_type];
    msg_ros_novatel->xyz_sol_status  = integrated_navigation::SOLUTION_STATUSES[msg_sdk.xyz_sol_status];
    msg_ros_novatel->xyz_type        = integrated_navigation::POSITION_TYPES[msg_sdk.xyz_type];
    msg_ros_novatel->heading_type    = integrated_navigation::POSITION_TYPES[msg_sdk.heading_type];
    msg_ros_novatel->num_satellites1 = msg_sdk.num_satellites1;
    msg_ros_novatel->num_satellites2 = msg_sdk.num_satellites2;
    msg_ros_novatel->board_type      = msg_sdk.board_type;  // "482" "718D" "NONE"

    // Radio Status
    msg_ros_novatel->diff_age        = msg_sdk.diff_age;
    msg_ros_novatel->solution_age    = msg_sdk.solution_age;
    msg_ros_novatel->baseline_length = msg_sdk.baseline_length;

    // Ros Custom Msg Tail
    integrated_navigation::GetExtendedSolutionStatusMessage(msg_sdk.extended_solution_status,
                                                            msg_ros_novatel->extended_solution_status);
    integrated_navigation::GetSignalsUsed(msg_sdk.signal_mask, msg_ros_novatel->signal_mask);

    return msg_ros_novatel;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceRtk : public DeviceBasic {
   public:
    bool OnInit() override {
        ros_pub_rtk_info_.Init(ros_handle_, "/Novatel/PosVelHeadingData", 1);
        sdk_sub_rtk_info_.Init(sdk_handle_, CMD_SET_RTK_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);

        sdk_sub_rtk_info_.register_topic_callback([this](const std::shared_ptr<winnie_base::cmd_rtk_info>& msg) {
            auto msg_imu = MessageConverter::to_ros_msg(*msg);
            ros_pub_rtk_info_.publish(msg_imu);
        });

        return true;
    }

   private:
    RosPublisherHelper<navit_msgs::NovatelPosVelHeading> ros_pub_rtk_info_;

    SdkSubscriberHelper<cmd_rtk_info> sdk_sub_rtk_info_;
};

}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceRtk, winnie_base::DeviceBase)
