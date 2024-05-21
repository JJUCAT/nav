//
// Created by fan on 23-6-25.
//

#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static sensor_msgs::Imu::Ptr to_ros_msg(const winnie_base::cmd_imu_raw_info& msg_sdk,
                                        const ros::Time& current_time = ros::Time::now()) {
    sensor_msgs::Imu::Ptr msg_ros_imu(new sensor_msgs::Imu);

    msg_ros_imu->header.stamp          = current_time;
    msg_ros_imu->header.frame_id       = "imu_link";
    msg_ros_imu->linear_acceleration.x = msg_sdk.imu_a[0];
    msg_ros_imu->linear_acceleration.y = msg_sdk.imu_a[1];
    msg_ros_imu->linear_acceleration.z = msg_sdk.imu_a[2];
    msg_ros_imu->angular_velocity.x    = msg_sdk.imu_g[0];
    msg_ros_imu->angular_velocity.y    = msg_sdk.imu_g[1];
    msg_ros_imu->angular_velocity.z    = msg_sdk.imu_g[2];

#if 0
    // 将欧拉角转换为四元数
    tf2::Quaternion quaternion;
    quaternion.setRPY(msg_sdk.imu_rpy[0], msg_sdk.imu_rpy[1], msg_sdk.imu_rpy[2]);
    msg_ros_imu->orientation.x = quaternion.x();
    msg_ros_imu->orientation.y = quaternion.y();
    msg_ros_imu->orientation.z = quaternion.z();
    msg_ros_imu->orientation.w = quaternion.w();
#else
    msg_ros_imu->orientation.x = msg_sdk.imu_q[1];
    msg_ros_imu->orientation.y = msg_sdk.imu_q[2];
    msg_ros_imu->orientation.z = msg_sdk.imu_q[3];
    msg_ros_imu->orientation.w = msg_sdk.imu_q[0];
#endif
    return msg_ros_imu;
}

static sensor_msgs::Imu::Ptr to_ros_msg(const winnie_base::cmd_imu_info& msg_sdk,
                                        const ros::Time& current_time = ros::Time::now()) {
    sensor_msgs::Imu::Ptr msg_ros_imu(new sensor_msgs::Imu());

    msg_ros_imu->header.stamp    = current_time;
    msg_ros_imu->header.frame_id = "imu_link";
    // Set orientation
    msg_ros_imu->orientation.x = msg_sdk.imu_orientation[0];
    msg_ros_imu->orientation.y = msg_sdk.imu_orientation[1];
    msg_ros_imu->orientation.z = msg_sdk.imu_orientation[2];
    msg_ros_imu->orientation.w = msg_sdk.imu_orientation[3];
    // Set angular velocity
    msg_ros_imu->angular_velocity.x = msg_sdk.imu_angular_velocity[0];
    msg_ros_imu->angular_velocity.y = msg_sdk.imu_angular_velocity[1];
    msg_ros_imu->angular_velocity.z = msg_sdk.imu_angular_velocity[2];
    // Set linear acceleration
    msg_ros_imu->linear_acceleration.x = msg_sdk.imu_linear_acceleration[0];
    msg_ros_imu->linear_acceleration.y = msg_sdk.imu_linear_acceleration[1];
    msg_ros_imu->linear_acceleration.z = msg_sdk.imu_linear_acceleration[2];
    return msg_ros_imu;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceImu : public DeviceBasic {
   public:
    bool OnInit() override {
        ros_pub_imu_info_.Init(ros_handle_, "/imu/data_raw", 1);
        sdk_sub_imu_info_.Init(sdk_handle_, CMD_SET_IMU_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);

        sdk_sub_imu_info_.register_topic_callback([this](const std::shared_ptr<winnie_base::cmd_imu_raw_info>& msg) {
            auto msg_imu = MessageConverter::to_ros_msg(*msg);
            ros_pub_imu_info_.publish(msg_imu);
        });

        return true;
    }

   private:
    RosPublisherHelper<sensor_msgs::Imu> ros_pub_imu_info_;

    SdkSubscriberHelper<cmd_imu_raw_info> sdk_sub_imu_info_;
};

}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceImu, winnie_base::DeviceBase)
