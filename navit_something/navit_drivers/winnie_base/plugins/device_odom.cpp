//
// Created by fan on 23-6-25.
//

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static nav_msgs::Odometry::Ptr to_ros_msg(const winnie_base::cmd_odom_info& msg_sdk,
                                          const ros::Time& current_time = ros::Time::now()) {
    // odom
    nav_msgs::Odometry::Ptr msg_ros_odom(new nav_msgs::Odometry());

    msg_ros_odom->header.stamp    = current_time;
    msg_ros_odom->header.frame_id = "odom";
    msg_ros_odom->child_frame_id  = "base_link";

    // Set position and orientation
    msg_ros_odom->pose.pose.position.x    = msg_sdk.odom_pose_position[0];
    msg_ros_odom->pose.pose.position.y    = msg_sdk.odom_pose_position[1];
    msg_ros_odom->pose.pose.position.z    = msg_sdk.odom_pose_position[2];
    msg_ros_odom->pose.pose.orientation.x = msg_sdk.odom_pose_orientation[0];
    msg_ros_odom->pose.pose.orientation.y = msg_sdk.odom_pose_orientation[1];
    msg_ros_odom->pose.pose.orientation.z = msg_sdk.odom_pose_orientation[2];
    msg_ros_odom->pose.pose.orientation.w = msg_sdk.odom_pose_orientation[3];
    // Set linear and angular velocities
    msg_ros_odom->twist.twist.linear.x  = msg_sdk.odom_twist_linear[0];
    msg_ros_odom->twist.twist.linear.y  = msg_sdk.odom_twist_linear[1];
    msg_ros_odom->twist.twist.linear.z  = msg_sdk.odom_twist_linear[2];
    msg_ros_odom->twist.twist.angular.x = msg_sdk.odom_twist_angular[0];
    msg_ros_odom->twist.twist.angular.y = msg_sdk.odom_twist_angular[1];
    msg_ros_odom->twist.twist.angular.z = msg_sdk.odom_twist_angular[2];
    return msg_ros_odom;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceOdom : public DeviceBasic {
   public:
    bool OnInit() override {
        ros_pub_odom_info_.Init(ros_handle_, "/odom", 1);
        sdk_sub_odom_info_.Init(sdk_handle_, CMD_SET_ODOM_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);

        sdk_sub_odom_info_.register_topic_callback([this](const std::shared_ptr<winnie_base::cmd_odom_info>& msg) {
            auto current_time = ros::Time::now();

            auto msg_odom = MessageConverter::to_ros_msg(*msg, current_time);
            ros_pub_odom_info_.publish(msg_odom);

            // tf
            static geometry_msgs::TransformStamped odom_tf;
            odom_tf.header.stamp            = current_time;
            odom_tf.header.frame_id         = "odom";
            odom_tf.child_frame_id          = "base_link";
            odom_tf.transform.translation.x = msg_odom->pose.pose.position.x;
            odom_tf.transform.translation.y = msg_odom->pose.pose.position.y;
            odom_tf.transform.translation.z = 0.0;
            odom_tf.transform.rotation      = msg_odom->pose.pose.orientation;
            tf_broadcaster_.sendTransform(odom_tf);
        });

        return true;
    }

   private:
    RosPublisherHelper<nav_msgs::Odometry> ros_pub_odom_info_;

    SdkSubscriberHelper<cmd_odom_info> sdk_sub_odom_info_;

    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceOdom, winnie_base::DeviceBase)
