//
// Created by fan on 23-6-25.
//

#include <geometry_msgs/Twist.h>

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static winnie_base::cmd_cmd_vel to_sdk_msg(const geometry_msgs::Twist& msg_ros) {
    winnie_base::cmd_cmd_vel msg_sdk;
    msg_sdk.linear[0]  = msg_ros.linear.x;
    msg_sdk.linear[1]  = msg_ros.linear.y;
    msg_sdk.linear[2]  = msg_ros.linear.z;
    msg_sdk.angular[0] = msg_ros.angular.x;
    msg_sdk.angular[1] = msg_ros.angular.y;
    msg_sdk.angular[2] = msg_ros.angular.z;
    return msg_sdk;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceAgv : public DeviceBasic {
   public:
    typedef struct {
        float wheel_radius;     // 轮径
        float wheel_distance;   // 轮距
        float reduction_ratio;  // 减速比
    } AgvParam;

   private:
    bool OnInit() override {
        ros_handle_->param("wheel_radius", agv_param_.wheel_radius, 0.09f);
        ros_handle_->param("wheel_distance", agv_param_.wheel_distance, 0.493f);
        ros_handle_->param("reduction_ratio", agv_param_.reduction_ratio, 15.0f);

        ros_sub_cmd_vel_.Init(ros_handle_, "/cmd_vel", 1);
        sdk_pub_cmd_vel_.Init(sdk_handle_, CMD_SET_CMD_VEL, 1, JETSON_ADDRESS, WINNIE_ADDRESS);
        sdk_pub_param_set_.Init(sdk_handle_, CMD_SET_PARAM_SET, 1, JETSON_ADDRESS, WINNIE_ADDRESS);

        ros_sub_cmd_vel_.register_topic_callback([this](const geometry_msgs::Twist::ConstPtr& msg) {
            auto sdk_msg = MessageConverter::to_sdk_msg(*msg);
            sdk_pub_cmd_vel_.publish(sdk_msg);
        });

        return true;
    }

    void SpinOnce() override {
        if (!param_set_succeed_) {
            param_set_succeed_ = SendParamSet();
        }
    }

    bool SendParamSet() {
        sdk_pub_param_set_.publish(agv_param_);
        return true;
    }

   private:
    bool param_set_succeed_{};
    winnie_base::cmd_param_set agv_param_{};

    RosSubscriberHelper<geometry_msgs::Twist> ros_sub_cmd_vel_;

    SdkPublisherHelper<winnie_base::cmd_cmd_vel> sdk_pub_cmd_vel_;
    SdkPublisherHelper<winnie_base::cmd_param_set> sdk_pub_param_set_;
};
}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceAgv, winnie_base::DeviceBase)
