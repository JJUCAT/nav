//
// Created by fan on 23-6-25.
//

#include <sensor_msgs/BatteryState.h>

#include <pluginlib/class_list_macros.hpp>

#include "protocol_winnie.h"
#include "ros_helper.hpp"
#include "sdk_helper.hpp"
#include "winnie_base/device_basic.hpp"

namespace MessageConverter {

static sensor_msgs::BatteryState::Ptr to_ros_msg(const winnie_base::cmd_battery_info& msg_sdk,
                                                 const ros::Time& current_time = ros::Time::now()) {
    sensor_msgs::BatteryState::Ptr msg_ros_battery_state(new sensor_msgs::BatteryState());

    msg_ros_battery_state->header.stamp            = current_time;
    msg_ros_battery_state->voltage                 = msg_sdk.voltage;
    msg_ros_battery_state->temperature             = msg_sdk.temperature;
    msg_ros_battery_state->current                 = msg_sdk.current;
    msg_ros_battery_state->charge                  = msg_sdk.charge;
    msg_ros_battery_state->capacity                = msg_sdk.capacity;
    msg_ros_battery_state->design_capacity         = msg_sdk.design_capacity;
    msg_ros_battery_state->percentage              = msg_sdk.percentage;
    msg_ros_battery_state->power_supply_status     = msg_sdk.power_supply_status;
    msg_ros_battery_state->power_supply_health     = msg_sdk.power_supply_health;
    msg_ros_battery_state->power_supply_technology = msg_sdk.power_supply_technology;
    return msg_ros_battery_state;
}

}  // namespace MessageConverter

namespace winnie_base {

class DeviceBattery : public DeviceBasic {
   public:
    bool OnInit() override {
        ros_pub_battery_state_.Init(ros_handle_, "/battery_state", 1);
        sdk_sub_battery_info_.Init(sdk_handle_, CMD_SET_BATTERY_INFO, 1, WINNIE_ADDRESS, JETSON_ADDRESS);

        sdk_sub_battery_info_.register_topic_callback(
            [this](const std::shared_ptr<winnie_base::cmd_battery_info>& msg) {
                data_battery_info_ = *msg;
                auto msg_battery   = MessageConverter::to_ros_msg(*msg);
                ros_pub_battery_state_.publish(msg_battery);
                // PublishStates();  // 发布 key value 形式的状态信息
            });

        // typedef struct {
        //     float voltage;
        //     float temperature;
        //     float current;
        //     float charge;
        //     float capacity;
        //     float design_capacity;
        //     float percentage;
        //     uint8_t power_supply_status;
        //     uint8_t power_supply_health;
        //     uint8_t power_supply_technology;
        // } cmd_battery_info;
        // AddState("voltage", data_battery_info_.voltage);
        // AddState("temperature", data_battery_info_.temperature);
        // AddState("current", data_battery_info_.current);
        // AddState("charge", data_battery_info_.charge);
        // AddState("capacity", data_battery_info_.capacity);
        // AddState("design_capacity", data_battery_info_.design_capacity);
        // AddState("percentage", data_battery_info_.percentage);
        // AddState("power_supply_status", data_battery_info_.power_supply_status);
        // AddState("power_supply_health", data_battery_info_.power_supply_health);
        // AddState("power_supply_technology", data_battery_info_.power_supply_technology);
        // return DeviceHybrid::OnInit();
        return true;
    }

    void SpinOnce() override {
        bool test = false;
        if (test) {
            /// for_test
            data_battery_info_.voltage     = 100;
            data_battery_info_.temperature = 200;
            data_battery_info_.current     = 300;
            data_battery_info_.capacity    = 4000;
            data_battery_info_.charge++;
            // SimulateReceived(msg_battery);
            auto msg_battery_state = MessageConverter::to_ros_msg(data_battery_info_);
            ros_pub_battery_state_.publish(msg_battery_state);
            // PublishStates();  // 发布 key value 形式的状态信息
        }
    }

   private:
    cmd_battery_info data_battery_info_{};

    RosPublisherHelper<sensor_msgs::BatteryState> ros_pub_battery_state_;

    SdkSubscriberHelper<cmd_battery_info> sdk_sub_battery_info_;
};

}  // namespace winnie_base

PLUGINLIB_EXPORT_CLASS(winnie_base::DeviceBattery, winnie_base::DeviceBase)
