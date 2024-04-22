// Copyright (c) 2020 Sarthak Mittal
// Copyright (c) 2019 Intel Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAVIT_BT_NODES__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
#define NAVIT_BT_NODES__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "ros/ros.h"
#include "sensor_msgs/BatteryState.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
class IsBatteryLowCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for navit_bt_nodes::IsBatteryLowCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  IsBatteryLowCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  IsBatteryLowCondition() = delete;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<double>("min_battery", "Minimum battery percentage/voltage"),
      BT::InputPort<std::string>(
        "battery_topic", std::string("/battery_status"), "Battery topic"),
      BT::InputPort<bool>(
        "is_voltage", false, "If true voltage will be used to check for low battery, otherwise percentage will be used"),
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */
  void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg);

  ros::NodeHandle node_;
  ros::Subscriber battery_sub_;
  std::string battery_topic_;
  double min_battery_;
  bool is_voltage_;
  bool is_battery_low_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__CONDITION__IS_BATTERY_LOW_CONDITION_HPP_
