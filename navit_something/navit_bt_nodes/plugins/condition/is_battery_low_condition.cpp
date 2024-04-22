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

#include <string>

#include "navit_bt_nodes/plugins/condition/is_battery_low_condition.hpp"

namespace navit_bt_nodes
{

IsBatteryLowCondition::IsBatteryLowCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf),
  battery_topic_("/battery_status"),
  min_battery_(0.0),
  is_voltage_(false),
  is_battery_low_(false)
{
  getInput("min_battery", min_battery_);
  getInput("battery_topic", battery_topic_);
  getInput("is_voltage", is_voltage_);
  node_ = config().blackboard->get<ros::NodeHandle>("node");

  battery_sub_ = node_.subscribe<sensor_msgs::BatteryState>(battery_topic_, 1, &IsBatteryLowCondition::batteryCallback, this);
}

BT::NodeStatus IsBatteryLowCondition::tick()
{
  ros::spinOnce();
  if (is_battery_low_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::FAILURE;
}

void IsBatteryLowCondition::batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg)
{
  if (is_voltage_) {
    is_battery_low_ = msg->voltage <= min_battery_;
  } else {
    is_battery_low_ = msg->percentage <= min_battery_;
  }
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::IsBatteryLowCondition>("IsBatteryLow");
}
