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

#ifndef NAVIT_BT_NODES__PLUGINS__CONDITION__NEED_PLAN_CONDITION_HPP_
#define NAVIT_BT_NODES__PLUGINS__CONDITION__NEED_PLAN_CONDITION_HPP_

#include <string>
#include <memory>
#include <mutex>

#include "ros/ros.h"
#include "behaviortree_cpp_v3/condition_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::ConditionNode that listens to a battery topic and
 * returns SUCCESS when battery is low and FAILURE otherwise
 */
class NeedPlanCondition : public BT::ConditionNode
{
public:
  /**
   * @brief A constructor for navit_bt_nodes::NeedPlanCondition
   * @param condition_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  NeedPlanCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  NeedPlanCondition() = delete;

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
      BT::InputPort<bool>(
        "need_plan", false, "Indicated whether need plan"),
    };
  }

private:
  /**
   * @brief Callback function for battery topic
   * @param msg Shared pointer to sensor_msgs::msg::BatteryState message
   */

  ros::NodeHandle node_;
  bool need_plan_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__CONDITION__NEED_PLAN_CONDITION_HPP_
