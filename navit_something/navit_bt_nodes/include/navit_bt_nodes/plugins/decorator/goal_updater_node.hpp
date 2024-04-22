// Copyright (c) 2018 Intel Corporation
// Copyright (c) 2020 Francisco Martin Rico
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

#ifndef NAVIT_BT_NODES__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
#define NAVIT_BT_NODES__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "navit_bt_nodes/bt_conversions.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class GoalUpdater : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for navit_bt_nodes::GoalUpdater
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  GoalUpdater(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::PoseStamped>("input_goal", "Original Goal"),
      BT::InputPort<bool>("is_docking", "is docking?"),
      BT::InputPort<bool>("can_be_preempted", "is preempted?"),
      BT::OutputPort<geometry_msgs::PoseStamped>(
        "output_goal",
        "Received Goal by subscription"),
      BT::OutputPort<bool>("goal_updated", "whether goal is updated"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  geometry_msgs::PoseStamped goal; 

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  ros::NodeHandle node_;
  ros::Subscriber goal_sub_;
  ros::Time node_init_time = ros::Time::now();

  geometry_msgs::PoseStamped last_goal_received_;
  
  bool new_goal_enable_{false};
  bool init_goal_enable_{true};
  bool is_docking_{false};
  bool is_preempted_{false};
  bool goal_updated_{false};
  bool can_be_preempted_{false};
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__DECORATOR__GOAL_UPDATER_NODE_HPP_
