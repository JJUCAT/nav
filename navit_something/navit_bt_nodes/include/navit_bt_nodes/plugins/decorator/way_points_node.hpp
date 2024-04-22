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

#ifndef NAVIT_BT_NODES__PLUGINS__DECORATOR__WAY_POINTS_NODE_HPP_
#define NAVIT_BT_NODES__PLUGINS__DECORATOR__WAY_POINTS_NODE_HPP_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Path.h>

#include "navit_bt_nodes/bt_conversions.hpp"
#include "behaviortree_cpp_v3/decorator_node.h"

namespace navit_bt_nodes
{

/**
 * @brief A BT::DecoratorNode that subscribes to a goal topic and updates
 * the current goal on the blackboard
 */
class WayPoints : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for navit_bt_nodes::WayPoints
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  WayPoints(
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
      BT::InputPort<std::string>("simple_goal_topic", "simple goal name"),
      BT::InputPort<std::string>("way_points_topic", "way points topic name"),
      BT::InputPort<int>("max_loop", "max loop number, zero or negative means inf."),
      BT::InputPort<bool>("jump_goal", "True for new received simple goal will replace current goal in loop; false for following current goal in loop after arrived simple goal"),
      BT::InputPort<bool>("accept_new_goal", "True for new goal receive"),
      BT::InputPort<bool>("goal_without_loop", "True for new goal to stop all way-points LOOP; false for restart remain loop"),
      BT::InputPort<nav_msgs::Path>("path", "Global path"),
      BT::OutputPort<geometry_msgs::PoseStamped>("output_goal", "Received Goal by subscription"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  void passedGoal();

  void initLoop();

  /**
   * @brief Callback function for goal update topic
   * @param msg Shared pointer to geometry_msgs::msg::PoseStamped message
   */
  void callback_updated_goal(const geometry_msgs::PoseStamped::ConstPtr& msg);

  void callback_way_points(const nav_msgs::Path::ConstPtr& msg);

  ros::NodeHandle node_;
  ros::Subscriber goal_sub_, way_points_sub_;
  ros::Time node_init_time = ros::Time::now();

  geometry_msgs::PoseStamped new_goal_received_, last_goal_received_, input_goal_, last_output_goal_;
  nav_msgs::Path new_way_points_, path_;

  std::string simple_goal_topic_{"move_base_simple/goal"};
  std::string way_points_topic_{"way_points"};
  int max_loop_{0};
  bool jump_goal_{false};
  bool accept_new_goal_{true};
  bool goal_without_loop_{false};
  
  bool init_goal_enable_{true};
  bool received_goal_enable_{false};
  bool way_points_enable_{false};

  int loop_index_{0};
  int goal_index_{0};
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__DECORATOR__WAY_POINTS_NODE_HPP_
