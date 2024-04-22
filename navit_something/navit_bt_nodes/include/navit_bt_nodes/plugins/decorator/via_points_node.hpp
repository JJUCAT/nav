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

#ifndef NAVIT_BT_NODES__PLUGINS__DECORATOR__VIA_POINTS_NODE_HPP_
#define NAVIT_BT_NODES__PLUGINS__DECORATOR__VIA_POINTS_NODE_HPP_

#include <memory>
#include <string>

#include <ros/ros.h>
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
class ViaPoints : public BT::DecoratorNode
{
public:
  /**
   * @brief A constructor for navit_bt_nodes::ViaPoints
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  ViaPoints(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("topic_name", "via points topic name"),
      BT::OutputPort<nav_msgs::Path>("output_path", "following via points path"),
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  void callback_via_points(const nav_msgs::Path::ConstPtr& msg);

  ros::NodeHandle node_;
  ros::Subscriber via_points_sub_;
  bool via_points_enable_{false};
  nav_msgs::Path new_via_points_, last_path_;

  std::string topic_name_{"/via_points"};
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__DECORATOR__VIA_POINTS_NODE_HPP_
