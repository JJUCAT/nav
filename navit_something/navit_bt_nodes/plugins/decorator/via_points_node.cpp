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

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/decorator_node.h"

#include "navit_bt_nodes/plugins/decorator/via_points_node.hpp"

#include <ros/ros.h>

namespace navit_bt_nodes
{

ViaPoints::ViaPoints(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<ros::NodeHandle>("node");

  std::string remapped_topic_name;
  getInput("topic_name", remapped_topic_name);
  if (topic_name_ != remapped_topic_name){
    topic_name_ = remapped_topic_name;
    ROS_WARN("Default topic name remapped by [%s]", remapped_topic_name.c_str());
  }

  via_points_sub_ = node_.subscribe<nav_msgs::Path>(topic_name_, 1, &ViaPoints::callback_via_points, this);

  ROS_INFO("via points node construction.");
}

inline BT::NodeStatus ViaPoints::tick()
{
  ros::spinOnce();

  while (!via_points_enable_)
  {
    ros::spinOnce();
    ros::Duration(1.0).sleep();
    ROS_WARN("No via points msg received");
  }
  

  if (via_points_enable_ && !new_via_points_.poses.empty()) {
    // ROS_INFO("via points ENABLE");
    via_points_enable_ = false;
    last_path_ = new_via_points_;
  }

  setOutput("output_path", new_via_points_);
  return child_node_->executeTick();
}

void ViaPoints::callback_via_points(const nav_msgs::Path::ConstPtr& msg)
{
  new_via_points_ = *msg;
  via_points_enable_ = true;
  // ROS_ERROR("via points got a new goal! ");
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::ViaPoints>("ViaPoints");
}
