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

#include "navit_bt_nodes/plugins/decorator/way_points_node.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace navit_bt_nodes
{

WayPoints::WayPoints(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<ros::NodeHandle>("node");

  getInput("input_goal", input_goal_);
  getInput("max_loop", max_loop_);
  getInput("jump_goal", jump_goal_);
  getInput("accept_new_goal", accept_new_goal_);
  getInput("goal_without_loop", goal_without_loop_);  // Do ONCE!  

  std::string simple_goal_topic, way_points_topic;
  getInput("simple_goal_topic", simple_goal_topic);
  getInput("way_points_topic", way_points_topic);
  if (simple_goal_topic != simple_goal_topic_)
    ROS_WARN("simple goal topic remap!"); 
  if (way_points_topic != way_points_topic_)
    ROS_WARN("way points topic remap!"); 

  goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>(simple_goal_topic, 1, &WayPoints::callback_updated_goal, this);
  way_points_sub_ = node_.subscribe<nav_msgs::Path>(way_points_topic, 1, &WayPoints::callback_way_points, this);

  ROS_INFO("Way Points construction.");
}

inline BT::NodeStatus WayPoints::tick()
{
  ros::spinOnce();

  getInput("path", path_);
  bool passed_goal = (path_.poses.size()<=8) ? true : false;
  if (passed_goal && !init_goal_enable_)
  {
    if (!goal_without_loop_)
      way_points_enable_ = true;
    if (!jump_goal_ && received_goal_enable_)
      passed_goal = false;
    ROS_WARN("Way Points Node: current goal passed, aim to next one.");
    ROS_WARN("Way Points Node: current goal passed, aim to next one.");
    ROS_WARN("Way Points Node: current goal passed, aim to next one.");
  }  

  geometry_msgs::PoseStamped output_goal; 
  
  if (init_goal_enable_)
  {
    // ROS_WARN("Way Points Node: set Original Goal.");
    output_goal = input_goal_;
    last_output_goal_ = output_goal;
  }

  if (way_points_enable_ && new_way_points_.poses.size()!=0)
  {
    received_goal_enable_ = false;
    // ROS_INFO("Way Points Node: Way points ENABLE");    
    if (max_loop_ <= 0) // inf loop
    {
      goal_index_ = passed_goal ? goal_index_+1 : goal_index_;      
      output_goal = new_way_points_.poses[goal_index_%new_way_points_.poses.size()]; // caution int upper
      // ROS_INFO("Way Points Node:  inf loop: output goal index: [%d], frame: [%s], position: [%f, %f, %f]",  
      //                     (goal_index_%new_way_points_.poses.size()), output_goal.header.frame_id.c_str(),
      //                     output_goal.pose.position.x, output_goal.pose.position.y, output_goal.pose.position.z);
      last_output_goal_ = output_goal;
    }
    else if (goal_index_/new_way_points_.poses.size() < max_loop_)
    {
      goal_index_ = passed_goal ? goal_index_+1 : goal_index_;
      output_goal = new_way_points_.poses[goal_index_%new_way_points_.poses.size()]; // caution int upper
      // ROS_INFO("Way Points Node: loop:[%d], output goal index: [%d], frame: [%s], position: [%f, %f, %f]",  
      //                     (goal_index_/new_way_points_.poses.size()), 
      //                     (goal_index_%new_way_points_.poses.size()), output_goal.header.frame_id.c_str(),
      //                     output_goal.pose.position.x, output_goal.pose.position.y, output_goal.pose.position.z);
      last_output_goal_ = output_goal;
    }
    else
      output_goal = last_output_goal_;

    setOutput("output_goal", output_goal);
    return child_node_->executeTick();
  }

  if(received_goal_enable_ && accept_new_goal_)
  {
    // ROS_INFO("Way Points Node: New simple goal ENABLE");
    last_goal_received_ = new_goal_received_;
    output_goal = new_goal_received_;
    last_output_goal_ = output_goal;
    if (goal_without_loop_)
      way_points_enable_ = false;

    setOutput("output_goal", output_goal);
    return child_node_->executeTick();
  }

  output_goal = last_output_goal_;  
  // ROS_INFO("Way Points Node:  output goal frame: [%s], position: [%f, %f, %f]",  output_goal.header.frame_id.c_str(),
  //                         output_goal.pose.position.x, output_goal.pose.position.y, output_goal.pose.position.z);
  setOutput("output_goal", output_goal);
  return child_node_->executeTick();
}

void WayPoints::callback_updated_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  new_goal_received_ = *msg;
  received_goal_enable_ = true;
  way_points_enable_ = false; // Notice: run loop until finish this simple goal!
  if (accept_new_goal_)
    init_goal_enable_ = false;
  // ROS_WARN("Way Points Node: Got a new point goal! ");
}

void WayPoints::callback_way_points(const nav_msgs::Path::ConstPtr& msg)
{
  new_way_points_ = *msg;
  initLoop();
  way_points_enable_ = true;
  init_goal_enable_ = false;
  // ROS_WARN("Way Points Node: Got a new way points goal! ");
}

void WayPoints::initLoop()
{
  loop_index_ = 0;
  goal_index_ = 0;
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::WayPoints>("WayPoints");
}
