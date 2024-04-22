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

#include "navit_bt_nodes/plugins/decorator/goal_updater_node.hpp"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

namespace navit_bt_nodes
{

GoalUpdater::GoalUpdater(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::DecoratorNode(name, conf)
{
  node_ = config().blackboard->get<ros::NodeHandle>("node");

  std::string goal_updater_topic;

  getInput("is_docking", is_docking_);
  
  goal_sub_ = node_.subscribe<geometry_msgs::PoseStamped>("move_base_simple/goal", 1, &GoalUpdater::callback_updated_goal, this);

  ROS_INFO("goal updater construction.");
}

inline BT::NodeStatus GoalUpdater::tick()
{

  // ROS_INFO("GoalUpdater set goal: [%f, %f, %f]", 
  //                         goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
  if (is_docking_) {
    getInput("input_goal", goal);
    setOutput("output_goal", goal);
    setOutput("goal_updated", true);
  } else {
    // ros::spinOnce();
    // getInput("goal_updated", goal_updated_);
    getInput("can_be_preempted", can_be_preempted_);
    if (is_preempted_ && can_be_preempted_) {
      is_preempted_ = false;
      goal_updated_ = false;
      return BT::NodeStatus::SUCCESS;
    }
    if(init_goal_enable_){
      return BT::NodeStatus::RUNNING;
    }
  
    if (ros::Time(last_goal_received_.header.stamp) > ros::Time(goal.header.stamp)) {
      ROS_INFO("Goal updater receive a new goal.");
      goal = last_goal_received_;
    }

    setOutput("output_goal", goal);
  }
  
  return child_node_->executeTick();
}

void
GoalUpdater::callback_updated_goal(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  last_goal_received_ = *msg;
  if (!goal_updated_) {
    setOutput("goal_updated", true);
    goal_updated_ = true;
  } else {
    is_preempted_ = true;
  }

  init_goal_enable_ = false;
  ROS_INFO("Goal updater got a new goal! ");
}
}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::GoalUpdater>("GoalUpdater");
}
