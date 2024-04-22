// Copyright (c) 2018 Intel Corporation
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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__COVERAGE_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__COVERAGE_PATH_ACTION_HPP_

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <vector>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <navit_msgs/CoveragePathAction.h>

#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

// # 起点
// geometry_msgs/PoseStamped start
//
// # 包围的边界
// nav_msgs/Path edge_path
//
// # 选择插件
// string planner_plugin
//
//---
// # 是否成功
// bool path_found
//
// # 规划结果
// nav_msgs/Path coverage_path
// # nav_msgs/Path wall_path

class CoveragePathAction : public BT::RosActionNode<navit_msgs::CoveragePathAction>
{
public:
  CoveragePathAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::CoveragePathAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<bool>("path_found", "Path found."),
        BT::OutputPort<nav_msgs::Path>("path", "Path out coverage path."),
        BT::InputPort<geometry_msgs::PoseStamped>("start_pose", "Start pose to start coverage."),
        BT::InputPort<nav_msgs::Path>("edge_path", " coverage path edge."),
        BT::InputPort<std::string>("planner_id", "planner_plugin"),
    });
  }

  bool on_first_tick() override
  {
    getInput("start_pose", goal_.start);
    getInput("edge_path", goal_.edge_path);
    getInput("planner_id", goal_.planner_plugin);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    nav_msgs::Path fixed_path;
    geometry_msgs::PoseStamped poses;
    for (int i = 0; i < res.coverage_path.poses.size(); ++i)
    {
      geometry_msgs::Pose pose;
      pose.position.x = res.coverage_path.poses[i].pose.position.x;
      pose.position.y = res.coverage_path.poses[i].pose.position.y;
      pose.orientation = res.coverage_path.poses[i].pose.orientation;
      poses.pose = pose;
      poses.header.frame_id = "map";
      fixed_path.poses.push_back(poses);
    }
    fixed_path.header.frame_id = "map";

    setOutput("path_found", true);
    setOutput("path", fixed_path);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__COVERAGE_PATH_ACTION_HPP_
