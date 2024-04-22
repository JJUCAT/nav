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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__WALL_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__WALL_PATH_ACTION_HPP_

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

class WallPathAction : public BT::RosActionNode<navit_msgs::CoveragePathAction>
{
public:
  WallPathAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::CoveragePathAction>(handle, name, conf)
  {
    ros::NodeHandle pnh("~/");
    map_sub_ = node_.subscribe<nav_msgs::OccupancyGrid>("/map", 10, &WallPathAction::mapCallback, this);
    path_pub_ = node_.advertise<nav_msgs::Path>("/fixed_polygon_wall", 100);
    edge_path_pub_ = node_.advertise<nav_msgs::Path>("/edge_path_wall", 100);
    pnh.param("x_1", x_1_, 100.0);
    pnh.param("y_1", y_1_, 100.0);
    pnh.param("x_2", x_2_, -100.0);
    pnh.param("y_2", y_2_, 100.0);
    pnh.param("x_3", x_3_, -100.0);
    pnh.param("y_3", y_3_, -100.0);
    pnh.param("x_4", x_4_, 100.0);
    pnh.param("y_4", y_4_, -100.0);
  }

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& _map_msg)
  {
    ROS_ERROR("call back!!");
    //   goal_.map.header = _map_msg->header;
    //   goal_.map.info = _map_msg->info;
    //   goal_.map.data = _map_msg->data;
    //   curr_map = *_map_msg;
    feedback_ = false;
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<bool>("need_plan", ""),
        BT::OutputPort<nav_msgs::Path>("ipa_path", ""),
    });
  }

  bool on_first_tick() override
  {
    std::vector<geometry_msgs::PoseStamped> poses;
    geometry_msgs::PoseStamped pose_1;
    pose_1.header.frame_id = "map";
    pose_1.pose.position.x = x_1_;
    pose_1.pose.position.y = y_1_;
    poses.push_back(pose_1);
    geometry_msgs::PoseStamped pose_2;
    pose_2.header.frame_id = "map";
    pose_2.pose.position.x = x_2_;
    pose_2.pose.position.y = y_2_;
    poses.push_back(pose_2);
    geometry_msgs::PoseStamped pose_3;
    pose_3.header.frame_id = "map";
    pose_3.pose.position.x = x_3_;
    pose_3.pose.position.y = y_3_;
    poses.push_back(pose_3);
    geometry_msgs::PoseStamped pose_4;
    pose_4.header.frame_id = "map";
    pose_4.pose.position.x = x_4_;
    pose_4.pose.position.y = y_4_;
    poses.push_back(pose_4);
    poses.push_back(pose_1);
    nav_msgs::Path path;
    path.poses = poses;
    path.header.frame_id = "map";
    goal_.edge_path = path;
    goal_.planner_plugin = "ccpp";

    edge_path_pub_.publish(path);
    if (!feedback_)
    {
      goal_updated_ = true;
      feedback_ = true;
      return true;
    }
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    // feedback_ = true;
    ROS_ERROR("On success");
    setOutput("need_plan", false);
    nav_msgs::Path fixed_path;
    geometry_msgs::PoseStamped poses;
    // for (int i = 0; i < res.wall_path.poses.size(); ++i)
    // {
    //   geometry_msgs::Pose pose;
    //   pose.position.x = res.wall_path.poses[i].pose.position.x;
    //   pose.position.y = res.wall_path.poses[i].pose.position.y;
    //   pose.orientation = res.coverage_path.poses[i].pose.orientation;
    //   poses.pose = pose;
    //   poses.header.frame_id = "map";
    //   fixed_path.poses.push_back(poses);
    // }
    fixed_path.header.frame_id = "map";
    path_pub_.publish(fixed_path);
    setOutput("ipa_path", fixed_path);
    return BT::NodeStatus::SUCCESS;
  }

private:
  nav_msgs::OccupancyGrid curr_map;
  ros::Subscriber map_sub_;
  ros::Publisher path_pub_, edge_path_pub_;
  bool feedback_ = true;
  double x_1_, y_1_, x_2_, y_2_, x_3_, y_3_, x_4_, y_4_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__COVERAGE_PATH_ACTION_HPP_
