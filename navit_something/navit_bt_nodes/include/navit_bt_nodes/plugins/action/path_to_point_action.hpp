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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_PATH_ACTION_HPP_

#include <string>
#include <ros/ros.h>

#include <nav_msgs/Path.h>
#include "navit_bt_nodes/bt_action_node.h"


namespace navit_bt_nodes
{

class PathToPointAction : public BT::ActionNodeBase
{
public:

  PathToPointAction(const std::string& name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(name, conf) {}
  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<nav_msgs::Path>("Controller_path", "Path to follow"),
        BT::OutputPort<nav_msgs::Path>("tracking_path", "Point to follow"),
    };
  }

private:

    void halt() override {}
    BT::NodeStatus tick() override {
      // Get the path from the blackboard
      nav_msgs::Path Controller_path;
      getInput("Controller_path", Controller_path);
      if(Controller_path.poses.empty()){
          ROS_ERROR("Controller_path is empty");
          return BT::NodeStatus::FAILURE;
      }
      // Set the tracking path as the first point of the path
      nav_msgs::Path tracking_path;
      tracking_path.poses.resize(3);
      tracking_path.poses[0] = Controller_path.poses[Controller_path.poses.size()-1];
      tracking_path.poses[1] = Controller_path.poses[Controller_path.poses.size()-2];
      tracking_path.poses[2] = Controller_path.poses[Controller_path.poses.size()-3];
      // std::cout << "tracking_path: " << tracking_path << std::endl;
      setOutput<nav_msgs::Path>("tracking_path", tracking_path);
      if(tracking_path.poses.empty()){
          ROS_ERROR("tracking_path is empty");
          return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::SUCCESS;
    };
};
};
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__PATH_TO_POINT_ACTION_HPP_
