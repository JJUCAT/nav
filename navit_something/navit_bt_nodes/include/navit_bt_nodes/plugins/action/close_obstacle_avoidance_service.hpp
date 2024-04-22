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

#include <std_srvs/Empty.h>
#include "navit_bt_nodes/bt_service_node.h"
#include <navit_msgs/ToggleAvoidance.h>


namespace navit_bt_nodes
{

class CloseObstacleAvoidanceService : public BT::RosServiceNode<navit_msgs::ToggleAvoidance>
{
public:

  CloseObstacleAvoidanceService(ros::NodeHandle& handle, const std::string& node_name, const BT::NodeConfiguration& conf)
    : RosServiceNode(handle, node_name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
        BT::InputPort<bool>("use_avoidance", "switch obstacles avoidance mode"),
        BT::InputPort<double>("server_timeout", "server_timeout"),
        BT::InputPort<std::string>("service_name", "service_name"),
    };
  }

  bool prepareRequest() override
  {
    request_.use_avoidance = getInput<bool>("use_avoidance").value();
    ROS_INFO ("prepareRequest: use_avoidance = %d", request_.use_avoidance);
    return true;
  }

  BT::NodeStatus onResponse(const ResponseType& rep) override
  {
    if (rep.success == true) {
      ROS_INFO("avoidance Response status is true");
      return BT::NodeStatus::SUCCESS;
    }
    else {
      ROS_ERROR("avoidance Response status is false");
      return BT::NodeStatus::FAILURE;
    }
  }

};
}


#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__PATH_TO_POINT_ACTION_HPP_
