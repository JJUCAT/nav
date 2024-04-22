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

#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__MOVE_TO_POSE_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__MOVE_TO_POSE_ACTION_HPP_

#include <string>

#include <navit_msgs/MoveThroughPosesAction.h>
#include <nav_msgs/Path.h>
#include "navit_bt_nodes/bt_action_node.h"
#include <boost/algorithm/string.hpp>

namespace navit_bt_nodes
{
// # MoveThroughPoses.action
// uint64[] poses #路点编号
// string bt_name #行为树名 optional, 含有默认行为树
//---
// int16 error_code #错误码
// string error_msg #错误信息
//---
// uint64 current_pose
// int16 num_of_poses_remaining
// int16 num_of_revoveries #重试次数
// float32 distance_remaining  #剩余距离
// std_msgs/Duration time_elapsed   #已用时间
// std_msgs/Duration time_remaining  #剩余时间

class MoveThroughPoseAction : public BT::RosActionNode<navit_msgs::MoveThroughPosesAction>
{
public:
  MoveThroughPoseAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::MoveThroughPosesAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({ BT::InputPort<std::string>("poses", "The poses split by comma[,]."),
                                BT::InputPort<std::string>("bt_name", "optional get goal_ids in blackboard.") });
  }

  bool on_first_tick() override
  {
    // ROS_INFO("ComputePath ticking.");

    std::string poses;
    getInput("poses", poses);
    getInput("bt_name", goal_.bt_name);

    std::vector<std::string> poses_str;
    boost::split(poses_str, poses, boost::is_any_of(","), boost::token_compress_on);
    if (poses_str.size() < 2)
    {
      return false;
    }

    std::vector<uint64_t> poses_int;
    for (auto& w : poses_str)
    {
      poses_int.push_back(stol(w));
    }
    goal_.poses = poses_int;
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif
