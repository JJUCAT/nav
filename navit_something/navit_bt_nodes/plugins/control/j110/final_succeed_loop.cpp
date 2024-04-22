// Copyright (c) 2019 Intel Corporation
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
#include "navit_bt_nodes/plugins/control/j110/final_succeed_loop.hpp"
#include <ros/ros.h>

namespace navit_bt_nodes
{

FinalSucceedLoop::FinalSucceedLoop(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0)
{
}

BT::NodeStatus FinalSucceedLoop::tick()
{
  double timeout = -1.0;
  if (getInput("timeout", timeout)) {
    if (!check_timeout_) start_time_ = ros::Time::now();
    if (timeout > 0) check_timeout_ = true;
    // ROS_INFO("[FSL] check timeout %d, timeout %f", check_timeout_, timeout);
  }

  const unsigned children_count = children_nodes_.size();
  if (children_count < 2) {
    ROS_ERROR("[FSL] children count less than 2 !");
    halt();
    ResetTimer();
    return BT::NodeStatus::FAILURE;
  }

  const unsigned final_idx = children_count-1;
  setStatus(BT::NodeStatus::RUNNING);
  // ROS_INFO("[BT][%s] children count %u", node_name_, children_count);

  while (current_child_idx_ < children_count) {
    if (Timeout(timeout)) {
      ROS_ERROR("[FSL] timeout !");
      halt();
      ResetTimer();
      return BT::NodeStatus::FAILURE;
    }

    // ROS_INFO("[BT][%s] tick child %u", node_name_, current_child_idx_);
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();
    // ROS_INFO("[BT][%s] child [%u] state %u", node_name_, current_child_idx_, (unsigned int)child_status);

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        if (current_child_idx_ == final_idx) {
          halt();
          ResetTimer();
          return BT::NodeStatus::SUCCESS;
        } else {
          current_child_idx_ ++;
        }
        break;

      case BT::NodeStatus::FAILURE:
        if (current_child_idx_ != final_idx) {
          halt();
          ResetTimer();
          return BT::NodeStatus::FAILURE;
        } else {
          current_child_idx_ ++;
        }
        break;

      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      default:
        throw BT::LogicError("A child node must never return IDLE");

    }  // end switch

    if (current_child_idx_ >= children_count) {
      halt();
    }
  }  // end while loop

  halt();
  return BT::NodeStatus::FAILURE;
}

void FinalSucceedLoop::halt()
{
  ControlNode::haltChildren();
  ControlNode::halt();
  current_child_idx_ = 0;
}

bool FinalSucceedLoop::Timeout(const double timeout)
{
  if (check_timeout_) {
    double time_elapse = (ros::Time::now() - start_time_).toSec();
    // ROS_INFO("[FSL] time elapse %f, timeout %f", time_elapse, timeout);
    if (time_elapse >= timeout) return true;
  }
  return false;
}

void FinalSucceedLoop::ResetTimer()
{
  check_timeout_ = false;
}

}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::FinalSucceedLoop>("FinalSucceedLoop");
}
