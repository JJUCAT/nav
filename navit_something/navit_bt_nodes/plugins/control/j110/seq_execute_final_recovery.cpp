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
#include "navit_bt_nodes/plugins/control/j110/seq_execute_final_recovery.hpp"
#include <ros/ros.h>

namespace navit_bt_nodes
{

SeqExecuteFinalRecovery::SeqExecuteFinalRecovery(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0)
{
}

BT::NodeStatus SeqExecuteFinalRecovery::tick()
{
  const unsigned int children_count = children_nodes_.size();
  if (children_count < 2) {
    ROS_ERROR("[SEFR] children count less than 2 !");
    halt();
    return BT::NodeStatus::FAILURE;
  }

  const unsigned int final_act_idx = children_count-2;
  const unsigned int recovery_idx = children_count-1;
  setStatus(BT::NodeStatus::RUNNING);
  // ROS_INFO("[BT][%s] children count %u, final act %u, recovery idx %u",
  //   node_name_, children_count, final_act_idx, recovery_idx);

  while (current_child_idx_ < children_count) {
    // ROS_INFO("[BT][%s] tick child %u", node_name_, current_child_idx_);
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();
    // ROS_INFO("[BT][%s] child [%u] state %u", node_name_, current_child_idx_, (unsigned int)child_status);

    switch (child_status) {
      case BT::NodeStatus::SUCCESS:
        if (current_child_idx_ == final_act_idx) {
          halt();
          return BT::NodeStatus::SUCCESS;
        } else if (current_child_idx_ == recovery_idx) {
          halt();
          return BT::NodeStatus::FAILURE;
        } else {
          current_child_idx_ ++;
        }
        break;

      case BT::NodeStatus::FAILURE:
        if (current_child_idx_ != recovery_idx) {
          current_child_idx_ = recovery_idx;
        } else {
          halt();
          return BT::NodeStatus::FAILURE;
        }
        break;

      case BT::NodeStatus::RUNNING:
        return BT::NodeStatus::RUNNING;

      default:
        throw BT::LogicError("A child node must never return IDLE");

    }  // end switch

    if (current_child_idx_ >= children_count) {
      current_child_idx_ = 0;
    }
  }  // end while loop

  halt();
  return BT::NodeStatus::FAILURE;
}

void SeqExecuteFinalRecovery::halt()
{
  ControlNode::haltChildren();
  ControlNode::halt();
  current_child_idx_ = 0;
}


}  // namespace navit_bt_nodes

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::SeqExecuteFinalRecovery>("SeqExecuteFinalRecovery");
}
