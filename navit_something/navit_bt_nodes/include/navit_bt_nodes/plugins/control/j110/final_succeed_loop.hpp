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

#ifndef NAVIT_BT_NODES__PLUGINS__CONTROL__FINAL_SUCCEED_LOOP_HPP_
#define NAVIT_BT_NODES__PLUGINS__CONTROL__FINAL_SUCCEED_LOOP_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"
#include <ros/ros.h>

namespace navit_bt_nodes
{
/**
 * @brief FinalSucceedLoop 该节点的成功由最后一个子节点返回，失败由其他子节点返回
 *
 * - 最后一个子节点返回成功，FinalSucceedLoop 返回成功，最后一个子节点返回失败，会重新 tick 第一个子节点
 *
 * - 其他子节点返回失败，FinalSucceedLoop 返回失败，其他子节点返回成功，继续 tick 后面子节点
 *
 * - 子节点返回 RUNNING，会继续 tick 该子节点
 */
class FinalSucceedLoop : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::FinalSucceedLoop
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  FinalSucceedLoop(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for nav2_behavior_tree::FinalSucceedLoop
   */
  ~FinalSucceedLoop() override = default;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<double>("timeout", "node control time limit."),};
  }

private:
  const char* node_name_ = "final_succeed_loop";
  unsigned int current_child_idx_;
  bool check_timeout_{false};
  ros::Time start_time_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;


  /**
   * @brief  是否超时
   * @param  timeout  超时限制
   * @return true 
   * @return false 
   */
  bool Timeout(const double timeout);

  /**
   * @brief  重置计时器，不能放在 halt() !!!
   */
  void ResetTimer();

};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__CONTROL__FINAL_SUCCEED_LOOP_HPP_
