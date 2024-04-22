//
// Created by fan on 23-1-30.
//

#ifndef NAVIT_BT_NAVIGATOR_NAVIT_BT_RUNNER_H
#define NAVIT_BT_NAVIGATOR_NAVIT_BT_RUNNER_H

#include <memory>
#include <vector>

#include "behaviortree_cpp_v3/blackboard.h"

namespace navit_bt_navigator
{
class NavitBtRunner
{
public:
  explicit NavitBtRunner(const std::string& name);

  ~NavitBtRunner();

  bool init(const std::string& xml_string, const std::string& log_path,
            BT::Blackboard::Ptr blackboard = BT::Blackboard::create());

  BT::NodeStatus tickRoot();

  void stop();

private:
  class Impl;
  std::shared_ptr<Impl> impl_;
};
}  // namespace navit_bt_navigator

#endif  // NAVIT_BT_NAVIGATOR_NAVIT_BT_RUNNER_H
