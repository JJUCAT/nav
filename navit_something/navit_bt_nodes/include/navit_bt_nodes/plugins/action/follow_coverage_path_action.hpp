#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_COVERAGE_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_COVERAGE_PATH_ACTION_HPP_

#include <string>

#include <navit_msgs/FollowCoveragePathAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class FollowCoveragePathAction : public BT::RosActionNode<navit_msgs::FollowCoveragePathAction>
{
public:
  FollowCoveragePathAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::FollowCoveragePathAction>(handle, name, conf)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<nav_msgs::Path>("path", "Path to follow"),
        BT::InputPort<std::string>("controller_id", ""),
        BT::OutputPort<std::vector<nav_msgs::Path>>("cleaned_paths", "Cleaned paths"),
        BT::OutputPort<std::vector<nav_msgs::Path>>("uncleaned_paths", "Uncleaned paths"),
    });
  }

  bool on_first_tick() override
  {
    getInput("path", goal_.path);
    getInput("controller_id", goal_.plugin_name);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    if (res.error_code == navit_msgs::FollowCoveragePathResult::OK) {
        setOutput("cleaned_paths", res.cleaned_paths);
        setOutput("uncleaned_paths", res.uncleaned_paths);
        return BT::NodeStatus::SUCCESS;
    }
  }
};
}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__FOLLOW_COVERAGE_PATH_ACTION_HPP_
