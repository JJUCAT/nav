#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__APPROACH_DOCK_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__APPROACH_DOCK_ACTION_HPP_

#include <string>

#include <navit_msgs/ApproachDockAction.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

namespace navit_bt_nodes
{

class ApproachDockAction : public BT::RosActionNode<navit_msgs::ApproachDockAction>
{
public:
  ApproachDockAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::ApproachDockAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<bool>("goal_reached", "Approach DONE"),
        BT::OutputPort<geometry_msgs::PoseStamped>("dock_pose", ""),
        BT::InputPort<geometry_msgs::PoseStamped>("expected_dock_pose", "for perception"),
        BT::InputPort<std::string>("controller_plugin_name", ""),
        BT::InputPort<std::string>("perception_plugin_name", ""),
        BT::InputPort<std::string>("filter_plugin_name", ""),
        BT::InputPort<std::string>("percepted_pose_offset", "always > 0, pose offset for perception"),
    });
  }

  bool on_first_tick() override
  {
    getInput("expected_dock_pose", goal_.expected_dock_pose);
    getInput("controller_plugin_name", goal_.controller_plugin_name);
    getInput("perception_plugin_name", goal_.perception_plugin_name);
    getInput("filter_plugin_name", goal_.filter_plugin_name);
    getInput("percepted_pose_offset", goal_.percepted_pose_offset);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    setOutput("goal_reached", res.reached);
    setOutput("dock_pose", res.dock_pose);
    // ROS_WARN("ApproachDockAction SUCCESS);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__APPROACH_DOCK_ACTION_HPP_
