#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__FINAL_DOCK_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__FINAL_DOCK_ACTION_HPP_

#include <string>

#include <navit_msgs/FinalDockAction.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

namespace navit_bt_nodes
{

class FinalDockAction : public BT::RosActionNode<navit_msgs::FinalDockAction>
{
public:
  FinalDockAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::FinalDockAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<bool>("dock_finished", "Approach DONE"),
        BT::InputPort<bool>("rotate_in_place", "rotate_in_place"),
        BT::InputPort<geometry_msgs::PoseStamped>("dock_pose", "Final dock pose"),
    });
  }

  bool on_first_tick() override
  {
    // ROS_INFO("FinalDock ticking.");
    getInput("dock_pose", goal_.dock_pose);
    std::string rotate_in_place_string;
    getInput("rotate_in_place", rotate_in_place_string);
    if (rotate_in_place_string == "true" || rotate_in_place_string == "True")
    {
      goal_.rotate_in_place = true;
    }
    else
    {
      goal_.rotate_in_place = false;
    }
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    setOutput("dock_finished", res.docked);
    // ROS_WARN("FinalDock SUCCESS);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__FINAL_DOCK_ACTION_HPP_
