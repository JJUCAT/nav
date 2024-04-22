#ifndef NEM_NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
#define NEM_NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_

#include <string>

#include <navit_msgs/ComputePathAction.h>
#include <nav_msgs/Path.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class NemComputePathToPoseAction : public BT::RosActionNode<navit_msgs::ComputePathAction>
{
public:
  NemComputePathToPoseAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::ComputePathAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::OutputPort<nav_msgs::Path>("path", "Path created by ComputePathToPose node"),
        BT::InputPort<geometry_msgs::PoseStamped>("goal", "Destination to plan to"),
        BT::InputPort<geometry_msgs::PoseStamped>("start", "Start pose of the path if overriding current robot pose"),
        BT::InputPort<std::string>("planner_id"),
        BT::InputPort<uint64_t>("closest_id"),
    });
  }

  bool on_first_tick() override
  {
    // ROS_INFO("ComputePath ticking.");
    getInput("closest_id", closest_id_);
    getInput("goal", goal_.goal);
    getInput("planner_id", goal_.planner_plugin);
    
    goal_.goal.header.seq = closest_id_;

    if (getInput("start", goal_.start))
    {
      goal_.use_start = true;
    }
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    setOutput("path", res.path);
    // ROS_WARN("ComputePath SUCCESS: result size: [%d]", result_->path.poses.size());

    if (first_time_)
    {
      first_time_ = false;
    }
    else
    {
      config().blackboard->set("path_updated", true);
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  bool first_time_{ true };
  uint64_t closest_id_;
};

}  // namespace navit_bt_nodes

#endif  //NEM_NAVIT_BT_NODES__PLUGINS__ACTION__COMPUTE_PATH_TO_POSE_ACTION_HPP_
