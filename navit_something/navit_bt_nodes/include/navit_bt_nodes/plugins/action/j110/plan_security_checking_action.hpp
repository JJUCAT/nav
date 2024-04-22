#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__PLAN_SECURITY_CHECKING_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__PLAN_SECURITY_CHECKING_ACTION_HPP_

#include <ros/ros.h>
#include <ros/package.h>

#include <string>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <nav_msgs/Path.h>

#include <navit_msgs/PlanSecurityCheckingAction.h>

#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class PlanSecurityCheckingAction : public BT::RosActionNode<navit_msgs::PlanSecurityCheckingAction>
{
public:
  PlanSecurityCheckingAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::PlanSecurityCheckingAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<size_t>("start", "Start on Plan for checking."),
        BT::InputPort<nav_msgs::Path>("plan", "Plan for checking."),
        BT::InputPort<double>("check_distance", "Distance for checing."),
        BT::OutputPort<bool>("collised", "Whether collised."),
        BT::OutputPort<std::vector<uint32_t>>("collised_indexes", "Indexes collised."),
    });
  }

  bool on_first_tick() override
  {
    getInput("start", goal_.start);
    getInput("plan", goal_.plan);
    getInput("check_distance", goal_.check_distance);
    return true;
  }

  bool on_every_tick() override
  {
    getInput("start", goal_.start);
    getInput("plan", goal_.plan);
    getInput("check_distance", goal_.check_distance);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    ROS_INFO("[BT][PSC] plan %s, collised indexes size is %lu",
      res.collised?"collised":"safe", res.collised_indexes.size());
    bool collised = res.collised ? true : false;
    std::vector<uint32_t> collised_indexes = res.collised_indexes;
    setOutput("collised", collised);
    setOutput("collised_indexes", collised_indexes);
    BT::NodeStatus status = res.collised ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
    return status;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__PLAN_SECURITY_CHECKING_ACTION_HPP_
