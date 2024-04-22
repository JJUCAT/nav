#include "navit_bt_navigator/j110/navigate_auto_dock_action_server.hpp"

namespace navit_bt_navigator
{
void NavigateAutoDockActionServer::executeCB(const GoalTypeConstPtr& goal)
{

  double server_timeout_;
  double bt_loop_duration_;

  auto blackboard = BT::Blackboard::create();
  blackboard->set<ros::NodeHandle>("node", nh_);
  blackboard->set<double>("server_timeout", server_timeout_);

  blackboard->set<uint16_t>("robot_model", goal->robot_model);
  blackboard->set<geometry_msgs::PoseStamped>("expected_dock_pose", goal->expected_dock_pose);

  NavitBtRunner navit_bt_runner("navigator_through_path");

  if (!navit_bt_runner.init(default_xml_string, "", blackboard))
  {
    result_.error_code = 999;
    result_.error_msg = "Aborting for bt_tree init error!";
    as_.setAborted(result_, result_.error_msg);
    return;
  }

  ros::Rate r(100);
  while (ros::ok())
  {
    auto node_status = navit_bt_runner.tickRoot();
    if (node_status == BT::NodeStatus::FAILURE)
    {
      result_.error_code = 999;
      result_.error_msg = "Aborting on the goal because the BT node_status is FAILURE";
      as_.setAborted(result_, result_.error_msg);
      return;
    }

    if (node_status == BT::NodeStatus::SUCCESS)
    {
      result_.error_code = 0;
      result_.error_msg = "Docked successfully.";

      as_.setSucceeded(result_, "Goal path is finished.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }
    //TODO(czk): feedback
    feedback_.status_code =
    blackboard->get("dock_pose", feedback_.dock_pose);
    blackboard->get("target_pose", feedback_.target_pose);
    blackboard->get("command", feedback_.command);
    blackboard->get("status_code", feedback_.status_code);
    blackboard->get("status_msg", feedback_.status_msg);

    as_.publishFeedback(feedback_);

    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navigate_auto_dock_action_server");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::NavigateAutoDockActionServer navigate_to_pose_action_server(nh, xml_string);
  ros::spin();
  return 0;
}
