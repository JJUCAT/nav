#include "navit_bt_navigator/j110/navigate_compute_coverage_path_action_server.hpp"

namespace navit_bt_navigator
{
void NavigateComputeCoveragePathActionServer::executeCB(const GoalTypeConstPtr& goal)
{

  double server_timeout_;

  auto blackboard = BT::Blackboard::create();

  blackboard->set<ros::NodeHandle>("node", nh_);
  blackboard->set<double>("server_timeout", server_timeout_);

  blackboard->set<geometry_msgs::Polygon>("coverage_area", goal->coverage_area);
  blackboard->set<std::vector<geometry_msgs::Polygon>>("holes", goal->holes);
  blackboard->set<geometry_msgs::Pose>("start", goal->start);
  blackboard->set<geometry_msgs::Pose>("end", goal->end);
  blackboard->set<uint16_t>("working_scene", goal->working_scene);
  blackboard->set<float>("sweep_distance", goal->sweep_distance);

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
      blackboard->get("coverage_paths", result_.coverage_paths);
      blackboard->get("contour_paths", result_.contour_paths);
      blackboard->get("error_code", result_.error_code);
      blackboard->get("error_msg", result_.error_msg);
      blackboard->get("is_path_found", result_.is_path_found);
      blackboard->get("length_path", result_.length_path);
      blackboard->get("area", result_.area);
      blackboard->get("planning_duration", result_.planning_duration);
      as_.setSucceeded(result_, "Goal path is finished.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }
    //TODO(czk): feedback
    feedback_.status_code = blackboard->get("status_code", feedback_.status_code);
    feedback_.status_msg = blackboard->get("status_msg", feedback_.status_msg);
    feedback_.planning_progress = blackboard->get("planning_progress", feedback_.planning_progress);

    as_.publishFeedback(feedback_);

    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "navigate_compute_coverage_path_server");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::NavigateComputeCoveragePathActionServer navigate_compute_coverage_path_server(nh, xml_string);
  ros::spin();
  return 0;
}
