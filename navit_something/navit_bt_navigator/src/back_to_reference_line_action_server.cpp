#include "navit_bt_navigator/back_to_reference_line_action_server.h"
//#include <fstream>

namespace navit_bt_navigator
{
static const std::string default_xml_string = R"(
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <Control ID="RecoveryNode" number_of_retries="8">
                <Sequence>
                    <Decorator ID="GetPositionInFrame" current_pos="{current_pos}" frame_id="map">
                        <Delay delay_msec="3000">
                            <Decorator ID="SelectNearestPose" current_pos="{current_pos}" reference_path="{reference_line}" step_length="1.0" target_point="{target_point}">
                                <Decorator ID="GoalUpdater" can_be_preempted="false" goal_updated="{goal_update}" input_goal="{target_point}" is_docking="true" output_goal="{output_goal}">
                                    <Action ID="ComputePathToPose" goal="{output_goal}" path="{replan_path}" planner_id="topoint" server_name="/compute_path" server_timeout="0.5" start="false"/>
                                </Decorator>
                            </Decorator>
                        </Delay>
                    </Decorator>
                </Sequence>
                <Sequence>
                    <Action ID="ClearEntireCostmap" server_timeout="0.5" service_name="/move_base/clear_local_costmaps"/>
                    <Action ID="ClearEntireCostmap" server_timeout="0.5" service_name="clear_controller_costmap"/>
                </Sequence>
            </Control>
            <Action ID="FollowPath" controller_id="path_follower" name="返回参考线控制器" path="{replan_path}" server_name="/follow_path" server_timeout="0.2"/>
        </Sequence>
    </BehaviorTree>
    <!-- ////////// -->
    <TreeNodesModel>
        <Action ID="ApproachDock">
            <input_port name="controller_plugin_name">...</input_port>
            <input_port name="expected_dock_pose">for perception</input_port>
            <input_port default="ekf_1" name="filter_plugin_name">...</input_port>
            <output_port name="goal_reached">Approach DONE</output_port>
            <input_port name="perception_plugin_name">...</input_port>
            <input_port name="pre_charging_pose">from current pose (usually should be pre_dock_pose)</input_port>
            <input_port default="/approach_dock" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Action ID="BackUp">
            <input_port default="0.25" name="backup_dist">Distance to backup</input_port>
            <input_port default="0.05" name="backup_speed">Speed at which to backup</input_port>
            <input_port default="/backup" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Action ID="ClearEntireCostmap">
            <input_port default="0.5" name="server_timeout">Service name</input_port>
            <input_port default="clear_planner_costmap" name="service_name">Service name</input_port>
        </Action>
        <Action ID="ComputePathToPose">
            <input_port name="goal">Destination to plan to</input_port>
            <output_port name="path">Path created by ComputePathToPose node</output_port>
            <input_port name="planner_id"/>
            <input_port default="/compute_path" name="server_name">remapped action server name.</input_port>
            <input_port name="server_timeout">...</input_port>
            <input_port name="start">Start pose of the path if overriding current robot pose</input_port>
        </Action>
        <Action ID="FinalDock">
            <output_port name="dock_finished">Dock DONE</output_port>
            <input_port name="dock_pose">Final dock pose</input_port>
            <input_port name="rotate_in_place">rotate_in_place</input_port>
            <input_port default="/final_dock" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Action ID="FollowPath">
            <input_port name="controller_id"/>
            <input_port name="path">Path to follow</input_port>
            <input_port default="/follow_path" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Decorator ID="GetPositionInFrame">
            <output_port name="current_pos">Robot current position</output_port>
            <input_port default="map" name="frame_id">Target frame id</input_port>
        </Decorator>
        <Action ID="GetRoutePath">
            <input_port name="distance_interval">The distance interval to gen path.</input_port>
            <output_port name="end">The end station of the route.</output_port>
            <input_port name="route">The route such as LM1, LM2, LM3, LM9.</input_port>
            <output_port name="route_path">The path of the route.</output_port>
            <input_port name="server_timeout">...</input_port>
            <input_port default="/get_map_route_path" name="service_name">remapped action server name</input_port>
            <output_port name="start">The start station of the route.</output_port>
        </Action>
        <Action ID="GetStationPose">
            <input_port name="server_timeout">...</input_port>
            <input_port default="/get_map_station_pose" name="service_name">remapped action server name</input_port>
            <input_port name="station">station name or id.</input_port>
            <output_port name="station_pose">The pose of the station.</output_port>
        </Action>
        <Condition ID="GoalUpdated">
            <input_port name="goal_updated">Is goal updated</input_port>
        </Condition>
        <Decorator ID="GoalUpdater">
            <input_port name="can_be_preempted">is preempted?</input_port>
            <output_port name="goal_updated">whether goal is updated</output_port>
            <input_port name="input_goal">Original goal in</input_port>
            <input_port name="is_docking">is docking?</input_port>
            <output_port name="output_goal">Output goal set by subscription</output_port>
        </Decorator>
        <Condition ID="IsBatteryLow">
            <input_port name="battery_topic">Topic for battery info</input_port>
            <input_port name="is_voltage">Bool if check based on voltage or total %</input_port>
            <input_port name="min_battery">Min battery % or voltage before triggering</input_port>
        </Condition>
        <Control ID="PipelineSequence"/>
        <Action ID="PrintBlackboardValue">
            <input_port name="target_point">Current position of the agent</input_port>
        </Action>
        <Decorator ID="RateController">
            <input_port name="hz">Rate</input_port>
        </Decorator>
        <Control ID="RecoveryNode">
            <input_port name="number_of_retries">Number of retries</input_port>
        </Control>
        <Action ID="RemovePassedGoals">
            <input_port name="input_goals">Original goals to remove viapoints from</input_port>
            <input_port name="input_path">Original Global Planner Path</input_port>
            <output_port name="output_goals">Goals with passed viapoints removed(truncated goals)</output_port>
            <input_port default="0.2" name="radius">radius to goal for it to be considered for removal</input_port>
        </Action>
        <Action ID="Rotate">
            <input_port default="true" name="is_rotate_clockwise">Rotate direction defalt is clockwise</input_port>
            <input_port default="0.1" name="rotate_rad">Rotate value in rad</input_port>
            <input_port default="0.1" name="rotate_speed">Rotate speed in rad/s</input_port>
            <input_port default="/rotate" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
        </Action>
        <Decorator ID="SelectNearestPose">
            <input_port name="current_pos">Current position of the agent</input_port>
            <input_port name="reference_path">Reference path</input_port>
            <input_port default="0.2" name="step_length">Step length to the next target point</input_port>
            <output_port name="target_point">Selected target point in the reference path</output_port>
        </Decorator>
        <Action ID="TruncatePath">
            <input_port name="distance">Distance before goal to truncate</input_port>
            <input_port name="input_path">Path to truncate</input_port>
            <output_port name="output_path">Truncated path to utilize</output_port>
        </Action>
        <Decorator ID="ViaPoints">
            <output_port name="output_path">following via points path</output_port>
            <input_port default="/via_points" name="topic_name">via points topic name</input_port>
        </Decorator>
        <Action ID="Wait">
            <input_port default="/wait" name="server_name">remapped action server name</input_port>
            <input_port name="server_timeout">...</input_port>
            <input_port name="wait_duration">Wait time</input_port>
        </Action>
        <Decorator ID="WayPoints">
            <input_port default="true" name="accept_new_goal">True for new goal receive</input_port>
            <input_port default="false" name="goal_without_loop">True for new goal to stop all way-points LOOP; false                for restart remain loop            </input_port>
            <input_port name="input_goal">Original goal in</input_port>
            <input_port default="false" name="jump_goal">True for new received simple goal will replace current goal in                loop; false for following current goal in loop after arrived simple goal            </input_port>
            <input_port default="0" name="max_loop">max loop number, zero or negative means inf.</input_port>
            <output_port name="output_goal">Output goal set by subscription</output_port>
            <input_port default="{path}" name="path">Global path</input_port>
            <input_port default="move_base_simple/goal" name="simple_goal_topic">simple goal name</input_port>
            <input_port default="way_points" name="way_points_topic">way points topic name</input_port>
        </Decorator>
    </TreeNodesModel>
    <!-- ////////// -->
</root>
)";

void BackToReferenceLineActionServer::executeCB(const GoalTypeConstPtr& goal)
{
  std::string xml_string;
  if (goal->bt_name.empty())
  {
    ROS_INFO("Goal bt name is empty.");
    xml_string = default_xml_string_.empty() ? default_xml_string : default_xml_string_;
  }
  else
  {
    // std::ifstream fin(goal->bt_name);
    // if (!fin.good())
    // {
    //   result_.error_code = 999;
    //   result_.error_msg = "Aborting for bt_name file is not exists.";
    //   as_.setAborted(result_, result_.error_msg);
    //   return;
    // }
    // xml_string = std::string((std::istreambuf_iterator<char>(fin)), std::istreambuf_iterator<char>());
  }
  double server_timeout_;
  double bt_loop_duration_;

  auto blackboard = BT::Blackboard::create();

  blackboard->set<ros::NodeHandle>("node", nh_);
  blackboard->set<double>("server_timeout", server_timeout_);
  blackboard->set<double>("bt_loop_duration", bt_loop_duration_);

  blackboard->set<nav_msgs::Path>("reference_line", goal->reference_line);

  NavitBtRunner navit_bt_runner("back_to_reference_line");
  if (!navit_bt_runner.init(xml_string, "", blackboard))
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
      as_.setSucceeded(result_, "Goal reached.");
      return;
    }

    if (as_.isPreemptRequested())
    {
      as_.setPreempted();
      return;
    }

    blackboard->get("num_of_revoveries", feedback_.num_of_revoveries);
    blackboard->get("time_remaining", feedback_.time_remaining);
    as_.publishFeedback(feedback_);

    r.sleep();
  }
}
};  // namespace navit_bt_navigator

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "back_to_reference_line");
  std::string xml_string;
  ros::NodeHandle nh;
  navit_bt_navigator::BackToReferenceLineActionServer back_to_reference_line_server(nh, xml_string);
  ros::spin();
  return 0;
}
