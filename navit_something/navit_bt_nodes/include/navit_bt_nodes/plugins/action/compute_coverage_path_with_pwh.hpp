#ifndef NAVIT_BT_NODES__PLUGINS__ACTION_COMPUTE_COVERAGE_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION_COMPUTE_COVERAGE_PATH_ACTION_HPP_

#include <string>
#include <geometry_msgs/Polygon.h>
#include <navit_msgs/CoveragePathOnPWHAction.h>
#include "navit_bt_nodes/bt_action_node.h"

namespace navit_bt_nodes
{

class ComputeCoveragePathAction : public BT::RosActionNode<navit_msgs::CoveragePathOnPWHAction>
{
public:
  ComputeCoveragePathAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::CoveragePathOnPWHAction>(handle, name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<geometry_msgs::Polygon>("coverage_area", "Coverage area to start coverage."),
        BT::InputPort<std::vector<geometry_msgs::Polygon>>("holes", "Hole to start coverage."),
        BT::InputPort<geometry_msgs::Pose>("start", "Start pose to start coverage."),
        BT::InputPort<geometry_msgs::Pose>("end", "End pose to start coverage."),
        BT::InputPort<std::string>("plugin_name", "End pose to start coverage."),

        BT::OutputPort<bool>("is_path_found", "Path found."),
        BT::OutputPort<uint16_t>("error_code", "Error code."),
        BT::OutputPort<std::string>("error_msg", "Error msg."),
        BT::OutputPort<std::vector<nav_msgs::Path>>("coverage_paths", "Coverage path."),
        BT::OutputPort<std::vector<nav_msgs::Path>>("contour_paths", "Coverage path."),
        BT::OutputPort<float>("length_path", "length path."),
        BT::OutputPort<float>("area", "area square."),
        BT::OutputPort<float>("planning_duration", "planning_duration."),
    });
  }

  bool on_first_tick() override
  {
    getInput("start", goal_.start);
    getInput("end", goal_.end);
    getInput("coverage_area", goal_.coverage_area);
    getInput("holes", goal_.holes);
    getInput("plugin_name", goal_.plugin_name);
    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
    bool is_path_found = res.is_path_found;
    setOutput("is_path_found", is_path_found);
    setOutput("error_code", res.error_code);
    setOutput("error_msg", res.error_msg);
    setOutput("coverage_paths", res.coverage_paths);
    setOutput("contour_paths", res.contour_paths);
    setOutput("length_path", res.length_path);
    setOutput("area", res.area);
    setOutput("planning_duration", res.planning_duration);
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION_COMPUTE_COVERAGE_PATH_ACTION_HPP_
