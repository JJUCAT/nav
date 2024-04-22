#ifndef NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H
#define NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H

#include <utility>
#include <fstream>

#include <navit_msgs/NavigateComputeCoveragePathAction.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Polygon.h>

#include "navit_bt_navigator/action_server_base.hpp"
#include "navit_bt_navigator/navit_bt_runner.h"

namespace navit_bt_navigator
{
class NavigateComputeCoveragePathActionServer : public ActionServerBase<navit_msgs::NavigateComputeCoveragePathAction>
{
public:
  inline static const std::string action_name = "/navigate_compute_coverage_path";

  explicit NavigateComputeCoveragePathActionServer(ros::NodeHandle& nh, std::string xml_string = "")
    : ActionServerBase(nh, action_name), default_xml_string_(std::move(xml_string))
  {
  }

protected:
  void executeCB(const GoalTypeConstPtr& goal) override;

private:
  std::string default_xml_string_;
  const std::string default_xml_string = R"(
    <?xml version="1.0"?>
    <root main_tree_to_execute="BehaviorTree">
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <Action ID="ComputeCoveragePathAction" area="{area}" contour_paths="{contour_paths}" coverage_area="{coverage_area}" coverage_paths="{coverage_paths}" end="{end}" error_code="{error_code}" error_msg="{error_msg}" holes="{holes}" is_path_found="{is_path_found}" length_path="{length_path}" planning_duration="{planning_duration}" plugin_name="ccpp" server_name="polygon_coverage_path" server_timeout="1.0" start="{start}"/>
        </Sequence>
    </BehaviorTree>
    </root>
)";
};
}  // namespace navit_bt_navigator
#endif  // NAVIT_BT_NAVIGATOR__NAVIGATE_THROUTH_PATH_ACTION_SERVER_H
