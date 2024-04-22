#include "navit_bt_nodes/plugins/action/j110/task_plan_reporter_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navit_bt_nodes::TaskPlanReporterAction>("TaskPlanReporterAction");
}
