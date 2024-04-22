#include "navit_bt_nodes/plugins/action/move_base/get_current_planner_plugin_service.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    static ros::NodeHandle nh;
    config.blackboard->template get<ros::NodeHandle>("node", nh);
    return std::make_unique<navit_bt_nodes::GetPlannerPluginService>(nh, name, config);
  };

  factory.registerBuilder<navit_bt_nodes::GetPlannerPluginService>("GetPlannerPluginService", builder);
}
