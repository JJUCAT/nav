
#include "navit_bt_nodes/plugins/action/compute_coverage_path_with_pwh.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    static ros::NodeHandle nh;
    config.blackboard->template get<ros::NodeHandle>("node", nh);
    return std::make_unique<navit_bt_nodes::ComputeCoveragePathAction>(nh, name, config);
  };

  factory.registerBuilder<navit_bt_nodes::ComputeCoveragePathAction>("ComputeCoveragePathAction", builder);
}
