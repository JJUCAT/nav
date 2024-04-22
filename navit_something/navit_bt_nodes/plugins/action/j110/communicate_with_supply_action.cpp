
#include "navit_bt_nodes/plugins/action/j110/communicate_with_supply_action.hpp"

BT_REGISTER_NODES(factory)
{
  BT::NodeBuilder builder = [](const std::string& name, const BT::NodeConfiguration& config) {
    static ros::NodeHandle nh;
    config.blackboard->template get<ros::NodeHandle>("node", nh);
    return std::make_unique<navit_bt_nodes::CommunicateWithSupply>(nh, name, config);
  };

  factory.registerBuilder<navit_bt_nodes::CommunicateWithSupply>("CommunicateWithSupplyAction", builder);
}
