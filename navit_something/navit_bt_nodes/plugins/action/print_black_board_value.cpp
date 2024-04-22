#include "navit_bt_nodes/plugins/action/print_black_board_value.hpp"

namespace navit_bt_nodes
{

BT::PortsList PrintBlackboardValue::providedPorts()
{
    return {BT::InputPort<geometry_msgs::Pose>("target_point", "Selected target point")};
}

BT::NodeStatus PrintBlackboardValue::tick()
{   
    geometry_msgs::Pose current_pose;
    if (!getInput<geometry_msgs::Pose>("target_point", current_pose)) {
        return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
}

} // namespace navit_bt_nodes
#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder = [](const std::string &name, const BT::NodeConfiguration &config) {
        return std::make_unique<navit_bt_nodes::PrintBlackboardValue>(name, config);
    };

    factory.registerBuilder<navit_bt_nodes::PrintBlackboardValue>("PrintBlackboardValue", builder);
}