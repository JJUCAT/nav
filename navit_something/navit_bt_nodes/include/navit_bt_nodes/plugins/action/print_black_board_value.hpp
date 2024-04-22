#ifndef PRINT_BLACKBOARD_VALUE_H
#define PRINT_BLACKBOARD_VALUE_H

#include "behaviortree_cpp_v3/action_node.h"
#include <behaviortree_cpp_v3/blackboard.h>
#include <geometry_msgs/PoseStamped.h>
namespace navit_bt_nodes
{

class PrintBlackboardValue : public BT::ActionNodeBase
{
public:
    PrintBlackboardValue(const std::string &name, const BT::NodeConfiguration &config)
        : BT::ActionNodeBase(name, config)
    {
    }

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;

      void halt() override {}
};

} // namespace navit_bt_nodes

#endif // PRINT_BLACKBOARD_VALUE_H