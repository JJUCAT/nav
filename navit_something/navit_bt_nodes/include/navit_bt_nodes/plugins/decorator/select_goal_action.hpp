#ifndef SELECT_GOAL_ACTION_H
#define SELECT_GOAL_ACTION_H

#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include "behaviortree_cpp_v3/decorator_node.h"
#include <limits>
#include <cmath>
#include <tf2/LinearMath/Vector3.h>

namespace navit_bt_nodes
{
class SelectGoalAction : public BT::DecoratorNode {
public:
    SelectGoalAction(const std::string& name, const BT::NodeConfiguration& config)
        : BT::DecoratorNode(name, config) {}

    static BT::PortsList providedPorts();
    BT::NodeStatus tick();
    void halt() {
        tick_count_ = 0;
    }
private:
    geometry_msgs::PoseStamped goal_point_;
    uint16_t tick_count_;
};
}
#endif // SELECT_GOAL_ACTION_H