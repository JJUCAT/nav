#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__RESET_VALUE_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__RESET_VALUE_HPP_

#include <string>
#include <geometry_msgs/Pose.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{
class ResetValueAction : public BT::ActionNodeBase
{
public:
    ResetValueAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<int>("task_nums", "task_nums"),
            BT::InputPort<int>("reset_value", "reset_value"),
        };
    }

private:
    void halt() override
    {
    }

    BT::NodeStatus tick() override {
        int task_nums;
        if (!getInput<int>("task_nums", task_nums)) {
            NAVIT_ROS_ERROR_STREAM("task_nums is not specified");
            return BT::NodeStatus::FAILURE;
        }
        int reset_value;
        if (!getInput<int>("reset_value", reset_value)) {
            NAVIT_ROS_ERROR_STREAM("reset_value is not specified");
            return BT::NodeStatus::FAILURE;
        }
        task_nums = reset_value;
        setOutput<int>("task_nums", task_nums);

        return BT::NodeStatus::SUCCESS;
    };
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__RESET_VALUE_HPP_
