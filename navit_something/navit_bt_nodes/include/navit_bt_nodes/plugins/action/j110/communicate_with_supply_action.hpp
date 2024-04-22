#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__COMMUNICATE_WITH_SUPPLY_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__COMMUNICATE_WITH_SUPPLY_HPP_

#include <string>

#include <app_chassis_control_msgs/SupplyCtrlAction.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

namespace navit_bt_nodes
{

class CommunicateWithSupply : public BT::RosActionNode<app_chassis_control_msgs::SupplyCtrlAction>
{
public:
    CommunicateWithSupply(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
                            : RosActionNode<app_chassis_control_msgs::SupplyCtrlAction>(handle, name, conf) {
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<uint8_t>("ctrl_cmd", "ctrl_cmd 0 is stop, 1 is start"),
        });
    }

    bool on_first_tick() override {

        uint8_t ctrl_cmd = 0;
        getInput("ctrl_cmd", ctrl_cmd);
        ROS_INFO("ctrl_cmd: %d", ctrl_cmd);
        goal_.ctrl = ctrl_cmd;
        return true;
    }

    BT::NodeStatus onResult(const ResultType& res) override {
        if (res.succeed == false) {
            return BT::NodeStatus::FAILURE;
        }
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__COMMUNICATE_WITH_SUPPLY_HPP_
