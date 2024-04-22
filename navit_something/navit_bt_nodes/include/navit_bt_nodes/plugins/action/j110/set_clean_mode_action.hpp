#ifndef SET_CLEAN_MODE_ACTION_H_
#define SET_CLEAN_MODE_ACTION_H_


#include <app_chassis_control_msgs/SwitchCleanModeAction.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"


namespace navit_bt_nodes
{

class SetCleanModeAction : public BT::RosActionNode<app_chassis_control_msgs::SwitchCleanModeAction>
{
public:
    SetCleanModeAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
                            : RosActionNode<app_chassis_control_msgs::SwitchCleanModeAction>(handle, name, conf) {
    }

    static BT::PortsList providedPorts() {
        return providedBasicPorts({
            BT::InputPort<int>("clean_mode", "clean mode: 0: off, 1: on"),
        });
    }

    bool on_first_tick() override {
        int temp_clean_mode = 0;
        getInput("clean_mode", temp_clean_mode);
        ROS_INFO("SwitchCleanModeAction: %d", temp_clean_mode);

        goal_.clean_mode = temp_clean_mode;
        ROS_INFO("goal_.clean_mode: %d", goal_.clean_mode);
        return true;
    }

    BT::NodeStatus onResult(const ResultType& res) override {
        ROS_INFO("SwitchCleanModeAction SUCCESS");
        return BT::NodeStatus::SUCCESS;
    }
};

}  // namespace navit_bt_nodes
# endif