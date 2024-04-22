#ifndef GET_PATH_FIRST_POINT_ACTION_H_
#define GET_PATH_FIRST_POINT_ACTION_H_

#include <string>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

namespace navit_bt_nodes
{

class GetPathFirstPointAction : public BT::ActionNodeBase
{
public:
    GetPathFirstPointAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<nav_msgs::Path>("follow_path_ros"),
            BT::OutputPort<geometry_msgs::PoseStamped>("task_path_first_pose"),
            BT::OutputPort<geometry_msgs::PoseStamped>("task_path_last_pose"),
            BT::OutputPort<nav_msgs::Path>("inverse_multi_teaching_path_ros"),
        };
    }

private:
    void halt() override
    {}

    BT::NodeStatus tick() override {
        nav_msgs::Path follow_path_ros;
        if (!getInput<nav_msgs::Path>("follow_path_ros", follow_path_ros)) {
            return BT::NodeStatus::FAILURE;
        }
        if (follow_path_ros.poses.size() == 0) {
            return BT::NodeStatus::FAILURE;
        }
        geometry_msgs::PoseStamped task_path_first_pose, task_path_last_pose;

        if (follow_path_ros.poses.size() != 0) {
            task_path_first_pose = follow_path_ros.poses[0];
            task_path_last_pose = follow_path_ros.poses[follow_path_ros.poses.size() - 1];
            task_path_first_pose.header.frame_id = "map";
            task_path_last_pose.header.frame_id = "map";

            setOutput<geometry_msgs::PoseStamped>("task_path_first_pose", task_path_first_pose);
            setOutput<geometry_msgs::PoseStamped>("task_path_last_pose", task_path_last_pose);

            nav_msgs::Path inverse_multi_teaching_path_ros;
            inverse_multi_teaching_path_ros.header.frame_id = follow_path_ros.header.frame_id;

            for (int i = follow_path_ros.poses.size() - 1; i >= 0; i--) {
                inverse_multi_teaching_path_ros.poses.push_back(follow_path_ros.poses[i]);
            }

            setOutput("inverse_multi_teaching_path_ros", follow_path_ros);
        }
    
        return BT::NodeStatus::SUCCESS;
    };

};

}  // namespace navit_bt_nodes
#endif  // GET_PATH_FIRST_POINT_ACTION_H_
