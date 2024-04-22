#ifndef NAVIT_BT_NODES__PLUGINS__INVERSE_PATH_HPP_
#define NAVIT_BT_NODES__PLUGINS__INVERSE_PATH_HPP_

#include <string>
#include <nav_msgs/Path.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>
#include <navit_common/path_handle.h>
#include <tf/tf.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{
class InversePath : public BT::ActionNodeBase
{
public:
    InversePath(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<nav_msgs::Path>("multi_teaching_path_ros", "multi_teaching_path_ros"),
            BT::OutputPort<nav_msgs::Path>("inverse_multi_teaching_path_ros", "inverse_multi_teaching_path_ros"),
        };
    }

private:
    void halt() override
    {
    }

    BT::NodeStatus tick() override {
        nav_msgs::Path multi_teaching_path_ros;
        if (!getInput<nav_msgs::Path>("multi_teaching_path_ros", multi_teaching_path_ros)) {
            NAVIT_ROS_ERROR_STREAM("multi_teaching_path_ros is not specified");
            return BT::NodeStatus::FAILURE;
        }
       
        nav_msgs::Path inverse_multi_teaching_path_ros;
        inverse_multi_teaching_path_ros.header.frame_id = multi_teaching_path_ros.header.frame_id;

        for (int i = multi_teaching_path_ros.poses.size() - 1; i >= 0; i--) {
            inverse_multi_teaching_path_ros.poses.push_back(multi_teaching_path_ros.poses[i]);
        }
        navit_common::checkPath(inverse_multi_teaching_path_ros);
        setOutput("inverse_multi_teaching_path_ros", inverse_multi_teaching_path_ros);
        return BT::NodeStatus::SUCCESS;
    };
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__INVERSE_PATH_HPP_
