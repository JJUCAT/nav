#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__HANDLE_VIA_POINTS_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__HANDLE_VIA_POINTS_HPP_

#include <string>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>
#include <navit_common/path_handle.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{
class HandleViaPoints : public BT::ActionNodeBase
{
public:
    HandleViaPoints(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
            BT::InputPort<std::vector<navit::protocol::map_info::MapLine>>("task_paths", "paths"),
            BT::InputPort<std::vector<std::string>>("via_indexes", "via indexes"),

            BT::OutputPort<std::vector<std::string>>("via_indexes_finished", "via indexes finished"),
            BT::OutputPort<nav_msgs::Path>("multi_teaching_path_ros", "multi teaching path ros"),
        };
    }

private:
    void halt() override
    {
    }

    BT::NodeStatus tick() override {
        std::vector<navit::protocol::map_info::MapArea> task_polygons;
        if (!getInput<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", task_polygons)) {
            NAVIT_ROS_ERROR_STREAM("task_polygons is not specified");
            return BT::NodeStatus::FAILURE;
        }

        std::vector<navit::protocol::map_info::MapLine> task_paths;
        if (!getInput<std::vector<navit::protocol::map_info::MapLine>>("task_paths", task_paths)) {
            NAVIT_ROS_ERROR_STREAM("task_paths is not specified");
            return BT::NodeStatus::FAILURE;
        }
        
        if (!getInput<std::vector<std::string>>("via_indexes", via_indexes_finished_)) {
            NAVIT_ROS_ERROR_STREAM("via_indexes is not specified");
            return BT::NodeStatus::FAILURE;
        } 
        NAVIT_ROS_INFO_STREAM("via_indexes_finished_ size: " << via_indexes_finished_.size());

        // convert task_paths to ros path
        nav_msgs::Path multi_teaching_path_ros;
        multi_teaching_path_ros.header.frame_id = "map";
        multi_teaching_path_ros.header.stamp = ros::Time::now();
        nav_msgs::Path path_ros;

        for (auto& task_path : task_paths) {
            path_ros.header.frame_id = "map";
            path_ros.header.stamp = ros::Time::now();
            for (auto& point : task_path.path()) {
                geometry_msgs::PoseStamped pose;
                pose.header.frame_id = "map";
                pose.header.stamp = ros::Time::now();
                pose.pose.position.x = point.x();
                pose.pose.position.y = point.y();
                pose.pose.position.z = 0;               
                path_ros.poses.push_back(pose);
            }
        }
        
        navit_common::checkPath(path_ros);
        setOutput("multi_teaching_path_ros", path_ros);
         
        if (via_indexes_finished_.size() > 0){
            via_indexes_finished_.pop_back();
            int size = via_indexes_finished_.size();
            finished_index_++;
            NAVIT_ROS_INFO_STREAM("via_indexes_finished_: " << finished_index_);
            setOutput("via_indexes_finished", via_indexes_finished_);
            return BT::NodeStatus::SUCCESS;
        }
        
 
        return BT::NodeStatus::FAILURE;
    };
    std::vector<std::string> via_indexes_finished_;
    int finished_index_ = 0;

};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__HANDLE_VIA_POINTS_HPP_
