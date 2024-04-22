#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__GET_DRIVEABLE_AREA_ID_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__GET_DRIVEABLE_AREA_ID_ACTION_HPP_

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{
class GetDriveableAreaId : public BT::ActionNodeBase
{
public:
    GetDriveableAreaId(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {}

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
            BT::InputPort<geometry_msgs::PoseStamped>("current_pos", "robot_current_pose"),
            BT::OutputPort<std::string>("current_area_id", "current_area_id"),
            BT::OutputPort<navit::protocol::map_info::MapArea>("multi_task_polygon", "tmulti_task_polygon"),
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

        geometry_msgs::PoseStamped current_pos;
        if (!getInput<geometry_msgs::PoseStamped>("current_pos", current_pos)) {
            NAVIT_ROS_ERROR_STREAM("current_pos is not specified");
            return BT::NodeStatus::FAILURE;
        }

        // 
        for (int i = 0; i < task_polygons.size(); i++) {
            navit::protocol::map_info::MapArea task_polygon = task_polygons[i];
            if (task_polygon.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
                Point boost_point(current_pos.pose.position.x, current_pos.pose.position.y);
                Polygon boost_polygon;
                for (const auto& pt : task_polygon.path()) {
                    boost::geometry::append(boost_polygon, Point(pt.x(), pt.y()));
                }
         
                if (navit_common::geometry::isPoseInPolygon(boost_point, boost_polygon)) {
                    setOutput("current_area_id", task_polygon.name());
                    setOutput("multi_task_polygon", task_polygon);
                    return BT::NodeStatus::SUCCESS;
                }
            }
        }
        NAVIT_ROS_ERROR_STREAM("GetDriveableAreaId failed");
        return BT::NodeStatus::FAILURE;
    };
};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__GET_DRIVEABLE_AREA_ID_ACTION_HPP_
