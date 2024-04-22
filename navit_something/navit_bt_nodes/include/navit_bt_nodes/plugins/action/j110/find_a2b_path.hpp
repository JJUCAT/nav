#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__FIND_A2B_PATH_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__FIND_A2B_PATH_ACTION_HPP_

#include <string>
#include <geometry_msgs/Pose.h>
#include <navit_common/geometry_algorithms.h>

#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"

#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{
class FindA2BPath : public BT::ActionNodeBase
{
public:
    FindA2BPath(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::InputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
            BT::InputPort<std::vector<navit::protocol::map_info::MapLine>>("task_paths", "paths"),
            BT::InputPort<std::string>("current_area_id", "current_area_id"),
            BT::InputPort<std::vector<std::string>>("taget_area_ids", "taget_area_ids")
        };
    }

private:
    void halt() override
    {
    }

    BT::NodeStatus tick() override {

    };

};

}  // namespace navit_bt_nodes
#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__FIND_A2B_PATH_ACTION_HPP_
