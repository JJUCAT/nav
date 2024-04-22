#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__ROUTING_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__ROUTING_ACTION_HPP_

#include <string>

//inner include
#include <navit_msgs/GraphPointInfo.h>
#include <navit_msgs/GraphPointsConnection.h>
#include <navit_msgs/RoutingAction.h>
#include <navit_common/log.h>
#include <navit_common/geometry_algorithms.h>
#include <memory>
#include "navit_bt_nodes/bt_action_node.h"
#include "message_navit_map.pb.h"
#include "handle_via_points.hpp"

namespace navit_bt_nodes
{

class RoutingAction : public BT::RosActionNode<navit_msgs::RoutingAction>
{
public:
  RoutingAction(ros::NodeHandle& handle, const std::string& name, const BT::NodeConfiguration& conf)
    : RosActionNode<navit_msgs::RoutingAction>(handle, name, conf)
  {  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
        BT::InputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
        BT::InputPort<std::vector<navit::protocol::map_info::MapLine>>("task_paths", "paths"),
        BT::InputPort<std::string>("start_area", "paths"),
        BT::InputPort<std::vector<std::string>>("via_areas", "paths"),
        BT::InputPort<std::string>("goal_area", "paths"),
        BT::OutputPort<bool> ("is_cross_region", "is_cross_region"),
        BT::OutputPort<std::vector<std::string>> ("via_indexes", "via_indexes"),
        BT::OutputPort<int> ("via_nums", "via_nums"),

    });
  }

  bool on_first_tick() override
  { 
    NAVIT_ROS_INFO_STREAM("First tick");
    if (!getInput<std::string>("start_area")) {
        NAVIT_ROS_ERROR_STREAM("Start_area is not specified");
    }
    std::string start_area = getInput<std::string>("start_area").value();

    if (!getInput<std::string>("goal_area")) {
        NAVIT_ROS_ERROR_STREAM("Goal_area is not specified");
    }
    std::string goal_area = getInput<std::string>("goal_area").value();

    if (start_area == goal_area) {
        NAVIT_ROS_WARN_STREAM("Start_area is same with goal_area, we donnot need routing.");
        setOutput("is_cross_region", false);
        return true;
    } else {
        setOutput("is_cross_region", true);
    }

    if (!getInput<std::vector<std::string>>("via_areas")) {
        NAVIT_ROS_ERROR_STREAM("Via_areas is not specified");
    }
    std::vector<std::string> via_areas_string = getInput<std::vector<std::string>>("via_areas").value();

    std::vector<navit::protocol::map_info::MapArea> task_polygons;
    std::vector<navit::protocol::map_info::MapLine> task_paths;
    
    if (!getInput<std::vector<navit::protocol::map_info::MapArea>>("task_polygons")) {
        NAVIT_ROS_ERROR_STREAM("Task_polygons is not specified");
    }
    task_polygons = getInput<std::vector<navit::protocol::map_info::MapArea>>("task_polygons").value();

    if (task_polygons.empty()) {
        NAVIT_ROS_ERROR_STREAM("Task_polygons is empty. Robot can't find path to target area.");
        return false;
    }

    if (!getInput<std::vector<navit::protocol::map_info::MapLine>>("task_paths")) {
        NAVIT_ROS_ERROR_STREAM("Task_paths is not specified");
    }

    task_paths = getInput<std::vector<navit::protocol::map_info::MapLine>>("task_paths").value();

    if (task_paths.empty()) {
        NAVIT_ROS_WARN_STREAM("Task_paths is empty, robot only find path in current area.");
    }
    // build graph map
    navit_msgs::GraphPointInfo graph_point_info;
    navit_msgs::GraphPointsConnection graph_points_connection;
    task_polygons_id_index_map_.clear();
    for (int i = 0; i < task_polygons.size(); i++) {
        task_polygons_id_index_map_[task_polygons[i].name()] = i;
    }

    for (int i = 0; i < task_paths.size(); i++) {
        navit::protocol::map_info::MapLine task_path = task_paths[i];
        for (int j = 0; j < task_polygons.size(); j++) {
            navit::protocol::map_info::MapArea task_polygon = task_polygons[j];
            if (task_polygon.type() == navit::protocol::map_info::MapArea::AREA_TYPE_FULL_COVERAGE) {
                Point boost_point_start(task_path.path(0).x(), task_path.path(0).y());
                Point boost_point_end(task_path.path(task_path.path_size() - 1).x(), task_path.path(task_path.path_size() - 1).y());
                Polygon boost_polygon;
                for (const auto& pt : task_polygon.path()) {
                    boost::geometry::append(boost_polygon, Point(pt.x(), pt.y()));
                }
                if (navit_common::geometry::isPoseInPolygon(boost_point_start, boost_polygon)) {
                    graph_point_info.from = task_polygons_id_index_map_[task_polygon.name()];
                    graph_points_connection.id = task_polygons_id_index_map_[task_polygon.name()];

                }
                if (navit_common::geometry::isPoseInPolygon(boost_point_end, boost_polygon)) {
                    graph_point_info.to = task_polygons_id_index_map_[task_polygon.name()];
                }
            }
        }
        float cost = 0.0;
        for (int k = 0; k < task_path.path_size() - 1; k++) {
            cost = cost + sqrt(pow(task_path.path(i).x() - task_path.path(i+1).x(), 2) + pow(task_path.path(i).y() - task_path.path(i+1).y(), 2));      
        }
        graph_point_info.cost = cost;

        graph_points_connection.predecessors.push_back(graph_point_info);
        graph_points_connection.successors.push_back(graph_point_info);
    }

    if (graph_points_connection.predecessors.empty() || graph_points_connection.successors.empty()) {
        NAVIT_ROS_ERROR_STREAM("Graph_points_connection is empty, robot can't find path to target area.");
        return false;
    }

    // 查询起点和终点的index
    int start_index = task_polygons_id_index_map_[start_area];
    int goal_index = task_polygons_id_index_map_[goal_area];

    std::vector<int64_t> via_points;
    // for (int i = 0; i < via_areas_string.size(); i++) {
    //     via_points.push_back(task_polygons_id_index_map_[via_areas_string[i]]);
    // }
    goal_.start = start_index;
    goal_.goal = goal_index;
    goal_.via_points = via_points;
    goal_.map = graph_points_connection;

    return true;
  }

  BT::NodeStatus onResult(const ResultType& res) override
  {
   
    std::vector<int64_t> route_points =  res.route;
    std::vector<std::string> route_points_index;
    std::vector<std::string> via_path_indexes;

    for (int i = 0; i < route_points.size(); i++) {
        for (auto it = task_polygons_id_index_map_.begin(); it != task_polygons_id_index_map_.end(); it++) {
            if (it->second == route_points[i]) {
                route_points_index.push_back(it->first);
            }
        }
    }
    if (route_points_index.empty()) {
        NAVIT_ROS_ERROR_STREAM("Route_points_index is empty, robot can't find path to target area.");
        return BT::NodeStatus::FAILURE;
    }

    setOutput("via_indexes", route_points_index);
    int size = route_points_index.size();
    setOutput("via_nums", size);
    return BT::NodeStatus::SUCCESS;
  }

private:
  
  std::map<std::string, int64_t> task_polygons_id_index_map_;
};

}  // namespace navit_bt_nodes

#endif  // NAVIT_BT_NODES__PLUGINS__ACTION__ROUTING_ACTION_HPP_
