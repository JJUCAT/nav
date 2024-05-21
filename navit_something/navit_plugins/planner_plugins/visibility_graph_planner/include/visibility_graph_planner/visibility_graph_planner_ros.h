#ifndef VISIBILITY_GRAPH_ROS_PLANNER_H
#define VISIBILITY_GRAPH_ROS_PLANNER_H
#include <map>
#include <vector>
#include <navit_core/base_global_planner.h>
#include <navit_core/base_graph_search.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <visibility_graph_planner/visibility_graph_planner.h>
#include <navit_router/router.h>

#include <ros/ros.h>

#include "load_json.h"

namespace navit_planner {

class VisibilityGraphROS : public navit_core::GlobalPlanner {
public:
    VisibilityGraphROS() {};

    ~VisibilityGraphROS() {};

    /* @brief  Initialization function for the VisibilityGraphROS object
     *
     * @param  name The name of this planner
     * @param  costmap_ros A pointer to the ROS wrapper of the costmap to use for planning
     * 
     * @return True if the initialization was successful, false otherwise
     */    
    void initialize(const std::string& name,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override;
    /* @brief Given a goal pose in the world, compute a plan
     *
     * @param start The start pose 
     * @param goal The goal pose 
     * @param plan The plan... filled by the planner
     * 
     * @return True if a valid plan was found, false otherwise
     */
    nav_msgs::Path makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) override;

    /* @brief  initialize polygon data
     *
     * @param  polygon_file_path The path of polygon data
     * 
     * @return True if the initialization was successful, false otherwise
     */
    bool initPolygonData(const std::string &polygon_file_path);
    
    /* @brief  get map info
     *
     * @return map info
     */
    MapInfo getMapInfo() {
        return map_info_;
    }

private:
    void publishProcessedPolygons(const MapInfo& processed_map_info);
    
    visualization_msgs::Marker convertToRosPolygonMarker(const Polygon& polygon, int id, const std::string& ns);

    void publishGraph(const Graph& graph);

    uint64_t findNearestNodeId(const geometry_msgs::Point& clicked_point,
                               const std::unordered_map<int, std::vector<std::tuple<int, float, uint16_t, std::pair<Point, Point>>>>& graph);

    void clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);

    ros::Publisher polygon_pub_, marker_array_pub_, marker_graph_pub_, marker_points_pub_;
    ros::Subscriber clicked_point_sub_;
    Polygon bounds_;
    PlanningSceneLoader planning_scene_loader_;
    MapInfo map_info_;
    ros::NodeHandle nh_;
    VisibilityGraph::Ptr visibility_graph_ptr_;
    Graph graph_;
    navit_routing::GraphSearch graph_search_;
    navit_core::SearchResultRoute out_result_;
    std::unordered_map<int, Point> id_to_point_;
};
} // namespace navit_planner

#endif  // VISIBILITY_GRAPH_ROS_PLANNER_H
