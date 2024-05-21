#ifndef DEBUG_VIEWER_H
#define DEBUG_VIEWER_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <navit_core/base_graph_search.h>
#include <navit_router/router.h>

#include "prm_planner/load_json.h"
#include "prm_planner/prm_global_planner.h"


namespace navit_planner {
class DebugViewer {
public:
    DebugViewer(ros::NodeHandle& nh) : nh_(nh){
        marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("debug_markers", 100);
        marker_edges_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("debug_edges_markers", 100);

        marker_edges_pub_1_ = nh_.advertise<visualization_msgs::MarkerArray>("polygon1", 100);
        marker_edges_pub_2_ = nh_.advertise<visualization_msgs::MarkerArray>("polygon2", 100);

        bounds_pub_ = nh_.advertise<visualization_msgs::Marker>("map_bounds", 10);
        inner_pub_ = nh_.advertise<visualization_msgs::Marker>("map_inner", 10);
        map_drivable_polygons_pub_ = nh.advertise<navit_msgs::PolygonArray>("map_drivable_polygons", 1);
        map_non_drivable_polygons_pub_ = nh.advertise<navit_msgs::PolygonArray>("map_undrivable_polygons", 1);
        result_path_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("result_path_marker_array_topic", 1000);
        edges_num_marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("edge_markers", 1);
        closest_pub_ = nh_.advertise<visualization_msgs::Marker>("/closest_edge", 1);
    }

    void publishRandomPoints(const std::map<int, Point>& points);
    void publishEdges(const std::map<int, Point>& points, const std::map<int, std::pair<Point, Point>>& edges);
    void publishMapPolygon(const std::vector<Point>& points);
    void publishMapPolygon(const Polygon& bounds);
    void publishMapPolygons(const std::vector<Polygon>& polygons);
    void initializePublishers(std::vector<Polygon> polygons);
    std::vector<visualization_msgs::MarkerArray> getEdges(MapInfo map_info);
    void publishEdges(std::vector<visualization_msgs::MarkerArray>& marker_array_vector);

    geometry_msgs::PolygonStamped convertRingToRosPolygon(const Ring& ring);

    std::vector<visualization_msgs::MarkerArray> buildGraph(MapInfo& map_info);

    std::map<int, std::tuple<Point, Point, double>> getGraph() {
        return graph_;
    }

    void publishResultMarker(const navit_core::SearchResultRoute& out_result,
                             std::map<int, std::tuple<int, Point, int, Point, float>> edges_with_ids);
    void vieweGraphIndex(const std::map<int, std::tuple<Point, Point, double>>& graph);
    void vieweGraphEdges(const std::map<int, std::tuple<Point, Point, double>>& graph);
private:

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    std::pair<int, std::tuple<Point, Point, double>> findClosestEdge(const std::map<int, std::tuple<Point, Point, double>>& edges, const Point& point);
    void publishEdge(const std::pair<int, std::tuple<Point, Point, double>>& edge);
   
    ros::NodeHandle nh_{"~"};
    ros::Publisher marker_pub_, marker_edges_pub_, bounds_pub_, inner_pub_, map_drivable_polygons_pub_;
    ros::Publisher map_non_drivable_polygons_pub_, marker_edges_pub_1_, marker_edges_pub_2_, result_path_marker_pub_;
    ros::Publisher closest_pub_, edges_num_marker_pub_;
    
    ros::Subscriber point_sub_ = nh_.subscribe("/clicked_point", 1, &DebugViewer::pointCallback, this);
    std::vector<ros::Publisher> drivable_polygon_pubs_;
    std::vector<ros::Publisher> non_drivable_polygon_pubs_;

    visualization_msgs::Marker createPointMarker(const Point& point, int id);
    visualization_msgs::Marker createLineMarker(const Point& p1, const Point& p2, int id);

    std::vector<visualization_msgs::MarkerArray> marker_array_vector_;
    std::map<int, std::tuple<Point, Point, double>> graph_;
};
}
#endif // DEBUG_VIEWER_H
