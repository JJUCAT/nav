#include "visibility_graph_planner/visibility_graph_planner_ros.h"
namespace navit_planner {

void VisibilityGraphROS::initialize(const std::string& name,
                const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) {
    //TODO(CZK):check file if exist
    initPolygonData("/home/yjh/test/polygons.json");
    marker_array_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("processed_polygons", 8);
    marker_graph_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("graph_markers", 100);
    marker_points_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker_arrayxx", 100);
    clicked_point_sub_ = nh_.subscribe("/clicked_point", 10, &VisibilityGraphROS::clickedPointCallback, this);
};

nav_msgs::Path VisibilityGraphROS::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) {
    visibility_graph_ptr_ = std::make_shared<VisibilityGraph>(0.2, 0.2, map_info_);
    
    auto start_time1 = std::chrono::high_resolution_clock::now();
    MapInfo processed_map_info = visibility_graph_ptr_->processMapInfo();
    auto end_time1 = std::chrono::high_resolution_clock::now();
    auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time1 - start_time1).count();

    std::cout << "processMapInfo() took " << duration1 << " milliseconds.\n";

    publishProcessedPolygons(processed_map_info);

    auto start_time2 = std::chrono::high_resolution_clock::now();
    visibility_graph_ptr_-> constructVisibilityGraph(processed_map_info);
    auto end_time2 = std::chrono::high_resolution_clock::now();
    auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds>(end_time2 - start_time2).count();
    
    std::cout << "constructVisibilityGraph() took " << duration2 << " milliseconds.\n";

    graph_ = visibility_graph_ptr_->getGraph();
    publishGraph(graph_);

    for (const auto& node : graph_) {
        int u = node.first;
        const std::vector<std::tuple<int, float, uint16_t, std::pair<Point, Point>>>& edges = node.second;
        for (const auto& edge : edges) {
            int v = std::get<0>(edge);
            float weight = std::get<1>(edge);
            graph_search_.AddEdge(std::get<2>(edge), u, v, weight);
            //std::cout << "u = " << u << ", v = " << v << ", weight = " << weight << std::endl;
            Point u_point = std::get<3>(edge).first; 
            Point v_point = std::get<3>(edge).second;
            id_to_point_[u] = u_point;
            id_to_point_[v] = v_point;
        }
    }
   
    return nav_msgs::Path();
};

bool VisibilityGraphROS::initPolygonData(const std::string &polygon_file_path) {
    planning_scene_loader_.ReadMapInfoFromJson(polygon_file_path, &map_info_);
    return true;
}

void VisibilityGraphROS::publishProcessedPolygons(const MapInfo& processed_map_info) {
    visualization_msgs::MarkerArray marker_array;
    sleep(1);
    int id = 0;
    for (const auto& [key, value] : processed_map_info) {
        Polygon outer_polygon;
        std::vector<Polygon> inner_polygons;
        std::vector<std::pair<Point, Point>> points_pairs;
        std::map<uint64_t, Point> workpoints;

        std::tie(outer_polygon, inner_polygons, points_pairs, workpoints) = value;

        // Convert and add the outer polygon to the marker array
        visualization_msgs::Marker outer_marker = convertToRosPolygonMarker(outer_polygon, id++, "outer_polygon");
        marker_array.markers.push_back(outer_marker);

        // Convert and add the inner polygons to the marker array
        for (const auto& inner_polygon : inner_polygons) {
            visualization_msgs::Marker inner_marker = convertToRosPolygonMarker(inner_polygon, id++, "inner_polygon");
            marker_array.markers.push_back(inner_marker);
        }
    }

    // Publish all markers in the array
    marker_array_pub_.publish(marker_array);
}


visualization_msgs::Marker VisibilityGraphROS::convertToRosPolygonMarker(const Polygon& polygon, int id, const std::string& ns) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1; // width of the line

    for (const auto& point : polygon.outer()) {
        geometry_msgs::Point p;
        p.x = point.x();
        p.y = point.y();
        p.z = 0;
        marker.points.push_back(p);
    }

    // add the first point again to close the polygon
    marker.points.push_back(marker.points[0]);

    // set color (change as needed)
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    return marker;
}
void VisibilityGraphROS::publishGraph(const Graph& graph) {
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;
    sleep(1);
    // 对于图中的每个顶点
    for (const auto& [vertex_id, edges] : graph) {
        // 对于每个顶点的每条边
        for (const auto& [target_id, weight, edge_id, edge_points] : edges) {
            // 创建一个新的Marker来可视化这条边
            visualization_msgs::Marker edge_marker;
            edge_marker.header.frame_id = "map";
            edge_marker.header.stamp = ros::Time::now();
            edge_marker.ns = "edges";
            edge_marker.id = marker_id++;
            edge_marker.type = visualization_msgs::Marker::LINE_LIST;
            edge_marker.action = visualization_msgs::Marker::ADD;
            edge_marker.scale.x = 0.02;
            edge_marker.color.a = 1.0;
            edge_marker.color.r = 0.3;
            edge_marker.color.g = 0.2;
            edge_marker.color.b = 0.5;
            
            // 设定线段的两个点
            geometry_msgs::Point p1, p2;
            p1.x = edge_points.first.get<0>();
            p1.y = edge_points.first.get<1>();
            p2.x = edge_points.second.get<0>();
            p2.y = edge_points.second.get<1>();
            edge_marker.points.push_back(p1);
            edge_marker.points.push_back(p2);

            // 将新的Marker添加到MarkerArray中
            marker_array.markers.push_back(edge_marker);
        }
    }

    // 发布MarkerArray
    marker_graph_pub_.publish(marker_array);
}
uint64_t VisibilityGraphROS::findNearestNodeId(const geometry_msgs::Point& clicked_point,
                           const std::unordered_map<int, std::vector<std::tuple<int, float, uint16_t, std::pair<Point, Point>>>>& graph) {
    uint64_t nearest_node_id = 0;
    float min_distance = std::numeric_limits<float>::max();

    // 遍历graph的所有节点
    for (const auto& node : graph) {
        int node_id = node.first;
        
        // 取节点的第一个边的第一个点作为该节点的位置
        // 假设graph的构造保证每个节点至少有一个边，且每个边的两个端点中的至少一个是该节点
        Point node_point = std::get<3>(node.second[0]).first;
        
        // 计算clicked_point与节点的距离
        float distance = std::sqrt(std::pow(clicked_point.x - node_point.get<0>(), 2) + 
                                   std::pow(clicked_point.y - node_point.get<1>(), 2));

        // 如果节点的距离比之前的最小距离小，那么更新最小距离并保存节点ID
        if (distance < min_distance) {
            min_distance = distance;
            nearest_node_id = node_id;
        }
    }

    return nearest_node_id;
}
void VisibilityGraphROS::clickedPointCallback(const geometry_msgs::PointStamped::ConstPtr& msg) {
    // 获取最近的节点ID
    uint64_t id = findNearestNodeId(msg->point, graph_);
    static std::vector<int64_t> way_points;
    way_points.push_back(id);
    std::cout << "id " << id << "id _size " << way_points.size() << std::endl;
    if (way_points.size() == 2) {
        graph_search_.search(way_points, out_result_);

        for (int i = 0; i < out_result_.via_points_order.size(); ++i) {
            std::cout << "out_result via_points_order %d " << i << " is " << out_result_.via_points_order[i] << std::endl;
        }
        sleep(1);
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "map";  // Assume you're using the map coordinate system
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "my_namespace";
        line_marker.id = 0;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;

        for (int i = 0; i < out_result_.via_points_order.size(); ++i) {
            int id = out_result_.via_points_order[i];
            Point point = id_to_point_[id];

            geometry_msgs::Point p;
            p.x = point.x();
            p.y = point.y();
            p.z = 0;
            line_marker.points.push_back(p);
        }

        line_marker.scale.x = 0.1;
        line_marker.color.r = 0.0f;
        line_marker.color.g = 0.0f;
        line_marker.color.b = 1.0f;
        line_marker.color.a = 1.0;

        marker_points_pub_.publish(line_marker);

        way_points.clear();
    }
}
} // namespace navit_planner
// #include "pluginlib/class_list_macros.hpp"
// PLUGINLIB_EXPORT_CLASS(navit_planner::VisibilityGraph, navit_core::GlobalPlanner)

int main(int argc, char** argv) {
    ros::init(argc, argv, "visibility_graph_planner");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    navit_planner::VisibilityGraphROS planner;
    std::shared_ptr<navit_costmap::Costmap2DROS> costmap_ros;
    planner.initialize("visibility_graph_planner", costmap_ros);
    planner.makePlan(geometry_msgs::PoseStamped(), geometry_msgs::PoseStamped());

    ros::spin();
    return 0;
}
