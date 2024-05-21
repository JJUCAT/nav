#include "prm_planner/prm_global_planner.h"
namespace navit_planner {
std::unordered_map<int, Point> PRM::generateRandomPointsInsidePolygon(const Ring& polygon, int numPoints) {
    std::unordered_map<int, Point> points;

    boost::geometry::model::box<Point> envelope;
    boost::geometry::envelope(polygon, envelope);
    double minX = boost::geometry::get<boost::geometry::min_corner, 0>(envelope);
    double minY = boost::geometry::get<boost::geometry::min_corner, 1>(envelope);
    double maxX = boost::geometry::get<boost::geometry::max_corner, 0>(envelope);
    double maxY = boost::geometry::get<boost::geometry::max_corner, 1>(envelope);

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> disX(minX, maxX);
    std::uniform_real_distribution<> disY(minY, maxY);

    int index = 0;
    while (points.size() < numPoints) {
        double x = disX(gen);
        double y = disY(gen);
        Point p(x, y);
        if (boost::geometry::within(p, polygon)) {
            points[index] = p;
            index++;
        }
    }

    samples_ = points;
    numSamples_ = numPoints;
    return points;
}

void PRM::samplePoints(int N) {
    samples_.clear();
    // Generate N random points inside the bounds
    samples_ = generateRandomPointsInsidePolygon(bounds_.outer(), N);

    // For each hole in the bounds, remove any points inside the hole
    for (const auto& hole : bounds_.inners()) {
        for (auto it = samples_.begin(); it != samples_.end();) {
            if (boost::geometry::within(it->second, hole)) {
                it = samples_.erase(it);
            } else {
                ++it;
            }
        }
    }
}

void PRM::insertSamplePoints(const Point& point) {
    numSamples_++;
    int size = samples_.size();
    std::cout << "size1: " << size << std::endl;
    samples_.insert(std::make_pair(numSamples_, point));
    int size2 = samples_.size();
    std::cout << "size2: " << size2 << std::endl;
}

bool PRM::lineIntersectsObstacle(const Point& p1, const Point& p2) {
    boost::geometry::model::linestring<Point> line;
    line.push_back(p1);
    line.push_back(p2);
    // 这里也可以考虑开放，这样的话可以尽可能让机器人远离边界
    // 但是这样会导致另外一个问题，由于膨胀了尺寸，可能会导致机器人对一些地方无法采样
    // // 缩小因子
    // double shrink_factor = 0.01;

    // // 缩小边界
    // Polygon shrunken_bounds;
    // boost::geometry::buffer(bounds_.outer(), shrunken_bounds,
    //                         boost::geometry::strategy::buffer::distance_symmetric<double>(-shrink_factor));

    // // 检查缩小后的线段是否与外部边界相交
    // if (boost::geometry::intersects(line, shrunken_bounds.outer())) {
    //     return true;
    // }

    // Check if the line intersects with any of the holes (obstacles)
    for (const auto& hole : bounds_.inners()) {
        if (boost::geometry::intersects(line, hole)) {
            return true;
        }
    }
    auto& outer = bounds_.outer();
    for (size_t i = 0; i < outer.size() - 1; ++i) {
        boost::geometry::model::linestring<Point> boundary_line;
        boundary_line.push_back(outer[i]);
        boundary_line.push_back(outer[i+1]);

    // Check if line touches the boundary line but not intersects
        if ((boost::geometry::touches(boundary_line, outer))
            && boost::geometry::covered_by(p1, bounds_.outer()) && boost::geometry::covered_by(p2, bounds_.outer())){
            return false;
        }
    }

    if (boost::geometry::intersects(line, outer)) {
        return true;
    }
    
    if (!boost::geometry::covered_by(p1, bounds_.outer()) && !boost::geometry::covered_by(p2, bounds_.outer())) {
        return true;
    }

    return false;
}

bool PRM::pointOnBoundary(const Point& p, const Ring& boundary) {
    for (size_t i = 0; i < boundary.size() - 1; ++i) {
        boost::geometry::model::linestring<Point> boundary_line;
        boundary_line.push_back(boundary[i]);
        boundary_line.push_back(boundary[i+1]);
        if (boost::geometry::distance(p, boundary_line) < 1e-6) { // adjust tolerance as needed
            return true;
        }
    }
    return false;
}

std::map<int, std::tuple<Point, Point, double>> PRM::connectPoints(int start_id) {
    std::set<std::pair<int, int>, PairComparator> added_edges;
    std::map<int, std::tuple<Point, Point, double>> result;

    for (const auto& sample1 : samples_) {
        std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;

        for (const auto& sample2 : samples_) {
            if (sample1.first != sample2.first) {
                double distance = boost::geometry::distance(sample1.second, sample2.second);
                pq.push({distance, sample2.first});
            }
        }

        int valid_edge_count = 0;
        while (valid_edge_count < 5 && !pq.empty()) {
            auto& sample2 = samples_[pq.top().second];
            if (!lineIntersectsObstacle(sample1.second, sample2)) {
                double weight = boost::geometry::distance(sample1.second, sample2);
                std::pair<int, int> edge = std::minmax(sample1.first, pq.top().second);

                // Only add the edge if it hasn't been added yet
                if (added_edges.find(edge) == added_edges.end()) {
                    boost::add_edge(edge.first, edge.second, graph_);
                    result[start_id] = std::make_tuple(sample1.second, sample2, weight);
                    added_edges.insert(edge);
                    start_id++;
                    valid_edge_count++;
                }
            }
            pq.pop();
        }
    }
    std::cout << "result size: " << result.size() << std::endl;
    return result;
}

std::map<int, std::tuple<Point, Point, double>> PRM::buildGraph(MapInfo& map_info) {
    std::map<int, std::tuple<Point, Point, double>> graph;
    map_graph_.clear();
    for (auto& area: map_info.areas) {
        int size = graph.size();
        setArea(area);
        samplePoints(2000);

        for (int i = 0; i < map_info.connect_edges.size(); ++i) {
            for (int j = 0; j < map_info.connect_edges[i].size(); ++j) {
                insertSamplePoints(map_info.connect_edges[i][j]);
            }
        }
        std::map<int, std::tuple<Point, Point, double>> edges = connectPoints(size);
        graph.insert(edges.begin(), edges.end());
    }
    return graph;
}

std::pair<int, std::tuple<Point, Point, double>>
PRM::findClosestEdge(const std::map<int, std::tuple<Point, Point, double>>& edges, const geometry_msgs::PoseStamped& pose) {
    double min_distance = std::numeric_limits<double>::max();
    std::pair<int, std::tuple<Point, Point, double>> closest_edge;

    for (const auto& edge : edges) {
        const Point& p1 = std::get<0>(edge.second);
        const Point& p2 = std::get<1>(edge.second);

        boost::geometry::model::linestring<Point> line;
        line.push_back(p1);
        line.push_back(p2);

        Point point;
        boost::geometry::set<0> (point, pose.pose.position.x);
        boost::geometry::set<1> (point, pose.pose.position.y);
        double distance = boost::geometry::distance(point, line);

        if (distance < min_distance) {
            min_distance = distance;
            closest_edge = edge;
        }
    }
    std::cout << "=========================================" << std::endl;
    std::cout << "closest_edge id is " << closest_edge.first << std::endl;
    std::cout << "closest_edge id is " << closest_edge.first << std::endl;
    std::cout << "closest_edge id is " << closest_edge.first << std::endl;

    return closest_edge;
}

nav_msgs::Path PRM::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal) {
    DebugViewer viewer(nh_);
    navit_planner::MapInfo map_info = planning_scene_loader_.getMapInfo();
    viewer.initializePublishers(map_info.areas);

    std::map<int, std::tuple<Point, Point, double>> graph = buildGraph(map_info);
    sleep(1.0);
    viewer.vieweGraphIndex(graph);
    sleep(1.0);
    viewer.vieweGraphEdges(graph);
    // use dijkstra to find shortest path
    navit_routing::GraphSearch g;

    std::map<Point, int, PointComparator> point_to_id;
    std::map<int, std::tuple<int, Point, int, Point, float>> edges_with_ids;
    int next_id = 0;

    std::pair<int, std::tuple<Point, Point, double>> start_line, goal_line;
    start_line = findClosestEdge(graph, start);
    goal_line = findClosestEdge(graph, goal);

    for (const auto& [edge_id, edge] : graph) {
        const auto& [p1, p2, weight] = edge;
        // Assign IDs to the points, if they haven't already been assigned
        if (point_to_id.count(p1) == 0) {
            point_to_id[p1] = next_id++;
        }
        if (point_to_id.count(p2) == 0) {
            point_to_id[p2] = next_id++;
        }
        // Retrieve the IDs
        int id1 = point_to_id[p1];
        int id2 = point_to_id[p2];

        edges_with_ids[edge_id] = std::make_tuple(id1, p1, id2, p2, weight);
    }

    for (const auto& [edge_id, edge] : edges_with_ids) {
        const auto& [id1, p1, id2, p2, weight] = edge;
        std::cout << "edge_id is " << edge_id <<  "  id1 is " << id1 << "  id2 is " << id2 << " "<< "weight is " << weight << std::endl;
        g.AddEdge(edge_id, id1, id2, weight);
    }

    std::vector<int64_t> via_points{int64_t(start_line.first), int64_t(goal_line.first)};
    
    navit_core::SearchResultRoute out_result;
    auto shortest_path = g.search(via_points, out_result, true);
    
    for (int i = 0; i < out_result.via_points_order.size(); ++i) {
        std::cout << "out_result via_points_order %d " << i << " is " << out_result.via_points_order[i] << std::endl;
    }

    nav_msgs::Path final_path;
    final_path.header.frame_id = "map";
    geometry_msgs::PoseStamped last_pose;
    for (size_t i = 0; i < out_result.via_points_order.size() - 1; ++i) {
        const auto& edge_id_1 = out_result.via_points_order[i];
        const auto& edge_id_2 = out_result.via_points_order[i + 1];

        if (edges_with_ids.count(edge_id_1) > 0 && edges_with_ids.count(edge_id_2) > 0) {
            const auto& [id1, p1, id2, p2, weight1] = edges_with_ids[edge_id_1];
            const auto& [id3, p3, id4, p4, weight2] = edges_with_ids[edge_id_2];

            geometry_msgs::PoseStamped pose_stamp;
            pose_stamp.header.frame_id = "map";

            if (i == 0) {
                if (p1.x() != p3.x() && p1.x() != p4.x()) {
                    pose_stamp.pose.position.x = p1.x();
                    pose_stamp.pose.position.y = p1.y();
                    pose_stamp.pose.position.z = 0.0;

                    pose_stamp.pose.orientation.x = 0.0;
                    pose_stamp.pose.orientation.y = 0.0;
                    pose_stamp.pose.orientation.z = 0.0;
                    pose_stamp.pose.orientation.w = 1.0;

                    final_path.poses.push_back(pose_stamp);

                    pose_stamp.pose.position.x = p2.x();
                    pose_stamp.pose.position.y = p2.y();
                    pose_stamp.pose.position.z = 0.0;

                    pose_stamp.pose.orientation.x = 0.0;
                    pose_stamp.pose.orientation.y = 0.0;
                    pose_stamp.pose.orientation.z = 0.0;
                    pose_stamp.pose.orientation.w = 1.0;

                    final_path.poses.push_back(pose_stamp);
                } else if (p2.x() != p3.x() && p2.x() != p4.x()) {
                    pose_stamp.pose.position.x = p2.x();
                    pose_stamp.pose.position.y = p2.y();
                    pose_stamp.pose.position.z = 0.0;

                    pose_stamp.pose.orientation.x = 0.0;
                    pose_stamp.pose.orientation.y = 0.0;
                    pose_stamp.pose.orientation.z = 0.0;
                    pose_stamp.pose.orientation.w = 1.0;

                    final_path.poses.push_back(pose_stamp);

                    pose_stamp.pose.position.x = p1.x();
                    pose_stamp.pose.position.y = p1.y();
                    pose_stamp.pose.position.z = 0.0;

                    pose_stamp.pose.orientation.x = 0.0;
                    pose_stamp.pose.orientation.y = 0.0;
                    pose_stamp.pose.orientation.z = 0.0;
                    pose_stamp.pose.orientation.w = 1.0;

                    final_path.poses.push_back(pose_stamp);
                }
            } else {
                 // 将p2加入到路径中
                pose_stamp.pose.position.x = p1.x();
                pose_stamp.pose.position.y = p1.y();
                pose_stamp.pose.position.z = 0.0;

                pose_stamp.pose.orientation.x = 0.0;
                pose_stamp.pose.orientation.y = 0.0;
                pose_stamp.pose.orientation.z = 0.0;
                pose_stamp.pose.orientation.w = 1.0;

                final_path.poses.push_back(pose_stamp);
                last_pose = pose_stamp;
            }
        }
    }

    return final_path;
}
} // namespace navit_planner
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(navit_planner::PRM, navit_core::GlobalPlanner)
