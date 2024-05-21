#ifndef PRM_PLANNER_LOAD_JSON_H_
#define PRM_PLANNER_LOAD_JSON_H_
#include <ros/ros.h>
#include <geometry_msgs/PolygonStamped.h>
#include <navit_msgs/PolygonArray.h>
#include <fstream>
#include <jsoncpp/json/json.h>
#include <random>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace navit_planner {

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>::ring_type Ring;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;

struct MapInfo {
    std::vector<Polygon> areas;
    std::vector<std::vector<Point>> connect_edges;
};

struct PairComparator {
    bool operator()(const std::pair<int, int>& a, const std::pair<int, int>& b) const {
        if (a.first != b.first) {
            return a.first < b.first;
        }
        return a.second < b.second;
    }
};

struct PointComparator {
    bool operator()(const Point& a, const Point& b) const {
        if (a.x() < b.x()) return true;
        if (a.x() > b.x()) return false;
        return a.y() < b.y();
    }
};
class PlanningSceneLoader {
public:
    PlanningSceneLoader() {
        drivable_pub_ = nh_.advertise<navit_msgs::PolygonArray>("drivable_areas", 1);
        non_drivable_pub_ = nh_.advertise<navit_msgs::PolygonArray>("non_drivable_areas", 1);

        timer_ = nh_.createTimer(ros::Duration(1.0), &PlanningSceneLoader::PublishScene, this);
    }

    MapInfo LoadFromFile(const std::string& filename) {
        std::ifstream file(filename, std::ifstream::binary);
        Json::Value root;
        file >> root;

        // Load areas
        for (const Json::Value& area_json : root["areas"]) {
            Polygon area;
            geometry_msgs::PolygonStamped ros_polygon;
            ros_polygon.header.frame_id = "map";

            // Load drivable area
            Json::Value bounds_json = area_json["drivableArea"]["polygon"];
            for (const auto& point : bounds_json) {
                Point p(point["x"].asDouble(), point["y"].asDouble());
                boost::geometry::append(area.outer(), p);

                geometry_msgs::Point32 ros_point;
                ros_point.x = p.x();
                ros_point.y = p.y();
                ros_polygon.polygon.points.push_back(ros_point);
            }
            boost::geometry::correct(area.outer());
            drivable_areas_.polygon_array.push_back(ros_polygon.polygon);

            // Load non-drivable areas
            Json::Value obstacles_json = area_json["nonDrivableAreas"];
            for (const auto& obstacle_json : obstacles_json) {
                Ring obstacle;
                geometry_msgs::PolygonStamped ros_obstacle;
                ros_obstacle.header.frame_id = "map";
                for (const auto& point : obstacle_json["polygon"]) {
                    Point p(point["x"].asDouble(), point["y"].asDouble());
                    boost::geometry::append(obstacle, p);

                    geometry_msgs::Point32 ros_point;
                    ros_point.x = p.x();
                    ros_point.y = p.y();
                    ros_obstacle.polygon.points.push_back(ros_point);
                }
                boost::geometry::correct(obstacle);
                area.inners().push_back(obstacle);
                non_drivable_areas_.polygon_array.push_back(ros_obstacle.polygon);
            }
            map_info_.areas.push_back(area);
        }

        for (const Json::Value& connect_edge_json : root["connect_edge"]) {
            std::vector<Point> connect_edge;
            for (const auto& point : connect_edge_json) {
                Point p {point["x"].asDouble(), point["y"].asDouble()};
                connect_edge.push_back(p);
            }
            map_info_.connect_edges.push_back(connect_edge);
        }

        return map_info_;
    }

    void PublishScene(const ros::TimerEvent&) {
        // Publish drivable areas
        drivable_pub_.publish(drivable_areas_);
        // Publish each non-drivable area
        non_drivable_pub_.publish(non_drivable_areas_);
    }

    std::map<int, Point> generateRandomPointsInsidePolygon(int numPoints) {
        std::map<int, Point> points;

        boost::geometry::model::box<Point> envelope;
        boost::geometry::envelope(bounds_, envelope);
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
            if (boost::geometry::within(p, bounds_)) {
                points[index] = p;
                index++;
            }
        }

        samples_ = points;
        return points;
    }

    bool lineIntersectsObstacle(const Point& p1, const Point& p2) {
        boost::geometry::model::linestring<Point> line;
        line.push_back(p1);
        line.push_back(p2);
        for (const auto& obstacle : obstacles_) {
            if (boost::geometry::intersects(line, obstacle)) {
                return true;
            }
        }
        return false;
    }

    std::map<int, std::pair<Point, Point>> connectPoints() {
        std::map<int, std::pair<Point, Point>> result;
        int id = 0;
        for (const auto& sample : samples_) {
            int index = sample.first;
            Point point = sample.second;

            for (const auto& other_sample : samples_) {
                int other_index = other_sample.first;
                Point other_point = other_sample.second;
                if (index != other_index && !lineIntersectsObstacle(point, other_point)) {
                    boost::add_edge(index, other_index, graph_);
                    result[id] = std::make_pair(point, other_point);
                    id++;
                }
            }
        }
        std::cout << "result size " << result.size() << std::endl;
        return result;
    }

    Polygon getBounds() { return bounds_; }

    bool setBounds(Polygon& bounds) {
        bounds_ = bounds;
        return true;
    }

    std::vector<Polygon> getAreas() { return areas_;}

    MapInfo getMapInfo () { return map_info_;}

private:
    ros::Publisher drivable_pub_, non_drivable_pub_;
    ros::NodeHandle nh_;
    ros::Timer timer_;
    navit_msgs::PolygonArray drivable_areas_, non_drivable_areas_;
    geometry_msgs::PolygonStamped drivable_area_;
    std::vector<Ring> obstacles_;
    Graph graph_;
    std::map<int, Point> samples_;
    Polygon bounds_;
    std::vector<Polygon> areas_;
    MapInfo map_info_;
};
} // namespace navit_planner
#endif // NAVIT_PLANNER_MAP_H