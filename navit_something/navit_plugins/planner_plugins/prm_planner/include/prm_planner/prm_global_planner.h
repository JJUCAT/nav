#ifndef PRM_H
#define PRM_H
#include <map>
#include <vector>
#include <navit_core/base_global_planner.h>

#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>

#include <iostream>
#include <prm_planner/debug_viewer.h>
#include "load_json.h"

namespace navit_planner {
class PRM : public navit_core::GlobalPlanner {
public:
    PRM() {};
    ~PRM() {};
    // base class interface
    void initialize(const std::string& name,
                    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override {};
    nav_msgs::Path makePlan(const geometry_msgs::PoseStamped& start,
                            const geometry_msgs::PoseStamped& goal) override;

    bool initPolygonData(const std::string &polygon_file_path) {
        map_info_ = planning_scene_loader_.LoadFromFile(polygon_file_path);
        return true;
    }
    void setArea(const Polygon& bounds) {
        bounds_ = bounds;
    };
    void samplePoints(int N);
    bool lineIntersectsObstacle(const Point& p1, const Point& p2);
    std::map<int, std::tuple<Point, Point, double>> connectPoints(int start_id);
    std::unordered_map<int, Point> generateRandomPointsInsidePolygon(const Ring& polygon, int numPoints);
    void insertSamplePoints(const Point& point);
    bool pointOnBoundary(const Point& p, const Ring& boundary);

private:
    std::map<int, std::tuple<Point, Point, double>> buildGraph(MapInfo& map_info);
    std::pair<int, std::tuple<Point, Point, double>> findClosestEdge(const std::map<int, std::tuple<Point, Point, double>>& edges, 
                                                                        const geometry_msgs::PoseStamped& pose);
    Polygon bounds_;
    Graph graph_;
    std::map<int, std::tuple<Point, Point, double>> map_graph_;
    std::unordered_map<int, Point> samples_;
    int numSamples_ = 0;

    PlanningSceneLoader planning_scene_loader_;
    std::vector<visualization_msgs::MarkerArray> marker_array_vector_;

    navit_planner::MapInfo map_info_;
    ros::NodeHandle nh_;
    
};
} // namespace navit_planner

#endif  // PRM_H