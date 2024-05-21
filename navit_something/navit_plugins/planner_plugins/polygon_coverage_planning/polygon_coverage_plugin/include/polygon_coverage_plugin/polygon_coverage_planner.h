#ifndef POLYGON_COVERAGE_BASE_PLANNER_H_
#define POLYGON_COVERAGE_BASE_PLANNER_H_

#include "ros/ros.h"
#include "navit_core/base_global_planner.h"
#include "navit_msgs/CoveragePathOnPWHAction.h"

#include <visualization_msgs/MarkerArray.h>
#include "polygon_coverage_planners/planners/polygon_stripmap_planner.h"

namespace polygon_coverage_planning {

class PolygonCoveragePlanner : public navit_core::CoveragePlanner {
public:
	PolygonCoveragePlanner() {};
	~PolygonCoveragePlanner() {};

    void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros);

    bool makePlan(const nav_msgs::Path& edge_path, const geometry_msgs::Pose& start_pose,
                                  nav_msgs::Path& coverage_path) override {return true;} ;

    bool makePlan(const geometry_msgs::Polygon& coverage_area,
                                  const std::vector<geometry_msgs::Polygon>& holes,
                                  const geometry_msgs::Pose& start,
                                  const geometry_msgs::Pose& end,
                                  std::vector<nav_msgs::Path>& coverage_paths,
                                  std::vector<nav_msgs::Path>& contour_paths) override
	{
        if (coverage_area.points.size() < 3) {
            ROS_ERROR("Coverage area is not a polygon");
            return false;
        }

        geometry_msgs::Polygon coverage_area_no_closure = coverage_area;
        if (coverage_area.points.front().x == coverage_area.points.back().x &&
            coverage_area.points.front().y == coverage_area.points.back().y) {
            coverage_area_no_closure.points.pop_back();
        }

        std::vector<geometry_msgs::Polygon> no_closure_holes(holes);
        for (auto& hole : no_closure_holes) {
          if (hole.points.front().x == hole.points.back().x &&
              hole.points.front().y == hole.points.back().y) {
            hole.points.pop_back();
          }
        }


        //nav_msgs::Path dummy_coverage_path;
		auto result = makePlan(coverage_area_no_closure, no_closure_holes, start, end, coverage_paths);
        //coverage_paths.push_back(dummy_coverage_path);
		ROS_INFO("Polygon coverage result: %d", result);
		return true;
	};

    int makePlan(const geometry_msgs::Polygon& coverage_area,
                 const std::vector<geometry_msgs::Polygon>& holes,
                 const geometry_msgs::Pose& start,
                 const geometry_msgs::Pose& end,
                 nav_msgs::Path& planned_path);

    int makePlan(const geometry_msgs::Polygon& coverage_area,
                 const std::vector<geometry_msgs::Polygon>& holes,
                 const geometry_msgs::Pose& start,
                 const geometry_msgs::Pose& end,
                 std::vector<nav_msgs::Path>& planned_path);

   private:
    bool publishTrajectoryPoints();
    bool publishVisualization();
    inline visualization_msgs::MarkerArray createDecompositionMarkers();

    sweep_plan_graph::SweepPlanGraph::Settings settings_;
    std::shared_ptr<polygon_coverage_planning::PolygonStripmapPlanner> planner_;
    // Parameters
    std::optional<PolygonWithHoles> polygon_;
    std::optional<Point_2> start_;
    std::optional<Point_2> goal_;
    std::string global_frame_id_;
    std::optional<double> altitude_{0};

    std::vector<Point_2> solution_;

    ros::Publisher marker_pub_;
    ros::Publisher waypoint_list_pub_;

    visualization_msgs::MarkerArray markers_;
};

} // namespace
#endif
