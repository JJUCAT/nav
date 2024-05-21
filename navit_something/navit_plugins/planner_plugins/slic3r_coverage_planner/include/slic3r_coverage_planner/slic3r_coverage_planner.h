#include <ros/ros.h>
#include <navit_core/base_global_planner.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// slic3r library api
#include "ExPolygon.hpp"
#include "Polyline.hpp"
#include "Fill/FillRectilinear.hpp"
#include "Fill/FillConcentric.hpp"

#include <Fill/FillPlanePath.hpp>
#include <PerimeterGenerator.hpp>
#include <ClipperUtils.hpp>
#include <ExtrusionEntityCollection.hpp>

namespace slic3r_coverage_planner {

class Slic3rCoveragePlanner : public navit_core::CoveragePlanner {
public:
  //virtual void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) = 0;
  void initialize(const std::string& name, const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) override 
  {
    ROS_INFO("Slic3rCoveragePlanner::initialize");
    initialize(name);
  }

  void initialize(const std::string& name);

  bool makePlan(const nav_msgs::Path& edge_path, 
                const geometry_msgs::Pose& start_pose,
                nav_msgs::Path& coverage_path) override
  {
    ROS_WARN("Slic3rCoveragePlanner does not support makePlan using only edge_path");
    return false;
  }

  bool makePlan(const geometry_msgs::Polygon& coverage_area,
                const std::vector<geometry_msgs::Polygon>& holes,
                const geometry_msgs::Pose& start,
                const geometry_msgs::Pose& end,
                std::vector<nav_msgs::Path>& coverage_path,
                std::vector<nav_msgs::Path>& contour_path) override;
private:
  void traverse(std::vector<PerimeterGeneratorLoop> &contours, std::vector<Polygons> &line_groups) {
    for (auto &contour: contours) {
        if (contour.children.empty()) {
            line_groups.push_back(Polygons());
        } else {
            traverse(contour.children, line_groups);
        }
        line_groups.back().push_back(contour.polygon);
    }
}
  double angle_, distance_, outer_offset_;
  bool is_contour_;
};

} // namespace slic3r_coverage_planner
