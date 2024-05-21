#include "polygon_coverage_plugin/polygon_coverage_planner.h"

#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>

#include "polygon_coverage_planners/graphs/sweep_plan_graph.h"
#include "polygon_coverage_planners/sensor_models/line.h"
#include "polygon_coverage_plugin/ros_interface.h"

namespace polygon_coverage_planning {

using ActionResult = navit_msgs::CoveragePathOnPWHResult;

void PolygonCoveragePlanner::initialize(
    const std::string& name,
    const std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros) {
  ros::NodeHandle pnh("~/" + name);

  // load param

  settings_.cost_function =
      std::bind(&computeEuclideanPathCost, std::placeholders::_1);

  double lateral_footprint, lateral_overlap;
  pnh.param("lateral_footprint", lateral_footprint, 1.5);
  pnh.param("lateral_overlap", lateral_overlap, 0.0);
  settings_.sensor_model =
      std::make_shared<Line>(lateral_footprint, lateral_overlap);

  int dt{1};
  pnh.param("decomposition_type", dt, 1);
  if (!checkDecompositionTypeValid(dt)) {
    ROS_WARN_STREAM(
        "decomposition type is invalid. Using default value of: kTCD");
    dt = static_cast<int>(DecompositionType::kTCD);
  }
  settings_.decomposition_type = static_cast<DecompositionType>(dt);

  double wd{0.0};
  pnh.param("wall_distance", wd, 0.0);
  settings_.wall_distance = wd;

  int turn_type{0}, route_type{0};
  double turn_radius{0}, robot_width{0};
  bool use_start_end_point{false};
  double shrink_factor{0};
  pnh.param("turn_type", turn_type, 0);
  pnh.param("route_type", route_type, 0);
  pnh.param("turn_radius", turn_radius, 1.5);
  pnh.param("robot_width", robot_width, 1.2);
  pnh.param("use_start_end_point", use_start_end_point, false);
  pnh.param("shrink_factor", shrink_factor, 0.0);
  settings_.turn_type = turn_type;
  settings_.route_type = route_type;
  settings_.turn_radius = turn_radius;
  settings_.robot_width = robot_width;
  settings_.use_start_end_point = use_start_end_point;
  settings_.shrink_factor = shrink_factor;
  ROS_WARN_STREAM("Load param decomposition_type, value: " << dt);
  ROS_WARN_STREAM("Load param wall_distance, value: " << wd);
  ROS_WARN_STREAM("Load param lateral_footprint, value: " << lateral_footprint);
  ROS_WARN_STREAM("Load param lateral_overlap, value: " << lateral_overlap);
  ROS_WARN_STREAM("Load param turn_type, value: " << turn_type);
  ROS_WARN_STREAM("Load param route_type, value: " << route_type);
  ROS_WARN_STREAM("Load param turn_radius, value: " << turn_radius);
  ROS_WARN_STREAM("Load param robot_width, value: " << robot_width);
  ROS_WARN_STREAM("Load param use_start_end_point, value: " << use_start_end_point);

  marker_pub_ =
      pnh.advertise<visualization_msgs::MarkerArray>("path_markers", 1, true);
  waypoint_list_pub_ =
      pnh.advertise<geometry_msgs::PoseArray>("waypoint_list", 1, true);
}

int PolygonCoveragePlanner::makePlan(
    const geometry_msgs::Polygon& coverage_area,
    const std::vector<geometry_msgs::Polygon>& holes,
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
    nav_msgs::Path& planned_path) {
  PolygonWithHoles temp_pwh;
  if (!polygonFromMsg(coverage_area, holes, &temp_pwh)) {
    return ActionResult::STATUS_NO_COVERAGE_AREA;
  }
  ROS_INFO_STREAM("Successfully loaded polygon.");
  ROS_INFO_STREAM("Polygon:" << temp_pwh);
  polygon_ = std::make_optional(temp_pwh);
  settings_.polygon = temp_pwh;
  planner_.reset(
      new polygon_coverage_planning::PolygonStripmapPlanner(settings_));

  planner_->setup();
  planner_->isInitialized();

  solution_.clear();
  Point_2 s = Point_2(start.position.x, start.position.y);
  Point_2 g = Point_2(end.position.x, end.position.y);

  auto result = planner_->solve(s, g, &solution_);

  if (!result) {
    return ActionResult::STATUS_INVALID_PLANNER;
  }
  // remove the first and last point
  if (!settings_.use_start_end_point && solution_.size() > 2) {
    solution_.erase(solution_.begin());
    solution_.erase(solution_.end() - 1);
  }

  for(auto &it : solution_){
    geometry_msgs::PoseStamped p;
		p.pose.position.x = CGAL::to_double(it.x());
		p.pose.position.y = CGAL::to_double(it.y());
		planned_path.poses.push_back(p);
  }

  // Publishing the plan if requested
  publishTrajectoryPoints();
  // Publishing the visualization if requested
  publishVisualization();

  return 0;
}

int PolygonCoveragePlanner::makePlan(const geometry_msgs::Polygon& coverage_area,
             const std::vector<geometry_msgs::Polygon>& holes,
             const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
             std::vector<nav_msgs::Path>& planned_path) {
  PolygonWithHoles temp_pwh;
  if (!polygonFromMsg(coverage_area, holes, &temp_pwh)) {
    return ActionResult::STATUS_NO_COVERAGE_AREA;
  }
  ROS_INFO_STREAM("Successfully loaded polygon.");
  ROS_INFO_STREAM("Polygon:" << temp_pwh);
  polygon_ = std::make_optional(temp_pwh);
  settings_.polygon = temp_pwh;
  planner_.reset(
      new polygon_coverage_planning::PolygonStripmapPlanner(settings_));

  planner_->setup();
  planner_->isInitialized();

  std::vector<std::vector<Point_2>> waypoints;
  Point_2 s = Point_2(start.position.x, start.position.y);
  Point_2 g = Point_2(end.position.x, end.position.y);

  auto result = planner_->solve(s, g, &waypoints);

  if (!result) {
    return ActionResult::STATUS_INVALID_PLANNER;
  }
  // remove the first and last point
  // if (!settings_.use_start_end_point && waypoints.size() > 2) {
  //   waypoints.erase(waypoints.begin());
  //   waypoints.erase(waypoints.end() - 1);
  // }

  for (auto& cluster : waypoints) {
    nav_msgs::Path path;
    for (auto& it : cluster) {
      geometry_msgs::PoseStamped p;
      p.pose.position.x = CGAL::to_double(it.x());
      p.pose.position.y = CGAL::to_double(it.y());
      path.poses.push_back(p);
    }
    planned_path.push_back(path);
  }

  // Publishing the plan if requested
  //publishTrajectoryPoints();
  // Publishing the visualization if requested
  //publishVisualization();

  return 0;
}

bool PolygonCoveragePlanner::publishVisualization() {
  ROS_INFO_STREAM("Sending visualization messages.");

  // Delete old markers.
  for (visualization_msgs::Marker& m : markers_.markers)
    m.action = visualization_msgs::Marker::DELETE;
  marker_pub_.publish(markers_);

  // Create new markers.
  markers_.markers.clear();
  // The planned path:
  visualization_msgs::Marker path_points, path_line_strips;
  const double kPathLineSize = 0.2;
  const double kPathPointSize = 0.2;

  createMarkers(solution_, altitude_.value(), global_frame_id_,
                "vertices_and_strip", Color::Gray(), Color::Gray(),
                kPathLineSize, kPathPointSize, &path_points, &path_line_strips);
  markers_.markers.push_back(path_points);
  markers_.markers.push_back(path_line_strips);

  // Start and end points
  visualization_msgs::Marker start_point, end_point;
  createStartAndEndPointMarkers(solution_.front(), solution_.back(),
                                altitude_.value(), global_frame_id_, "points",
                                &start_point, &end_point);
  markers_.markers.push_back(start_point);
  markers_.markers.push_back(end_point);

  // Start and end text.
  visualization_msgs::Marker start_text, end_text;
  createStartAndEndTextMarkers(solution_.front(), solution_.back(),
                               altitude_.value(), global_frame_id_, "points",
                               &start_text, &end_text);
  markers_.markers.push_back(start_text);
  markers_.markers.push_back(end_text);

  // The original polygon:
  const double kPolygonLineSize = 0.4;
  visualization_msgs::MarkerArray polygon;
  if (!polygon_.has_value()) {
    ROS_WARN_STREAM("Cannot send visualization because polygon not set.");
    return false;
  }
  createPolygonMarkers(polygon_.value(), altitude_.value(), global_frame_id_,
                       "polygon", Color::Black(), Color::Black(),
                       kPolygonLineSize, kPolygonLineSize, &polygon);
  markers_.markers.insert(markers_.markers.end(), polygon.markers.begin(),
                          polygon.markers.end());

  // The decomposed polygons.
  visualization_msgs::MarkerArray decomposition_markers =
      createDecompositionMarkers();
  markers_.markers.insert(markers_.markers.end(),
                          decomposition_markers.markers.begin(),
                          decomposition_markers.markers.end());

  // Publishing
  marker_pub_.publish(markers_);

  // Success
  return true;
}

bool PolygonCoveragePlanner::publishTrajectoryPoints() {
  ROS_INFO_STREAM("Sending trajectory messages");

  // Convert path to pose array.
  geometry_msgs::PoseArray trajectory_points_pose_array;
  if (!altitude_.has_value()) {
    ROS_WARN_STREAM("Cannot send trajectory because altitude not set.");
    return false;
  }
  poseArrayMsgFromPath(solution_, altitude_.value(), global_frame_id_,
                       &trajectory_points_pose_array);

  // Publishing
  waypoint_list_pub_.publish(trajectory_points_pose_array);

  // Success
  return true;
}

inline visualization_msgs::MarkerArray
PolygonCoveragePlanner::createDecompositionMarkers() {
  visualization_msgs::MarkerArray markers;
  if (!planner_) {
    return markers;
  }

  std::vector<Polygon_2> decomposition = planner_->getDecomposition();
  for (size_t i = 0; i < decomposition.size(); ++i) {
    visualization_msgs::MarkerArray decomposition_markers;
    std::string name = "decomposition_polygon_" + std::to_string(i);
    const double kPolygonLineSize = 0.4;
    createPolygonMarkers(PolygonWithHoles(decomposition[i]), altitude_.value(),
                         global_frame_id_, name, Color::Gray(), Color::Gray(),
                         kPolygonLineSize, kPolygonLineSize,
                         &decomposition_markers);
    markers.markers.insert(markers.markers.end(),
                           decomposition_markers.markers.begin(),
                           decomposition_markers.markers.end());
  }

  return markers;
}

}  // namespace polygon_coverage_planning
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(polygon_coverage_planning::PolygonCoveragePlanner,
                       navit_core::CoveragePlanner)
