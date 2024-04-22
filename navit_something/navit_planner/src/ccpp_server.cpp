#include "navit_planner/ccpp_server.h"

namespace navit_planner {

CoveragePlannerServer::CoveragePlannerServer(
    std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh,
    const std::string& param_ns)
    : nh_(nh),
      plugin_manager_(
          "coverage_planner",
          boost::bind(&CoveragePlannerServer::loadPlugin, this, _1),
          boost::bind(&CoveragePlannerServer::initPlugin, this, _1, _2), nh_),
      plugin_loader_("navit_core", "navit_core::CoveragePlanner"),
      action_server_(nullptr) {
  action_server_ = std::make_shared<ActionServer>(
      nh_, "/coverage_path",
      boost::bind(&CoveragePlannerServer::computePath, this, _1), false);

  plugin_manager_.loadPlugins();

  plan_pub_ =
      nh_.advertise<nav_msgs::Path>("/planned_polygon_coverage_path", 1);

  action_server_->start();
}

CoveragePlannerServer::~CoveragePlannerServer() {
  plugin_manager_.clearPlugins();
}

int CoveragePlannerServer::getPlan(
    const geometry_msgs::Polygon& coverage_area,
    const std::vector<geometry_msgs::Polygon>& holes,
    const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
    const std::string& plugin_name, nav_msgs::Path& planned_path) 
{
  
	auto planner_ = plugin_manager_.getPlugin(plugin_name);

    // use old navit_core interface
	std::vector<nav_msgs::Path> coverage_paths;
	std::vector<nav_msgs::Path> contour_paths;
	bool ret = planner_->makePlan(coverage_area, holes, start, end, coverage_paths, contour_paths);

	if (ret) {
		ROS_INFO("Planner returned a valid plan");
		// covert coverage paths to a single planned_path
		for (auto& path : coverage_paths) {
			planned_path.poses.insert(planned_path.poses.end(), path.poses.begin(), path.poses.end());
		}
	} else {
		ROS_ERROR("Planner returned an invalid plan");
		return -1;
	}

	return 0;
}

void CoveragePlannerServer::computePath(const ActionGoal& goal) {
  ActionResult result;
  geometry_msgs::Polygon coverage_area = goal->coverage_area;
  std::vector<geometry_msgs::Polygon> holes = goal->holes;

  nav_msgs::Path plan_path;
  int status = getPlan(coverage_area, holes, goal->start, goal->end,
                       goal->plugin_name, plan_path);

  if (status) {
    result.is_path_found = false;
    result.error_msg = "Failed to find path";
    result.error_code = status;
    action_server_->setAborted(result, "Failed to find path");
    ROS_ERROR_STREAM("Failed to find coverage path");
    return;
  }

  action_server_->setSucceeded(result);
  // pub plan

  plan_path.header.frame_id = "map";
  plan_pub_.publish(plan_path);
}

CoveragePlannerPtr CoveragePlannerServer::loadPlugin(const std::string& plugin_name) {
  CoveragePlannerPtr planner_ptr;
  try {
    planner_ptr = plugin_loader_.createInstance(plugin_name);
    std::string planner_name = plugin_loader_.getName(plugin_name);
    ROS_DEBUG_STREAM("planner plugin " << plugin_name << "with name " << planner_name
                                       << " loaded");
  } catch (const pluginlib::PluginlibException& ex) {
    ROS_WARN_STREAM("Failed to load " << plugin_name << " with " << ex.what());
  }

  return planner_ptr;
}
bool CoveragePlannerServer::initPlugin(const std::string& plugin_name,
                                       const CoveragePlannerPtr& planner_ptr) {
  planner_ptr->initialize(plugin_name, costmap_);
  return true;
}

}  // namespace navit_planner
