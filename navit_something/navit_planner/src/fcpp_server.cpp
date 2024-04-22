#include <navit_planner/fcpp_server.h>

namespace navit_planner
{

PolygonCoverageServer::PolygonCoverageServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh,
                               const std::string& param_ns):
    nh_(nh),
    planner_plugin_manager_("coverage_planner", boost::bind(&PolygonCoverageServer::loadPlannerPlugins, this, _1),
                            boost::bind(&PolygonCoverageServer::initPlannerPlugins, this, _1, _2), nh_),
    planner_loader_("navit_core", "navit_core::CoveragePlanner"),
    as_(nullptr) {
        as_ = std::make_shared<PolygonCoverageActionServer>(nh_, "/polygon_coverage_path",
                                                    boost::bind(&PolygonCoverageServer::computePath, this, _1), false);

        planner_plugin_manager_.loadPlugins();

        nh_.param("expand_boundary", expand_boundary_, 0.0f);
        nh_.param("contract_boundary", contract_boundary_, 0.0f);
        nh_.param("simplification_dis", simplification_dis_, 0.1f);
        nh_.param("use_dubins", use_dubins_, false);
        nh_.param("max_angle_threshold", max_angle_threshold_, 150.0);
        nh_.param("num_points_before_after", num_points_before_after_, 10);

        coverage_plan_pub_ = nh_.advertise<nav_msgs::Path>("/planned_polygon_coverage_path", 1);
        wall_plan_pub_ = nh_.advertise<nav_msgs::Path>("/planned_polygon_wall_path", 1);

        coverage_area_pub_ = nh_.advertise<visualization_msgs::Marker>("/polygon_coverage_area", 1);
        holes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/polygon_holes", 1);
        rehanld_path_pub_ = nh_.advertise<nav_msgs::Path>("/rehandle_path", 1);

        ompl_ros_ptr_ = std::make_shared<OmplRos>(nh_, tf_buffer);
        as_->start();
}

PolygonCoveragePlannerPtr PolygonCoverageServer::loadPlannerPlugins(const std::string& plugin)
{
  PolygonCoveragePlannerPtr planner_ptr;
  try
  {
    planner_ptr = planner_loader_.createInstance(plugin);
    std::string planner_name = planner_loader_.getName(plugin);
    ROS_DEBUG_STREAM("planner plugin " << plugin << "with name " << planner_name << " loaded");
  }
  catch (const pluginlib::PluginlibException& ex)
  {
    ROS_WARN_STREAM("Failed to load " << plugin << " with " << ex.what());
  }

  return planner_ptr;
}

bool PolygonCoverageServer::initPlannerPlugins(const std::string& plugin, const PolygonCoveragePlannerPtr& planner_ptr)
{
  planner_ptr->initialize(plugin, costmap_);
  return true;
}

void PolygonCoverageServer::computePath(const PolygonCoverageActionGoal& action_goal)
{
    PolygonCoverageActionResult result;
    PolygonCoverageActionGoal goal = action_goal;
    if (goal->coverage_area.points.size() < 3) {
        result.is_path_found = false;
        result.error_msg = "Coverage area is invalid";
        result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_NO_COVERAGE_AREA;
        as_->setAborted(result, "Coverage area is invalid");
        ROS_ERROR_STREAM("Coverage area is invalid, abort coverage planning, size is " << goal->coverage_area.points.size());
        return;
    }

    if (goal->holes.size() > 0) {
        for (auto& hole : goal->holes) {
            if (hole.points.size() < 3) {
                result.is_path_found = false;
                result.error_msg = "Hole is invalid";
                result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_NO_COVERAGE_AREA;
                as_->setAborted(result, "Hole is invalid");
                ROS_ERROR_STREAM("Hole is invalid, abort coverage planning");
                return;
            }
        }
    }

    geometry_msgs::Polygon coverage_area = goal->coverage_area;
    std::vector<geometry_msgs::Polygon> holes = goal->holes;
    // expand_boundary
    geometry_msgs::Polygon expand_coverage_area;
    std::vector<geometry_msgs::Polygon> expand_holes;
    planner_server_helper_->adjustPolygonBoundary(coverage_area, expand_boundary_, expand_coverage_area);
    planner_server_helper_->adjustPolygonsBoundary(holes, expand_boundary_, expand_holes);

    geometry_msgs::Polygon simpli_coverage_area;
    std::vector<geometry_msgs::Polygon> simpli_holes;

    planner_server_helper_->simplifyPolygon(coverage_area, simplification_dis_, simpli_coverage_area);
    planner_server_helper_->simplifyPolygons(expand_holes, simplification_dis_, simpli_holes);

    // if (!planner_server_helper_->isPoseInPolygon(goal->start, simpli_coverage_area))
    // {
    //     result.is_path_found = false;
    //     result.error_msg = "Start pose is out of the coverage area";
    //     result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_START_OUTSIDE_COVERAGE_AREA;
    //     as_->setAborted(result, "start pose is out of the coverage area");
    //     ROS_ERROR_STREAM("Start pose is out of the coverage area, abort coverage planning");
    //     return;
    // }

    // if (!planner_server_helper_->isPoseInPolygon(goal->end, simpli_coverage_area))
    // {
    //     result.is_path_found = false;
    //     result.error_msg = "Goal pose is out of the coverage area";
    //     result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_END_OUTSIDE_COVERAGE_AREA;
    //     as_->setAborted(result, "goal pose is out of the coverage area");
    //     ROS_ERROR_STREAM("Goal pose is out of the coverage area, abort coverage planning");
    //     return;
    // }

    if (planner_server_helper_->isPoseInPolygons(goal->start, simpli_holes))
    {
        result.is_path_found = false;
        result.error_msg = "Start pose is in the holes";
        result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_START_IN_FORBIDDEN_ZONE;
        as_->setAborted(result, "start pose is in the holes");
        ROS_ERROR_STREAM("Start pose is in the holes, abort coverage planning");
        return;
    }

    if (planner_server_helper_->isPoseInPolygons(goal->end, simpli_holes))
    {
        result.is_path_found = false;
        result.error_msg = "goal pose is in the holes";
        result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_END_IN_FORBIDDEN_ZONE;
        as_->setAborted(result, "goal pose is in the holes");
        ROS_ERROR_STREAM("Goal pose is in the holes, abort coverage planning");
        return;
    }

    if (!planner_plugin_manager_.hasPlugin(goal->plugin_name))
    {
        result.is_path_found = false;
        result.error_msg = "Failed to load planner plugin";
        result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_INVALID_PLANNER;
        as_->setAborted(result, "Failed to load planner plugin");
        ROS_ERROR_STREAM("Failed to load ccpp planner plugin " << goal->plugin_name << ", abort coverage planning");
        return;
    }
    // TODO(czk): maybe need check tf?
    ros::Time planning_start = ros::Time::now();

    // pub simpli_coverage_area
    visualization_msgs::Marker coverage_area_marker;
    coverage_area_marker.header.frame_id = "map";
    coverage_area_marker.header.stamp = ros::Time::now();
    coverage_area_marker.ns = "coverage_area";
    coverage_area_marker.id = 0;
    coverage_area_marker.type = visualization_msgs::Marker::LINE_STRIP;
    coverage_area_marker.action = visualization_msgs::Marker::ADD;
    coverage_area_marker.pose.orientation.w = 1.0;
    coverage_area_marker.scale.x = 0.1;
    coverage_area_marker.color.r = 1.0;
    coverage_area_marker.color.g = 0.0;
    coverage_area_marker.color.b = 0.0;
    coverage_area_marker.color.a = 1.0;
    coverage_area_marker.lifetime = ros::Duration(0.0);
    coverage_area_marker.frame_locked = false;
    for (auto& point : simpli_coverage_area.points) {
      geometry_msgs::Point p;
      p.x = point.x;
      p.y = point.y;
      p.z = 0.0;
      coverage_area_marker.points.push_back(p);
    }
    coverage_area_pub_.publish(coverage_area_marker);

    // pub simpli_holes
    visualization_msgs::MarkerArray holes_marker_array;
    for (int i = 0; i < simpli_holes.size(); i++) {
      visualization_msgs::Marker hole_marker;
      hole_marker.header.frame_id = "map";
      hole_marker.header.stamp = ros::Time::now();
      hole_marker.ns = "hole";
      hole_marker.id = i;
      hole_marker.type = visualization_msgs::Marker::LINE_STRIP;
      hole_marker.action = visualization_msgs::Marker::ADD;
      hole_marker.pose.orientation.w = 1.0;
      hole_marker.scale.x = 0.1;
      hole_marker.color.r = 0.0;
      hole_marker.color.g = 1.0;
      hole_marker.color.b = 0.0;
      hole_marker.color.a = 1.0;
      hole_marker.lifetime = ros::Duration(0.0);
      hole_marker.frame_locked = false;
      for (auto& point : simpli_holes[i].points) {
        geometry_msgs::Point p;
        p.x = point.x;
        p.y = point.y;
        p.z = 0.0;
        hole_marker.points.push_back(p);
      }
      holes_marker_array.markers.push_back(hole_marker);
    }

    holes_pub_.publish(holes_marker_array);
    std::vector<nav_msgs::Path> planned_coverage_path_v, planned_wall_path_v, smooth_planned_coverage_path_v, smooth_planned_wall_path_v;

    bool ret = getPlan(simpli_coverage_area,
                       simpli_holes,
                       goal->start,
                       goal->end,
                       goal->plugin_name,
                       planned_coverage_path_v,
                       planned_wall_path_v);

    ros::Time planning_end = ros::Time::now();

    if (!ret)
    {
        result.is_path_found = false;
        result.error_msg = "Failed to find path";
        result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_PLANNING_FAILED;
        as_->setAborted(result, "Failed to find path");
        ROS_ERROR_STREAM("Failed to find coverage path");
        return;
    }

    float planning_used_time = (ros::Time::now() - planning_start).toSec();

    // check if path is valid, ccpp orientation is not valid
    nav_msgs::Path whole_path;
    whole_path.header.frame_id = "map";               // default is map

    if (planned_coverage_path_v.size() == 0) {
      result.is_path_found = false;
      result.error_msg = "Failed to find path";
      result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_PLANNING_FAILED;
      as_->setAborted(result, "Failed to find path");
      ROS_ERROR_STREAM("Failed to find coverage path");
      return;
    }
    // TODO(CZK): 将分区后的全覆盖区域进行分离，方便业务端进行处理

    for (auto& it : planned_coverage_path_v) {
      it = navit_common::resamplePath(it, 0.1);
      checkPath(it);
      smooth_planned_coverage_path_v.push_back(it);
    }

    for (auto& it : planned_wall_path_v) {
      it = navit_common::resamplePath(it, 0.1);
      checkPath(it);
    }
    // resample path
    geometry_msgs::Polygon second_expand_coverage_area;
    planner_server_helper_->adjustPolygonBoundary(coverage_area, 1.0, second_expand_coverage_area);

    ompl_ros_ptr_->setBoundingBox(coverage_area, holes);

    std::vector<geometry_msgs::PoseStamped> path_poses ;

    for (int i = 0; i < smooth_planned_coverage_path_v.size(); ++i) {
        try {
            planPathBetweenPoints(smooth_planned_coverage_path_v[i], max_angle_threshold_, num_points_before_after_);
        } catch (const std::exception& e) {
            ROS_WARN("Cannot find valid path for index %d.", i);
        }

        whole_path.poses.insert(whole_path.poses.end(),
                                 smooth_planned_coverage_path_v[i].poses.begin(),
                                 smooth_planned_coverage_path_v[i].poses.end());
        if (!smooth_planned_coverage_path_v[i].poses.empty() && i < smooth_planned_coverage_path_v.size() - 1) {
            geometry_msgs::Pose lastPose = smooth_planned_coverage_path_v[i].poses.back().pose;
            geometry_msgs::Pose firstPose = smooth_planned_coverage_path_v[i + 1].poses.front().pose;
            ompl_ros_ptr_->planPath(lastPose, firstPose, path_poses, true, false);
            whole_path.poses.insert(whole_path.poses.end(), path_poses.begin(), path_poses.end() );
        }
    }

    coverage_plan_pub_.publish(whole_path);
    float path_length = 0.0f, area = 0.0f;

    planner_server_helper_->getCoveragePathLength(planned_coverage_path_v, planned_wall_path_v, path_length);
    planner_server_helper_->getPolygonWithHolesArea(goal->coverage_area, goal->holes, area);

    planned_coverage_path_v.clear();
    planned_coverage_path_v.push_back(whole_path);
    result.length_path = path_length;
    result.area = area;
    result.coverage_paths = planned_coverage_path_v;
    result.contour_paths = planned_wall_path_v;
    result.planning_duration = planning_used_time;
    result.is_path_found = true;
    result.error_code = navit_msgs::CoveragePathOnPWHResult::STATUS_OK;
    result.error_msg = "Path found";

    as_->setSucceeded(result);

    // pub planned_coverage_path

    // nav_msgs::Path planned_coverage_path, planned_wall_path;

    // planned_coverage_path.header.frame_id = "map";
    // for (auto path : planned_wall_path_v) {
    //   planned_coverage_path.poses.insert(planned_coverage_path.poses.end(), path.poses.begin(), path.poses.end());
    // }
    // coverage_plan_pub_.publish(planned_coverage_path);

    // planned_wall_path.header.frame_id = "map";
    // for (auto path : planned_wall_path_v) {
    //   planned_wall_path.poses.insert(planned_wall_path.poses.end(), path.poses.begin(), path.poses.end());
    // }
    // wall_plan_pub_.publish(planned_wall_path);
}

bool PolygonCoverageServer::getPlan(const geometry_msgs::Polygon& coverage_area,
                                    const std::vector<geometry_msgs::Polygon>& holes,
                                    const geometry_msgs::Pose& start,
                                    const geometry_msgs::Pose& end,
                                    const std::string& plugin,
                                    std::vector<nav_msgs::Path>& planned_coverage_path,
                                    std::vector<nav_msgs::Path>& planned_wall_path)
{
  planner_ = planner_plugin_manager_.getPlugin(plugin);

  geometry_msgs::Polygon prepared_coverage_area;

  // if (!planner_server_helper_->adjustPolygonBoundary(coverage_area, 1.8, prepared_coverage_area)) {
  //   ROS_ERROR("Failed to adjust polygon boundary");
  //   return false;
  // }

  try  {
    planner_->makePlan(coverage_area, holes, start, end, planned_coverage_path, planned_wall_path);
    ROS_INFO("Planner returned a valid plan");
    return true;
  } catch (const std::exception& ex) {
    ROS_ERROR_STREAM("Planner failed with exception: " << ex.what());
    return false;
  }

}

bool PolygonCoverageServer::checkPath(nav_msgs::Path& path)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("Path size is too less, size is %d.", static_cast<int>(path.poses.size()));
    return false;
  }
  else
  {
    path.header.frame_id = "map";

    for (int i = 0; i < path.poses.size(); i++)
    {
      path.poses[i].header.frame_id = "map";
      float yaw = i < path.poses.size() - 1 ? atan2(path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
                                                    path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x) :
                                              atan2(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y,
                                                    path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
      geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(yaw);
      path.poses[i].pose.orientation = qua;
    }
  }
  return true;
}
void PolygonCoverageServer::planPathBetweenPoints(nav_msgs::Path& input_path,
                                                  double max_angle_threshold,
                                                  int num_points_before_after)
{
    if (input_path.poses.size() < 1 + num_points_before_after)
    {
      return;
    }

    for (size_t i = num_points_before_after; i < input_path.poses.size() - num_points_before_after; ++i)
    {
        const geometry_msgs::Pose& current_pose = input_path.poses[i].pose;
        const geometry_msgs::Pose& start_pose = input_path.poses[i - num_points_before_after].pose;

        const geometry_msgs::Pose& goal_pose = input_path.poses[i + num_points_before_after].pose;

        double angle = calculateAngleBetweenPoses(input_path.poses[i - 1].pose, current_pose, input_path.poses[i + 1].pose);

        if (fabs(angle) < max_angle_threshold)
        {
            std::vector<geometry_msgs::PoseStamped> path_poses;

            if (!ompl_ros_ptr_->planPath(start_pose, goal_pose, path_poses, true, true)) {
              ROS_INFO("Dubins planner donnot find valid path, try RS planner");
              path_poses.clear();
              ompl_ros_ptr_->planPath(start_pose, goal_pose, path_poses, false, true);
            }
            // should i set path poses with speed?
            input_path.poses.erase(input_path.poses.begin() + i - num_points_before_after,
                                   input_path.poses.begin() + i + num_points_before_after + 1);

            input_path.poses.insert(input_path.poses.begin() + i - num_points_before_after,
                                    path_poses.begin(),
                                    path_poses.end());

            i += path_poses.size() - 1 - 2 * num_points_before_after;
        }
    }
}
double PolygonCoverageServer::calculateAngleBetweenPoses(const geometry_msgs::Pose& pose1,
                                                         const geometry_msgs::Pose& pose2,
                                                         const geometry_msgs::Pose& pose3)
{
    double angle1 = atan2(pose1.position.y - pose2.position.y, pose1.position.x - pose2.position.x);
    double angle2 = atan2(pose3.position.y - pose2.position.y, pose3.position.x - pose2.position.x);


    double angle = fabs(angle2 - angle1) * 180.0 / M_PI;

    if (angle > 180.0) {
      angle -= 360.0;
    }
    if (angle < -180.0) {
      angle += 360.0;
    }

    return angle;
}
}  // namespace navit_planner
