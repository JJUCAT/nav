/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#include "tf/transform_datatypes.h"
#include <angles/angles.h>
#include <limits>
#include <rrt_ssplanner/rrt_ssplanner.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rrt_ssplanner/nanoflann_port.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(rrt_planner::RrtStarSmartPlanner, nav_core::BaseGlobalPlanner)
//TODO:cmake 依赖添加，完成单独编译

namespace rrt_planner {
RrtStarSmartPlanner::~RrtStarSmartPlanner()
{
  
}


RrtStarSmartPlanner::RrtStarSmartPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
  : costmap_ros_(NULL), costmap_(NULL), world_model_(NULL), initialized_(false)
{
  initialize(name, costmap_ros);
}

void RrtStarSmartPlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_) {
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    world_model_ = std::make_shared<base_local_planner::CostmapModel>(*costmap_);
    tree_ = std::make_shared<rrt_planner::Tree>();

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("max_iterations", max_iterations_, 10000);
    private_nh.param("timeout", timeout_, 3.0);
    private_nh.param("step", step_, 1.0);
    private_nh.param("r_gamma", r_gamma_, 10.0);

    calculateCheckDeltaStep();

    initialized_ = true;
  } else {
    ROS_WARN("This planner has already been initialized... doing nothing");    
  }
}

double RrtStarSmartPlanner::footprintCost(double x_i, double y_i, double theta_i)
{
  return 0.f;
}

bool RrtStarSmartPlanner::makePlan(const geometry_msgs::PoseStamped& start, 
    const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
{
  tree_->Insert(0, start.pose.position);
  for (int i = 1; i < max_iterations_; i ++) {
    geometry_msgs::Point sample = Sample();
    size_t nearest_index = GetNearestPoint(sample);
    geometry_msgs::Point nearest_point = tree_->nodes()->at(nearest_index).point();
    geometry_msgs::Point new_point;
    if (Steer(nearest_point, sample, new_point)) {
      double r = GetNearRadius(i);
      std::vector<size_t> near_index = GetNearPoints(new_point, r);
      tree_->InsertNode(i, new_point, near_index);
    }
  }
  return true;
}

// -------------------- private --------------------

geometry_msgs::Point RrtStarSmartPlanner::Sample()
{
  srand(static_cast<unsigned int>(time(NULL)));
  int rand0 = rand();
  srand(static_cast<unsigned int>(rand0));
  int rand1 = rand();

  double x_size = costmap_->getSizeInMetersX();
  double y_size = costmap_->getSizeInMetersY();
  geometry_msgs::Point p;
  p.x = static_cast<double>(rand0) / RAND_MAX * x_size - x_size / 2;
  p.y = static_cast<double>(rand1) / RAND_MAX * y_size - y_size / 2;
  ROS_INFO("[RRT] rand sample [%f,%f]", p.x, p.y);
  return p;
}

size_t RrtStarSmartPlanner::GetNearestPoint(const geometry_msgs::Point random_point)
{
  ROS_ASSERT_MSG(tree_->nodes()->empty(), "[RRT] samples must be has size.");

  auto kdt = nanoflann_port_ns::NanoflannPort(tree_->nodes());
  nanoflann_port_ns::KDTIndex idx = kdt.FindClosestPoint(random_point);
  return idx.dist;
}

bool RrtStarSmartPlanner::Steer(const geometry_msgs::Point nestest_point,
  const geometry_msgs::Point random_point, geometry_msgs::Point& new_point)
{
  double orient = atan2(random_point.y - nestest_point.y, random_point.x - nestest_point.x);
  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(orient);

  new_point = nestest_point;
  for (double d = 0.0; d <= step_; d+= check_dstep_) {
    new_point.x = new_point.x + check_dstep_ * cos(orient);
    new_point.y = new_point.y + check_dstep_ * sin(orient);
    pose.position = new_point;
    if (IsCollised(pose)) return false;
  }
  return true;
}

bool RrtStarSmartPlanner::IsCollised(const geometry_msgs::Pose p)
{
  double cost = world_model_->footprintCost(
    p.position.x, p.position.y, tf2::getYaw(p.orientation),
    costmap_ros_->getRobotFootprint());

  return cost < 0.0;
}

void RrtStarSmartPlanner::calculateCheckDeltaStep()
{
  double inscribed_radius, circumscribed_radius;
  costmap_2d::calculateMinAndMaxDistances(
    costmap_ros_->getRobotFootprint(), inscribed_radius, circumscribed_radius);
  int n = step_ / inscribed_radius;
  n = n < 1 ? 1 : n;
  check_dstep_ = step_ / n;
}

double RrtStarSmartPlanner::GetNearRadius(size_t n)
{
  ROS_ASSERT_MSG(n <= 1, "[RRT] iterations must be greater than 1.");
  return r_gamma_ * std::sqrt(log(n)/n);
}

std::vector<size_t> RrtStarSmartPlanner::GetNearPoints(
  const geometry_msgs::Point center_point, const double radius)
{
  auto kdt = nanoflann_port_ns::NanoflannPort(tree_->nodes());
  std::vector<nanoflann_port_ns::KDTIndex> idxs;
  kdt.FindClosestPoint(center_point, radius, idxs);

  std::vector<size_t> near_points_index;
  for (auto i : idxs) near_points_index.push_back(i.idx);
  return near_points_index;
}

}; // namespace rrt_planner
