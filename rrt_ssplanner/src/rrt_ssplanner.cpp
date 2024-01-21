/**
 * @copyright Copyright (c) {2022} JUCAT
 * @author jucat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#include "nav_msgs/Path.h"
#include "ros/duration.h"
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
    private_nh.param("goal_err", goal_err_, 0.25);

    nodes_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree_nodes", 1);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);

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
  ros::Time start_time = ros::Time::now();
  bool reach = false;

  tree_->Insert(0, start.pose.position);
  PubNode(tree_->nodes()->at(0).point(), 0);
  std::cout << "star [" << start.pose.position.x << "," << start.pose.position.y << "]" << std::endl;
  for (int i = 1; i < max_iterations_;) {
    if (ros::Time::now()-start_time > ros::Duration(timeout_)) break;
    geometry_msgs::Point sample = Sample();
    std::cout << "iter " << i << ", sample [" << sample.x << "," << sample.y << "]" << std::endl;
    size_t nearest_index = GetNearestPoint(sample);
    std::cout << "nearest index:" << nearest_index << std::endl;
    geometry_msgs::Point nearest_point = tree_->nodes()->at(nearest_index).point();
    geometry_msgs::Point new_point;
    std::cout << "nearest point [" << nearest_point.x << "," << nearest_point.y << "]" << std::endl;
    if (Steer(nearest_point, sample, new_point)) {
      std::cout << "new point [" << new_point.x << "," << new_point.y << "]" << std::endl;
      double r = GetNearRadius(i);
      std::cout << "near radius " << r << std::endl;
      std::vector<size_t> near_index;
      if (i == 1) near_index.push_back(0u);
      else near_index = GetNearPoints(new_point, r);
      std::cout << "near index size is " << near_index.size() << std::endl;
      tree_->InsertNode(i, new_point, near_index);
      PubNode(new_point, i);
      i ++;
      if (IsReach(goal, new_point)) {
        std::cout << "reach goal !" << std::endl;
        reach = true;
        break;
      }
    }
  }
  std::cout << "finish search." << std::endl;
  if (reach) {
    GetPlan(plan);
    PubPlan(plan);
    ROS_INFO("[RRT] make plan succeed !");
    return true;
  }

  ROS_ERROR("[RRT] make plan failed !");
  return false;
}

// -------------------- private --------------------

geometry_msgs::Point RrtStarSmartPlanner::Sample()
{
  unsigned int now = clock();
  srand(static_cast<unsigned int>(now));
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
  ROS_ASSERT_MSG(!tree_->nodes()->empty(), "[RRT] samples must be has size.");

  auto kdt = nanoflann_port_ns::NanoflannPort(tree_->nodes());
  nanoflann_port_ns::KDTIndex idx = kdt.FindClosestPoint(random_point);
  return idx.idx;
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
  bool is_collised = cost < 0.0;
  if (is_collised) ROS_WARN("[RRT] point [%f, %f] collised", p.position.x, p.position.y);
  return is_collised;
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
  ROS_ASSERT_MSG(n >= 1, "[RRT] iterations must be greater than 1.");
  return r_gamma_ * std::sqrt(log(n)/n);
}

std::vector<size_t> RrtStarSmartPlanner::GetNearPoints(
  const geometry_msgs::Point center_point, const double radius)
{
  auto kdt = nanoflann_port_ns::NanoflannPort(tree_->nodes());
  std::vector<nanoflann_port_ns::KDTIndex> idxs;
  kdt.FindPointsInRadius(center_point, radius, idxs);

  std::vector<size_t> near_points_index;
  for (auto i : idxs) near_points_index.push_back(i.idx);
  return near_points_index;
}

bool RrtStarSmartPlanner::IsReach(const geometry_msgs::PoseStamped& goal, const geometry_msgs::Point point)
{
  double dist_err = pow(goal.pose.position.y-point.y, 2) + pow(goal.pose.position.x-point.x, 2);
  bool is_reach = dist_err <= goal_err_;
  if (is_reach) ROS_INFO("[RRT] goal reach, err %f, limit %f", dist_err, goal_err_);
  return is_reach;
}

size_t RrtStarSmartPlanner::GetPlan(std::vector<geometry_msgs::PoseStamped>& plan)
{
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = costmap_ros_->getGlobalFrameID();
  pose.header.stamp = ros::Time::now();
  std::vector<geometry_msgs::Point> poses =
    tree_->GeTrajectory(&tree_->nodes()->at(tree_->nodes()->size()-1));
  plan.reserve(poses.size());
  for (auto p : poses) {
    pose.pose.position = p;
    plan.emplace_back(pose);
  }
  return plan.size();
}

void RrtStarSmartPlanner::PubNode(const geometry_msgs::Point& point, const size_t index)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = costmap_ros_->getGlobalFrameID();
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = 0.8;
  v_trajectory.color.g = 0;
  v_trajectory.color.b = 0;
  v_trajectory.color.a = 0.8;
  v_trajectory.pose.position.x = point.x;
  v_trajectory.pose.position.y = point.y;
  v_trajectory.pose.orientation.w = 1.0;
  v_trajectory.scale.x = 0.1;
  v_trajectory.scale.y = 0.1;
  v_trajectory.scale.z = 0.1;
  v_trajectory.type = visualization_msgs::Marker::SPHERE;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.id = index;
  nodes_pub_.publish(v_trajectory);
}

void RrtStarSmartPlanner::PubPlan(const std::vector<geometry_msgs::PoseStamped>& points)
{
  nav_msgs::Path plan;
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = ros::Time::now();
  plan.poses.reserve(points.size());
  for (auto p : points) plan.poses.emplace_back(p);
  plan_pub_.publish(plan);
}

}; // namespace rrt_planner
