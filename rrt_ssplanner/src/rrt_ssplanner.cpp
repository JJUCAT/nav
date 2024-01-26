/**
 * @copyright Copyright (c) {2022} LMRCAT
 * @author lmrcat (lmr2887@163.com)
 * @date 2024-01-01
 * @brief 
 */
#include "nav_msgs/Path.h"
#include "ros/duration.h"
#include "ros/publisher.h"
#include "tf/transform_datatypes.h"
#include <angles/angles.h>
#include <cmath>
#include <limits>
#include <rrt_ssplanner/rrt_ssplanner.h>
#include <pluginlib/class_list_macros.hpp>
#include <rrt_ssplanner/nanoflann_port.h>
#include <string>

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

    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("max_iterations", max_iterations_, 10000);
    private_nh.param("timeout", timeout_, 3.0);
    private_nh.param("step", step_, 1.0);
    private_nh.param("r_gamma", r_gamma_, 10.0);
    private_nh.param("goal_err", goal_err_, 0.25);
    private_nh.param("r_beacon", r_beacon_, 1.0);
    private_nh.param("biasing_ratio", biasing_ratio_, 1.0);

    nodes_pub_ = private_nh.advertise<visualization_msgs::Marker>("tree_nodes", 1);
    plan_pub_ = private_nh.advertise<nav_msgs::Path>("plan", 1);
    arrows_pub_ = private_nh.advertise<visualization_msgs::Marker>("arrows", 1);
    opt_plan_pub_ = private_nh.advertise<nav_msgs::Path>("opt_plan", 1);

    check_dstep_ = CalculateCheckDeltaStep(step_);

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
  geometry_msgs::Point sample, beacon;
  size_t reach_index;
  auto sampler = Sample(costmap_, biasing_ratio_, r_beacon_);
  double opt_cost = 0.0;

  Insert(0, start.pose.position);
  PubNode(nodes_.at(0).point(), 0);
  ROS_INFO("[RRT] star [%f,%f]", start.pose.position.x, start.pose.position.y);
  for (size_t i = 1; i < max_iterations_;) {
    if (ros::Time::now()-start_time > ros::Duration(timeout_)) break;
    sample = sampler.SamplePoint();
    ROS_INFO("[RRT] <========== iter %lu ==========>", i);
    ROS_INFO("[RRT] sample [%f,%f]", sample.x, sample.y);
    size_t nearest_index = GetNearestPoint(sample);
    ROS_INFO("[RRT] nearest index %lu", nearest_index);
    geometry_msgs::Point nearest_point = nodes_.at(nearest_index).point();
    geometry_msgs::Point new_point;
    ROS_INFO("[RRT] nearest point [%f,%f]", nearest_point.x, nearest_point.y);
    if (Steer(nearest_point, sample, new_point)) {
      ROS_INFO("[RRT] new point [%lu][%f,%f]", i, new_point.x, new_point.y);
      double r = GetNearRadius(i);
      std::vector<size_t> near_index;
      if (i == 1) near_index.push_back(0u);
      else near_index = GetNearPoints(new_point, r);
      if (near_index.empty()) near_index.push_back(nearest_index);
      PubNode(new_point, i);
      Insert(i, new_point);
      Rewire(i, near_index);

      if (IsReach(goal, new_point) && !reach) {
        ROS_INFO("[RRT] goal reach !");
        terminal_ = i;

        reach = true;
        reach_index = i;
        sampler.SetBiasing(i);
        std::vector<geometry_msgs::PoseStamped> tmp_plan;
        GetPlan(tmp_plan);
        PubPlan(plan_pub_, tmp_plan);
      }
      i ++;
    }

    if (reach) {
      double cost = GetPlanCost();
      if (cost - opt_cost > fabs(0.01)) {
        ROS_INFO("[RRT] opt plan.");
        std::vector<geometry_msgs::Point> beacons;
        opt_cost = PathOptimization(beacons);
        sampler.SetBeacons(beacons);
        GetPlan(plan);
        PubPlan(opt_plan_pub_, plan);
        ROS_INFO("[RRT] opt plan cost %f, beacons size %lu",opt_cost, beacons.size());
      }
    }
  }

  if (reach) {
    GetPlan(plan);
    PubPlan(opt_plan_pub_, plan);
    ROS_INFO("[RRT] make plan succeed !");
    while(1);
    return true;
  }

  ROS_ERROR("[RRT] make plan failed !");
  return false;
}

// -------------------- private --------------------
geometry_msgs::Point RrtStarSmartPlanner::Sample::SamplePoint()
{
  if (!reach_) return RandomSample();
  ROS_INFO("[RRT] beacons size %lu, c %lu, b %lu", beacons_.size(), c_, b_);
  if (c_++ >= b_ && !beacons_.empty()) {
    c_ = 0;
    return BeaconSample();
  }
  return RandomSample();
}

void RrtStarSmartPlanner::Sample::SetBiasing(const size_t n)
{
  reach_ = true;

  int x_size = costmap_->getSizeInCellsX();
  int y_size = costmap_->getSizeInCellsY();
  int total = x_size * y_size;
  int b = n * ratio_ / total;
  b_ = std::max(10, std::min(50, b));
  ROS_INFO("[RRT] x %d, y %d, total %d, n %lu, ratio %f, b %d, b_ %lu",
    x_size, y_size, total, n, ratio_, b, b_);
}

void RrtStarSmartPlanner::Sample::SetBeacons(const std::vector<geometry_msgs::Point> beacons)
{
  beacons_ = beacons;
}

geometry_msgs::Point RrtStarSmartPlanner::Sample::RandomSample()
{
  unsigned int now = clock();
  srand(static_cast<unsigned int>(now));
  int rand0 = rand();
  srand(static_cast<unsigned int>(rand0));
  int rand1 = rand();

  double x_size = costmap_->getSizeInMetersX();
  double y_size = costmap_->getSizeInMetersY();
  geometry_msgs::Point p;
  p.x = static_cast<double>(rand0) / RAND_MAX * x_size + costmap_->getOriginX();
  p.y = static_cast<double>(rand1) / RAND_MAX * y_size + costmap_->getOriginY();
  return p;
}

geometry_msgs::Point RrtStarSmartPlanner::Sample::BeaconSample()
{
  unsigned int now = clock();
  srand(static_cast<unsigned int>(now));
  int rand0 = rand();
  size_t i = rand0 % beacons_.size();
  geometry_msgs::Point beacon = beacons_.at(i);

  srand(static_cast<unsigned int>(rand0));
  int rand1 = rand();
  srand(static_cast<unsigned int>(rand1));
  int rand2 = rand();
  double x_size = costmap_->getSizeInMetersX();
  double y_size = costmap_->getSizeInMetersY();
  geometry_msgs::Point p;
  p.x = static_cast<double>(rand1) / RAND_MAX * r_ * 2 + beacon.x - r_;
  p.y = static_cast<double>(rand2) / RAND_MAX * r_ * 2 + beacon.y - r_;
  ROS_INFO("[RRT] beacons size %lu, beacon [%f,%f], sample [%f,%f]",
    beacons_.size(), beacon.x, beacon.y, p.x, p.y);
  return p;
}

size_t RrtStarSmartPlanner::GetNearestPoint(const geometry_msgs::Point random_point)
{
  ROS_ASSERT_MSG(!nodes_.empty(), "[RRT] samples must be has size.");

  auto kdt = nanoflann_port_ns::NanoflannPort(&nodes_);
  nanoflann_port_ns::KDTIndex idx = kdt.FindClosestPoint(random_point);
  return idx.idx;
}

bool RrtStarSmartPlanner::Steer(const geometry_msgs::Point nestest_point,
  const geometry_msgs::Point random_point, geometry_msgs::Point& new_point)
{
  double orient = atan2(random_point.y - nestest_point.y, random_point.x - nestest_point.x);
  double dist = std::hypot(random_point.y - nestest_point.y, random_point.x - nestest_point.x);
  geometry_msgs::Pose pose;
  pose.orientation = tf::createQuaternionMsgFromYaw(orient);

  new_point = nestest_point;
  for (double d = 0.0; d <= step_; d += check_dstep_) {
    d = d >= dist ? dist : d;
    new_point.x = nestest_point.x + d * cos(orient);
    new_point.y = nestest_point.y + d * sin(orient);
    pose.position = new_point;
    if (IsCollised(pose)) return false;
    if (d >= dist) break;
  }
  return true;
}

void RrtStarSmartPlanner::Insert(const size_t index, const geometry_msgs::Point point)
{
  nodes_.insert(std::make_pair(index, rrt_planner::Node(index, point)));
}

void RrtStarSmartPlanner::Rewire(const size_t node, const std::vector<size_t>& near_points)
{
  auto point = nodes_.at(node).point();
  double cost_min = std::numeric_limits<double>::max();
  size_t parent_index;
  double c2p_cost;
  for (auto p : near_points) {
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x);
    double total_cost = new_cost + TrajectoryCost(&nodes_.at(p));
    if (total_cost < cost_min) {
      cost_min = total_cost;
      parent_index = p;
      c2p_cost = new_cost;
    }
  }
  nodes_.at(node).set_cost(c2p_cost);
  nodes_.at(node).set_parent(&nodes_.at(parent_index));
  ROS_INFO("[RRT] insert point [%lu], child [%f,%f] to parent [%f,%f]",
    node, point.x, point.y, nodes_.at(parent_index).point().x, nodes_.at(parent_index).point().y);
  PubArrow(node, point, nodes_.at(parent_index).point(), 1, 1, 0);

  for (auto p : near_points) {
    if (p == parent_index) continue;
    if (nodes_.at(p).is_root()) continue;
    double original_cost = TrajectoryCost(&nodes_.at(p));
    auto pp = nodes_.at(p).point();
    double new_cost = std::hypot(pp.y-point.y, pp.x-point.x);
    if (original_cost > new_cost+cost_min) {
      if (Connect(pp, point)) {
        nodes_.at(p).set_parent(&nodes_.at(node));
        nodes_.at(p).set_cost(new_cost);
        ROS_INFO("[RRT] rewire index [%lu], child [%f,%f] to parent [%f,%f]",
          p, nodes_.at(p).point().x, nodes_.at(p).point().y, nodes_.at(node).point().x, nodes_.at(node).point().y);
        PubArrow(p, nodes_.at(p).point(), nodes_.at(node).point(), 1, 0, 1);
      }
    }
  }
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

double RrtStarSmartPlanner::CalculateCheckDeltaStep(const double dist)
{
  double inscribed_radius, circumscribed_radius;
  costmap_2d::calculateMinAndMaxDistances(
    costmap_ros_->getRobotFootprint(), inscribed_radius, circumscribed_radius);
  int n = dist / inscribed_radius;
  n = n < 1 ? 1 : n;
  double step = dist / n;
  return step;
}

rrt_planner::Node* RrtStarSmartPlanner::GetTerminal()
{
  return &nodes_.at(terminal_);
}

double RrtStarSmartPlanner::TrajectoryCost(rrt_planner::Node* node)
{
  double cost = 0.0;
  auto p = node;
  while (!p->is_root()) {
    cost += p->cost();
    p = p->parent();
  }
  return cost;
}

std::vector<geometry_msgs::Point> RrtStarSmartPlanner::GeTrajectory(rrt_planner::Node* node)
{
  std::vector<geometry_msgs::Point> points;
  auto p = node;
  do {
    points.push_back(p->point());
    p = p->parent();
  } while (!p->is_root());
  points.push_back(p->point());
  return std::move(points);
}

double RrtStarSmartPlanner::GetNearRadius(size_t n)
{
  ROS_ASSERT_MSG(n >= 1, "[RRT] iterations must be greater than 1.");
  return r_gamma_ * std::sqrt(log(n)/n);
}

std::vector<size_t> RrtStarSmartPlanner::GetNearPoints(
  const geometry_msgs::Point center_point, const double radius)
{
  auto kdt = nanoflann_port_ns::NanoflannPort(&nodes_);
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
  std::vector<geometry_msgs::Point> poses = GeTrajectory(GetTerminal());
  plan.clear();
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
  v_trajectory.color.r = 1;
  v_trajectory.color.g = 0;
  v_trajectory.color.b = 0;
  v_trajectory.color.a = 1;
  v_trajectory.pose.position.x = point.x;
  v_trajectory.pose.position.y = point.y;
  v_trajectory.pose.position.z = 0;
  v_trajectory.pose.orientation.x = 0;
  v_trajectory.pose.orientation.y = 0;
  v_trajectory.pose.orientation.z = 0;
  v_trajectory.pose.orientation.w = 1.0;
  v_trajectory.scale.x = 0.1;
  v_trajectory.scale.y = 0.1;
  v_trajectory.scale.z = 0.1;
  v_trajectory.ns = "rrt";
  // v_trajectory.text = std::to_string(index).c_str();
  v_trajectory.type = visualization_msgs::Marker::SPHERE;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.id = index;
  nodes_pub_.publish(v_trajectory);
}

void RrtStarSmartPlanner::PubPlan(
  const ros::Publisher pub, const std::vector<geometry_msgs::PoseStamped>& points)
{
  nav_msgs::Path plan;
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = ros::Time::now();
  plan.poses.reserve(points.size());
  for (auto p : points) plan.poses.emplace_back(p);
  pub.publish(plan);
}

void RrtStarSmartPlanner::PubArrow(const size_t index,
  const geometry_msgs::Point& child, const geometry_msgs::Point& parent,
  const double r, const double g, const double b)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = costmap_ros_->getGlobalFrameID();
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r;
  v_trajectory.color.g = g;
  v_trajectory.color.b = b;
  v_trajectory.color.a = 1;
  v_trajectory.pose.position.x = child.x;
  v_trajectory.pose.position.y = child.y;
  v_trajectory.pose.position.z = 0;
  double yaw = atan2(parent.y-child.y, parent.x-child.x);
  v_trajectory.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  double dist = std::hypot(parent.y-child.y, parent.x-child.x);
  v_trajectory.scale.x = dist - 0.05;
  v_trajectory.scale.y = 0.03;
  v_trajectory.scale.z = 0.05;
  v_trajectory.type = visualization_msgs::Marker::ARROW;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.id = index;
  arrows_pub_.publish(v_trajectory);
}

bool RrtStarSmartPlanner::Connect(const geometry_msgs::Point& s, const geometry_msgs::Point& e)
{
  double dist = std::hypot(e.y-s.y, e.x-s.x);
  double step = CalculateCheckDeltaStep(dist);
  double orient = atan2(e.y-s.y, e.x-s.x);
  geometry_msgs::Pose pose;
  pose.position = s;
  pose.orientation = tf::createQuaternionMsgFromYaw(orient);
  for (double d = 0.0; d <= dist; d += step) {
    pose.position.x = s.x + d * cos(orient);
    pose.position.y = s.y + d * sin(orient);
    if (IsCollised(pose)) return false;
  }
  return true;
}

double RrtStarSmartPlanner::PathOptimization(std::vector<geometry_msgs::Point>& beacons)
{
  auto c = GetTerminal();
  auto p = c->parent();
  do {
    bool collised = false;
    if (!Connect(c->point(), p->point())) {
      beacons.push_back(p->point());
      collised = true;
    }

    if (!collised) c->set_parent(p);
    if (p->is_root()) break;

    if (collised) {
      c = p;
      p = c->parent();
    } else {
      p = p->parent();
    }
  } while (1);
  return TrajectoryCost(GetTerminal());
}

double RrtStarSmartPlanner::GetPlanCost()
{
  return TrajectoryCost(GetTerminal());
}

}; // namespace rrt_planner
