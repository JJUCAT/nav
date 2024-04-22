/**
 * @copyright Copyright (c) {2022} LZY
 * @author LZY (linziyan@yijiahe.com)
 * @date 2023-11-08
 * @brief
 */

#include <local_planner/local_planner.h>
#include <local_planner/fast_triangular_fun.h>
#include <iostream>
#include <memory>
#include "Eigen/src/Core/Matrix.h"
#include "angles/angles.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/duration.h"
#include "tf/transform_datatypes.h"
#include "tf2/utils.h"
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/ransac.h>

namespace local_planner {

// 查表实现快速三角函数，屏蔽定义使用原函数
#define sin(x) fast_triangular::fast_sin((x))
#define cos(x) fast_triangular::fast_cos((x))
#define M_2PI fast_triangular::M_2PI

LocalPlanner::LocalPlanner(std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros,
                           std::shared_ptr<tf2_ros::Buffer>& tf):
  nh_("/"), local_nh_("~/local_planner/") {
  costmap_ros_ = costmap_ros;
  tf_ = tf;
  cfg_ = std::make_shared<Configuration>(local_nh_);
  security_detector_ = std::make_shared<SecurityDetector>(
    cfg_->BASE_COLLISION_COST, cfg_->HEAD_COLLISION_COST, cfg_->COLLISION_COST_OFFSET,
    cfg_->BASE_SECURITY_COST, cfg_->HEAD_SECURITY_COST, cfg_->SECURITY_IGNORE_DIST);
  slp_.set_sampling_params(StateLatticePlanner::SamplingParams(cfg_->N_P, cfg_->N_H, cfg_->MAX_ALPHA, cfg_->MAX_PSI));
  slp_.set_optimization_params(cfg_->MAX_ITERATION, cfg_->OPTIMIZATION_TOLERANCE);
  slp_.set_vehicle_params(cfg_->WHEEL_RADIUS, cfg_->TREAD);
  slp_.set_motion_params(cfg_->MAX_ACCELERATION, cfg_->MAX_YAWRATE, cfg_->MAX_D_YAWRATE);
  slp_.set_target_velocity(cfg_->TARGET_VELOCITY);

  candidate_trajectories_pub_ =
    local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  candidate_trajectories_no_collision_pub_ =
    local_nh_.advertise<visualization_msgs::MarkerArray>("no_collision_trajectories", 1);
  cost_trajectories_pub_ = local_nh_.advertise<visualization_msgs::MarkerArray>("cost_trajectories", 1);
  selected_trajectory_pub_ =
    local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  goal_sampling_pub_ = local_nh_.advertise<geometry_msgs::PoseArray>("goal_sampling", 1);
  goal_in_robot_frame_pub_ = local_nh_.advertise<geometry_msgs::PoseStamped>("goal_in_robot_frame", 1);
  check_plan_pub_ = local_nh_.advertise<nav_msgs::Path>("check_plan", 1);
  ref_plan_pub_ = local_nh_.advertise<nav_msgs::Path>("ref_plan", 1);
  local_plan_pub_ = local_nh_.advertise<nav_msgs::Path>("local_plan", 1);
  sample_start_pub_ = local_nh_.advertise<visualization_msgs::Marker>("sample_start", 1);
  sample_jump_pub_ = local_nh_.advertise<visualization_msgs::Marker>("sample_jump", 1);
  sample_end_pub_ = local_nh_.advertise<visualization_msgs::Marker>("sample_end", 1);

  odom_sub_ = local_nh_.subscribe("/odom", 1, &LocalPlanner::OdomCallback, this);
  critic_ = std::make_shared<StateLatticePlanner::Critic>(
    cfg_->DIST_ERR, cfg_->YAW_ERR, cfg_->RIGHT_PUNISHMENT, cfg_->DIST_SCALE, cfg_->YAW_SCALE, cfg_->RIGHT_PUNISHMENT_SCALE);
  slp_.load_lookup_table(cfg_->LOOKUP_TABLE_FILE_NAME, xyyaw_table_);
}

void LocalPlanner::SetPlan(const nav_msgs::Path& plan) {
  ROS_INFO("[LP] set plan.");
  odom_udpated_ = false;
  finished_ = false;
  first_pub_ = false;
  if (plan.poses.empty()) {
    ROS_ERROR("[LP] plan can't be empty !");
    throw "plan can't empty !";
  }

  dispatcher_.Reset(ros::Duration(cfg_->DISPATCHER_WAIT), ros::Duration(cfg_->DISPATCHER_SLEEP));
  wait_ = false;
  sand_clock_.Load(cfg_->BLOCK_TIMEOUT);

  final_plan_ = false;
  check_dist_ = cfg_->CHECK_DISTANCE;
  pub_timer_ = ros::Time(0);

  ClearRefPlan();
  ClearLocalPlan();
  ClearSamplingIndex();

  ref_plan_.plan = plan;
  if (plan.header.frame_id == "") {
    ROS_WARN("[LP] global plan frame id is empty, set frame id [map].");
    ref_plan_.plan.header.frame_id = "map";
  }
  ref_plan_pub_.publish(ref_plan_.plan);
}

bool LocalPlanner::IsFinished() {
  if (finished_) return true;  
  if (!ref_plan_.plan.poses.empty()) {
    bool finished = ref_plan_.idx == ref_plan_.plan.poses.size()-1;
    bool sampling_finished = sampling_idx_.idx == ref_plan_.plan.poses.size()-1;
    if (sampling_finished) {
      ROS_WARN("[LP] sampling finish local plan.");
      finished = true;
    }
    if (finished) {
      ROS_INFO("[LP] finish local plan.");
      wait_ = false; // 局部规划结束，停障功能也要关闭了      
    }
    finished_ = finished;
    return finished;
  }
  return false;
}

void LocalPlanner::SetWaitConfig(double terminal_check, double terminal_wait)
{
  std::unique_lock<std::mutex> lock(wait_config_mutex_);
  cfg_->TERMINAL_CHECK = terminal_check;
  cfg_->TERMINAL_WAIT = terminal_wait;
}

void LocalPlanner::SetCollisionDetectLevel(const int level)
{
  ROS_INFO("[LP] collision detect level : %d", level);
  switch (level) {
    case 0:
      security_detector_->SetCollisionCost(
        cfg_->BASE_SECURITY_COST, cfg_->HEAD_SECURITY_COST);
      break;
    case 1:
      security_detector_->SetCollisionCost(
        cfg_->BASE_COLLISION_COST, cfg_->HEAD_COLLISION_COST);
      break;
    default:
      security_detector_->SetCollisionCost(
        cfg_->BASE_SECURITY_COST, cfg_->HEAD_SECURITY_COST);
      break;
  }
}

bool LocalPlanner::GetLocalPlan(nav_msgs::Path& plan) {
  if (ref_plan_.plan.poses.empty()) return true;
  if (finished_) return false;
  WaitForOdom();

  double start = ros::Time::now().toSec();

  // 更新基础信息，检查是否走丢
  bool is_lost = false;
  if (!Update(is_lost)) {
    ROS_ERROR("[LP] update failed !");
    return false;
  }

  // 采样
  Eigen::Vector3d goal;
  std::vector<Eigen::Vector3d> states;
  size_t ref_sampling_num = Sampling(goal , states);

  // 检查路径是否有堵塞
  size_t local_plan_end = 0, ref_plan_end = 0;
  bool is_collid_soon = false;
  bool plan_blocked = IsPlanBlocked(
    check_dist_, plan_connected_, local_plan_end, ref_plan_end, is_collid_soon);
  // if (!plan_blocked && plan_connected_) ref_plan_end = ref_plan_.plan.poses.size()-1; // 发完参考路径
  // else if (!plan_blocked) local_plan_end = local_plan_.plan.poses.size()-1;
  VizCheckPlan(plan_connected_, local_plan_end, ref_plan_end);

  // 调度器检查是否要停下等待
  if (Wait(is_collid_soon)) {
    ROS_INFO("[LP] has checked obstacle, wait ...");
    std::unique_lock<std::mutex> lock(wait_mutex_);
    if (!wait_) sand_clock_.Overturn();
    wait_ = true;
    return false;
  }

  // 如果局部规划走了一段距离了，就可以再更新一下
  bool update_local_plan = false;
  // if (!plan_connected_ && local_plan_.plan.poses.size() > 0) {
  //   update_local_plan = IsNeedToUpdateLocalPlan(check_dist_/3);
  // }

  ROS_INFO("[LP] plan block %d, is lost %d, update local plan %d, plan connected %d, "
    "final plan %d, local plan end: %lu, ref plan end: %lu",
    plan_blocked, is_lost, update_local_plan, plan_connected_, final_plan_, local_plan_end, ref_plan_end);

  if (HasPubFinalPlan()) return false;

  // 更新绕障路径
  bool local_plan = false;
  if (plan_blocked || is_lost || update_local_plan) {
    auto GenerateTrajectory = std::mem_fn(&LocalPlanner::GenerateTrajectoriesToSample);
    double relative_direction = atan2(goal(1), goal(0)); // 目标点方位判断
    if (goal.segment(0, 2).norm() < 0.25) { // 机器接近参考路径的目标点了，怎么处理 ?
      // ClearLocalPlan();
    } else if (fabs(relative_direction) > cfg_->TURN_DIRECTION_THRESHOLD) { // 目标点在机器后方，不好规划了
      ROS_WARN("[LP] goal in the rear !");
      GenerateTrajectory = std::mem_fn(&LocalPlanner::GenerateDubinsToSample);
    }
    // 根据采样生成多条轨迹
    std::vector<MotionModelDiffDrive::Trajectory> trajectories;
    if (!GenerateTrajectory(this, states, ref_sampling_num, trajectories)) {
      ROS_ERROR("[LP] generate trajectories failed !");
      return false;
    }
    // 挑选轨迹，更新局部路径
    if (!GetBetterTrajectories(trajectories, goal)) {
      ROS_ERROR("[LP] can not find local plan !");
      SamplingJumpForward();
      return false;
    }
    UpdatePubPlanIndex(local_plan_end, ref_plan_end, plan_connected_, check_dist_/2);
    // if (plan_connected_) ref_plan_end = ref_plan_.plan.poses.size()-1; // 发完参考路径
    ROS_INFO("[LP] local plan success, plan_connected %d, local plan end: %lu, ref plan end: %lu",
      plan_connected_, local_plan_end, ref_plan_end);
    local_plan = true;
  }

  // 是否发布终点路径了
  CheckTerminal(ref_plan_end);

  if ((PubLocalPlan()&&(!plan_blocked)&&(!is_lost)&&(!update_local_plan)) || local_plan) {
  // if (local_plan || !first_pub_) {
    first_pub_ = true;
    pub_timer_ = ros::Time::now();
    // 按检查过的局部路径和参考路径发布局部规划
    MergePlan(plan_connected_, local_plan_end, ref_plan_end, plan);
    local_plan_pub_.publish(plan);
    ROS_INFO("[LP] state lattice planner pub local plan (%lu) speed time%f s.",
      plan.poses.size(), ros::Time::now().toSec() - start);
    return true;
  }

  // 频繁更新局部路径对控制器不友好，这里返回失败不更新路径，不影响控制器
  return false;
}


// -------------------- private --------------------
void LocalPlanner::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  auto odom = *msg;
  ros::Time now = ros::Time::now();
  odom.header.stamp = now;
  if ((now - cur_odom_.header.stamp).toSec() < 0.5) {
    double alpha = 0.5;
    cur_odom_.twist.twist.linear.x = alpha * odom.twist.twist.linear.x +
      (1-alpha) * cur_odom_.twist.twist.linear.x;
    cur_odom_.twist.twist.linear.y = alpha * odom.twist.twist.linear.y +
      (1-alpha) * cur_odom_.twist.twist.linear.y;
    cur_odom_.twist.twist.angular.z = alpha * odom.twist.twist.angular.z +
      (1-alpha) * cur_odom_.twist.twist.angular.z;
  }
  cur_odom_ = *msg;
}

bool LocalPlanner::UpdateFrame() {
  std::string map_frame = costmap_ros_->getGlobalFrameID();
  std::string robot_frame = costmap_ros_->getBaseFrameID();
  std::string plan_frame = ref_plan_.plan.header.frame_id;
  ROS_DEBUG("[LP] map frame: %s, robot frame: %s, plan frame: %s",
    map_frame.c_str(), robot_frame.c_str(), plan_frame.c_str());

  // ref_plan -> map frame
  try{
    plan_to_map_transform_ = tf_->lookupTransform(map_frame, plan_frame, ros::Time(0));
    plan_in_map_frame_(0) = plan_to_map_transform_.transform.translation.x;
    plan_in_map_frame_(1) = plan_to_map_transform_.transform.translation.y;
    plan_in_map_frame_(2) = tf2::getYaw(plan_to_map_transform_.transform.rotation);
    ROS_DEBUG_STREAM("[LP] ref plan -> map tf:" << std::endl << plan_to_map_transform_ << std::endl);
  } catch (const tf2::TransformException& ex) {
    ROS_ERROR("[LP] tf listen ref plan -> map failed, %s", ex.what());
    return false;
  }

  // ref_plan -> robot frame
  try{
    plan_to_robot_transform_ = tf_->lookupTransform(robot_frame, plan_frame, ros::Time(0));
    plan_in_robot_frame_(0) = plan_to_robot_transform_.transform.translation.x;
    plan_in_robot_frame_(1) = plan_to_robot_transform_.transform.translation.y;
    plan_in_robot_frame_(2) = tf2::getYaw(plan_to_robot_transform_.transform.rotation);
    ROS_DEBUG_STREAM("[LP] ref plan -> robot tf:" << std::endl << plan_to_robot_transform_ << std::endl);
  } catch (tf2::TransformException& ex){
    ROS_ERROR("[LP] tf listen ref plan -> robot failed, %s", ex.what());
    return false;
  }

  // robot -> map frame
  try{
    robot_to_map_transform_ = tf_->lookupTransform(map_frame, robot_frame, ros::Time(0));
    robot_in_map_frame_(0) = robot_to_map_transform_.transform.translation.x;
    robot_in_map_frame_(1) = robot_to_map_transform_.transform.translation.y;
    robot_in_map_frame_(2) = tf2::getYaw(robot_to_map_transform_.transform.rotation);
    ROS_DEBUG_STREAM("[LP] robot -> map tf:" << std::endl << robot_to_map_transform_ << std::endl);
  } catch (tf2::TransformException& ex){
    ROS_ERROR("[LP] tf listen robot -> map failed, %s", ex.what());
    return false;
  }

  return true;
}

bool LocalPlanner::UpdateMap() {
  if (costmap_.use_count()>0) {costmap_.reset();}
  auto map_mutex = costmap_ros_->getCostmap()->getMutex();
  boost::recursive_mutex::scoped_lock lock(*map_mutex);
  costmap_ = std::make_shared<navit_costmap::Costmap2D>(*costmap_ros_->getCostmap());
  return true;
}

bool LocalPlanner::UpdateRobotPose() {
  if (!costmap_ros_->getRobotPose(robot_pose_)) {
    ROS_ERROR("[LP] update robot pose failed !");
    return false;
  }
  return true;
}

bool LocalPlanner::UpdatePlanIndex(TrackingPath& plan, const double rang, const double check_dist, bool use_closest_idx)
{
  bool find = false;
  double check_sum = 0.f;
  geometry_msgs::PoseStamped p;
  size_t min_i;
  double min_dist = std::numeric_limits<double>::max();
  for (size_t i = plan.idx, j = i; i < plan.plan.poses.size(); j = i, i ++) {
    check_sum += Poses2DDistance(plan.plan.poses.at(j), plan.plan.poses.at(i));
    if (check_sum >= check_dist) break;
    p = plan.plan.poses.at(i);
    if (plan.plan.header.frame_id != robot_pose_.header.frame_id) {
      PoseTransform(plan_to_map_transform_, p, p);
      // ROS_INFO("[LP] robot pose [%f,%f,%f]",
      //   robot_pose_.pose.position.x, robot_pose_.pose.position.y, tf2::getYaw(robot_pose_.pose.orientation));
      // ROS_INFO("[LP] tf pose, before [%f,%f,%f], after [%f,%f,%f]",
      //   plan.plan.poses.at(i).pose.position.x, plan.plan.poses.at(i).pose.position.y, tf2::getYaw(plan.plan.poses.at(i).pose.orientation),
      //   p.pose.position.x, p.pose.position.y, tf2::getYaw(p.pose.orientation));
    }
    float d = Poses2DDistance(robot_pose_, p);
    // ROS_INFO("[LP] d %f", d);
    if (d < rang) {
      find = true;
      plan.idx = i;
      if (d < min_dist) {
        min_dist = d;
        min_i = i;
      }
    }
    else if (find) break;
  }
  if (use_closest_idx && find) plan.idx = min_i;
  return find;
}

bool LocalPlanner::UpdateLocalPlanIndex(const double rang) {
  return UpdatePlanIndex(local_plan_, rang, check_dist_);
}

bool LocalPlanner::UpdateRefPlanIndex(const double rang) {
  return UpdatePlanIndex(ref_plan_, rang, check_dist_, true);
}

double LocalPlanner::GetShadow(const bool shadow_on_refplan)
{
  geometry_msgs::PoseStamped sample_pose = ref_plan_.plan.poses.at(sampling_idx_.idx);
  Eigen::Vector3d refpose_in_map, reforient_in_map;
  PoseTransform(plan_to_map_transform_, sample_pose, refpose_in_map);
  reforient_in_map << refpose_in_map(0) + 1 * cos(refpose_in_map(2)),
                      refpose_in_map(1) + 1 * sin(refpose_in_map(2)),
                      refpose_in_map(2);
  Eigen::Vector2d refpose_vector(reforient_in_map(0)-refpose_in_map(0),
                                 reforient_in_map(1)-refpose_in_map(1));

  double v = cur_odom_.twist.twist.linear.x;
  if (v < 0.01) v = 0;
  else if (v > 1.0) v = 1.0;
  double norm = v / cfg_->HZ;
  Eigen::Vector3d robot_in_map, robotorient_in_map;
  robot_in_map << robot_pose_.pose.position.x,
                  robot_pose_.pose.position.y,
                  tf2::getYaw(robot_pose_.pose.orientation);
  robotorient_in_map << robot_in_map(0) + norm * cos(robot_in_map(2)),
                        robot_in_map(1) + norm * sin(robot_in_map(2)),
                        robot_in_map(2);
  Eigen::Vector2d robot_vector(robotorient_in_map(0)-robot_in_map(0),
                               robotorient_in_map(1)-robot_in_map(1));

  double s0 , s1;
  double scale = log(Poses2DDistance(robot_pose_, refpose_in_map)+1.0) * cfg_->SAMPLE_SHADOW_K + 1.0;
  double max = norm * scale;
  s0 = robot_vector.dot(refpose_vector)/refpose_vector.norm();
  s1 = refpose_vector.dot(robot_vector)/robot_vector.norm();
  ROS_INFO("[LP] norm %f, max %f, shadow on ref plan %f, shadow on move %f", norm, max, s0, s1);
  s0 = std::max(0.0, std::min(max, s0));
  s1 = std::max(0.0, std::min(max, s1));
  ROS_INFO("[LP] after filter, shadow on refplan %d, shadow on ref plan %f, shadow on move %f",
    shadow_on_refplan, s0, s1);
  return shadow_on_refplan ? s0 : s1;
}

bool LocalPlanner::InsideSamplingIndexCircle(const double circle_r)
{
  if (sampling_idx_.idx == sampling_idx_.jump || sampling_idx_.jump == sampling_idx_.end)
    return false;

  auto SamplingIndex2PCLPoint = [&](const geometry_msgs::PoseStamped p) -> pcl::PointXYZ {
    pcl::PointXYZ point;
    point.x = p.pose.position.x;
    point.y = p.pose.position.y;
    point.z = 0.0;
    return point;
  };
  // std::cout << "sample index, idx:" << sampling_idx_.idx << ", jump:" << sampling_idx_.jump << ", end:" << sampling_idx_.end << std::endl;
  // pcl 三点拟合圆
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  cloud->points.push_back(SamplingIndex2PCLPoint(ref_plan_.plan.poses.at(sampling_idx_.idx)));
  cloud->points.push_back(SamplingIndex2PCLPoint(ref_plan_.plan.poses.at(sampling_idx_.jump)));
  cloud->points.push_back(SamplingIndex2PCLPoint(ref_plan_.plan.poses.at(sampling_idx_.end)));
  pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr model_circle2D (
    new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud));
  pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_circle2D);
  ransac.setDistanceThreshold(0.01);
  ransac.computeModel();
  std::vector<int> inliers;
  ransac.getInliers(inliers);

  if (inliers.empty()) return false;

  Eigen::VectorXf modelParas;
  ransac.getModelCoefficients(modelParas);
  ROS_INFO("[LP] sample index circle in ref plan frame, centor [%f, %f], radius %f.",
    modelParas(0), modelParas(1), modelParas(2));

  // 采样区间是弧形
  if (modelParas(2) < circle_r) {
    Eigen::Vector3d centor(modelParas(0), modelParas(1), 0.0);
    Eigen::Vector3d centor_in_robot, sample_in_robots;
    PoseTransform(plan_to_map_transform_, centor, centor_in_robot);
    PoseTransform(plan_to_map_transform_, ref_plan_.plan.poses.at(sampling_idx_.idx), sample_in_robots);
    Eigen::Vector2d c2p(sample_in_robots(0)-centor_in_robot(0), sample_in_robots(1)-centor_in_robot(1));
    Eigen::Vector2d r2p(robot_pose_.pose.position.x-centor_in_robot(0), robot_pose_.pose.position.y-centor_in_robot(1));
    if (c2p.dot(r2p) >= 0) {
      ROS_INFO("[LP] robot inside sample index circle.");
      return true;
    }
  }
  ROS_INFO("[LP] robot outside sample index circle.");
  return false;
}

bool LocalPlanner::UpdateSamplingIndex() {
  geometry_msgs::PoseStamped p;
  float check_dist = 0.f, tmp_dist = 0.f;
  std::vector<float> dist_list;
  dist_list.push_back(0.f);
  ROS_DEBUG("[LP] sampling idx %lu, jump %lu, end %lu", sampling_idx_.idx, sampling_idx_.jump, sampling_idx_.end);

  // 更新 idx
  double s = GetShadow(!InsideSamplingIndexCircle(13.0));
  sampling_idx_.cache_dist += s;
  for (size_t i = sampling_idx_.idx, j = i; i < ref_plan_.plan.poses.size(); j = i, i ++) {
    check_dist += Poses2DDistance(ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(j));
    if (check_dist > sampling_idx_.cache_dist) {
      if (i != sampling_idx_.idx)sampling_idx_.cache_dist -= tmp_dist;
      break;
    }
    tmp_dist = check_dist;
    sampling_idx_.idx = i;
  }
  sampling_idx_.init = true;
  ROS_INFO("[LP] sampling idx %lu, jump %lu, end %lu", sampling_idx_.idx, sampling_idx_.jump, sampling_idx_.end);

  // 更新 jump，end
  sampling_idx_.jump = ref_plan_.plan.poses.size()-1;
  sampling_idx_.end = ref_plan_.plan.poses.size()-1;
  check_dist = 0;
  for (size_t i = sampling_idx_.idx, j = i; i < ref_plan_.plan.poses.size();j = i, i++) {
    check_dist += Poses2DDistance(ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(j));
    if (check_dist >= sampling_idx_.jump_dist && sampling_idx_.jump == ref_plan_.plan.poses.size()-1)
      sampling_idx_.jump = i;
    if (check_dist >= cfg_->SAMPLE_MAX_DIS) {
      sampling_idx_.end = i;
      break;
    }
  }

  ref_plan_.plan.poses.at(sampling_idx_.idx).header = ref_plan_.plan.header;
  ref_plan_.plan.poses.at(sampling_idx_.jump).header = ref_plan_.plan.header;
  ref_plan_.plan.poses.at(sampling_idx_.end).header = ref_plan_.plan.header;
  VizMarker(sample_start_pub_, ref_plan_.plan.poses.at(sampling_idx_.idx),
      visualization_msgs::Marker::CUBE, 0.3, 0, 1.0, 0, 1.0);
  VizMarker(sample_jump_pub_, ref_plan_.plan.poses.at(sampling_idx_.jump),
      visualization_msgs::Marker::CUBE, 0.3, 0, 0, 1.0, 1.0);
  VizMarker(sample_end_pub_, ref_plan_.plan.poses.at(sampling_idx_.end),
      visualization_msgs::Marker::CUBE, 0.3, 1.0, 0, 0, 1.0);
  ROS_DEBUG("[LP] sampling idx %lu, jump %lu, end %lu", sampling_idx_.idx, sampling_idx_.jump, sampling_idx_.end);
  return true;
}

void LocalPlanner::SetLocalPlan(const nav_msgs::Path& plan) {
  local_plan_.plan = plan;
  local_plan_.idx = 0;
}

void LocalPlanner::ClearLocalPlan() {
  local_plan_.plan.poses.clear();
  local_plan_.idx = 0;
}

void LocalPlanner::ClearRefPlan() {
  ref_plan_.plan.poses.clear();
  ref_plan_.idx = 0;
}

void LocalPlanner::ClearSamplingIndex() {
  sampling_idx_.idx = 0;
  sampling_idx_.end = 0;
  sampling_idx_.jump = 0;
  sampling_idx_.jump_dist = cfg_->SAMPLE_BEGIN;
  sampling_idx_.cache_dist = 0;
  sampling_idx_.init = false;
}

void LocalPlanner::ResetSamplingJumpDist() {
  sampling_idx_.jump_dist = cfg_->SAMPLE_BEGIN;
  ROS_INFO("[LP] sampling reset, jump dist is %f", sampling_idx_.jump_dist);
}

void LocalPlanner::SetSamplingIndex(const size_t index)
{
  sampling_idx_.idx = index;
}

void LocalPlanner::SamplingJumpForward() {
  sampling_idx_.jump_dist += cfg_->SAMPLE_STEP;
  if (sampling_idx_.jump_dist > cfg_->SAMPLE_MAX_DIS)
    sampling_idx_.jump_dist = cfg_->SAMPLE_BEGIN;
  ROS_INFO("[LP] sampling forward, jump dist is %f", sampling_idx_.jump_dist);
}

std::vector<Eigen::Vector3d> LocalPlanner::RefForwardSampling(const size_t num) {
  std::vector<Eigen::Vector3d> sample;
  refplan_sample_index_.clear();
  float sum = 0.f, standard = cfg_->PARALLEL_B;
  for (size_t i = sampling_idx_.jump; i <= sampling_idx_.end; i ++) {
    if (sum >= standard || sample.empty() || i == sampling_idx_.end) {
      if (num > 2) sum = 0;
      Eigen::Vector3d s(
        ref_plan_.plan.poses.at(i).pose.position.x,
        ref_plan_.plan.poses.at(i).pose.position.y,
        tf::getYaw(ref_plan_.plan.poses.at(i).pose.orientation));
      sample.push_back(s);
      refplan_sample_index_.push_back(i);
      if (sample.size() >= num) break;
    }
    if (i == sampling_idx_.end) break;
    sum += Poses2DDistance(
      ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i+1));
  }
  return sample;
}

void LocalPlanner::Parallel_Sampling(
  const std::vector<Eigen::Vector3d>& sample, std::vector<Eigen::Vector3d>& states) {
  double yaw_offset = M_PI/6;
  std::vector<double> R;
  for (int i = 1; i <= cfg_->PARALLEL_N; i ++) {
    R.push_back(i * cfg_->PARALLEL_R / cfg_->PARALLEL_N);
  }
  for (auto s : sample) {
    for (auto r : R) {
      Eigen::Vector3d sl(s(0) + r * cos(s(2)+M_PI/2),
        s(1) + r * sin(s(2)+M_PI/2), s(2));
      states.push_back(sl);
      sl(2) = sl(2) - yaw_offset;
      states.push_back(sl);
      if(!cfg_->FORBID_RIGHT_SAMPLE) {
        if (r > cfg_->FORBID_RIGHT_RANG) {
          Eigen::Vector3d sr(s(0) + r * cos(s(2)-M_PI/2),
            s(1) + r * sin(s(2)-M_PI/2), s(2));
          states.push_back(sr);
          sr(2) = sr(2) + yaw_offset;
          states.push_back(sr);
        }
      } else {
        sl(2) = sl(2) + 2*yaw_offset;
        states.push_back(sl);
      }
    }
  }
}

size_t LocalPlanner::Reference_Sampling(
  const std::vector<Eigen::Vector3d>& sample,
  std::vector<Eigen::Vector3d>& states) {
  std::vector<Eigen::Vector3d> ref_states;
  double yaw_offset = M_PI/6;
  for (auto s : sample) {
    ref_states.push_back(s);
    if(!cfg_->FORBID_RIGHT_SAMPLE) {
      auto tfl = s; tfl(2) = tfl(2) + yaw_offset; ref_states.push_back(tfl);
    }
    auto tfr = s; tfr(2) = tfr(2) - yaw_offset; ref_states.push_back(tfr);
  }
  states.insert(states.begin(), ref_states.begin(), ref_states.end());
  return ref_states.size();
}

void LocalPlanner::PoseTransform(const geometry_msgs::TransformStamped& transformer,
  const geometry_msgs::PoseStamped& src_pose, geometry_msgs::PoseStamped& tgt_pose)
{
  tf2::doTransform(src_pose, tgt_pose, transformer);
}

void LocalPlanner::PoseTransform(const geometry_msgs::TransformStamped& transformer,
  const Eigen::Vector3d& src_pose, Eigen::Vector3d& tgt_pose)
{
  geometry_msgs::PoseStamped src_p, tgt_p;
  src_p.pose.position.x = src_pose(0);
  src_p.pose.position.y = src_pose(1);
  src_p.pose.orientation = tf::createQuaternionMsgFromYaw(src_pose(2));
  tf2::doTransform(src_p, tgt_p, transformer);
  tgt_pose(0) = tgt_p.pose.position.x;
  tgt_pose(1) = tgt_p.pose.position.y;
  tgt_pose(2) = tf2::getYaw(tgt_p.pose.orientation);
}

void LocalPlanner::PoseTransform(const geometry_msgs::TransformStamped& transformer,
  const Eigen::Vector3d& src_pose, geometry_msgs::PoseStamped& tgt_pose)
{
  geometry_msgs::PoseStamped src_p;
  src_p.pose.position.x = src_pose(0);
  src_p.pose.position.y = src_pose(1);
  src_p.pose.orientation = tf::createQuaternionMsgFromYaw(src_pose(2));
  tf2::doTransform(src_p, tgt_pose, transformer);
}

void LocalPlanner::PoseTransform(const geometry_msgs::TransformStamped& transformer,
  const geometry_msgs::PoseStamped& src_pose, Eigen::Vector3d& tgt_pose)
{
  geometry_msgs::PoseStamped tgt_p;
  tf2::doTransform(src_pose, tgt_p, transformer);
  tgt_pose(0) = tgt_p.pose.position.x;
  tgt_pose(1) = tgt_p.pose.position.y;
  tgt_pose(2) = tf2::getYaw(tgt_p.pose.orientation);
}

bool LocalPlanner::IsRobotCollision(const geometry_msgs::PoseStamped& pose,
  const unsigned int base_cost, const unsigned int head_cost) {
  auto IsCollision = [&](const geometry_msgs::PoseStamped& p, const unsigned int cost) ->bool {
    unsigned int mx, my;
    if (costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
      if (costmap_->getCost(mx, my) >= cost) return true;
      return false;
    } else {
      return true;
    }
  };

  if (IsCollision(pose, base_cost)) {
    return true;
  } else {
    auto head = pose;
    double yaw = tf::getYaw(pose.pose.orientation);
    head.pose.position.x += cfg_->HEAD * cos(yaw);
    head.pose.position.y += cfg_->HEAD * sin(yaw);
    if (IsCollision(head, head_cost)) return true;
    return false;
  }
}

bool LocalPlanner::IsRobotCollision(const Eigen::Vector3d& pose,
  const unsigned int base_cost, const unsigned int head_cost) {
  auto IsCollision = [&](const Eigen::Vector3d& p, const unsigned int cost) ->bool {
    unsigned int mx, my;
    if (costmap_->worldToMap(p(0), p(1), mx, my)) {
      if (costmap_->getCost(mx, my) >= cost) return true;
      return false;
    } else {
      return true;
    }
  };

  if (IsCollision(pose, base_cost)) {
    return true;
  } else {
    auto head = pose;
    head(0) += cfg_->HEAD * cos(head(2));
    head(1) += cfg_->HEAD * sin(head(2));
    if (IsCollision(head, head_cost)) return true;
    return false;
  }
}

bool LocalPlanner::IsNeedToUpdateLocalPlan(const float check_dist)
{
  double dist_sum = 0.f;
  for (size_t i = 0, j = i; i < local_plan_.idx; j = i, i ++) {
    dist_sum += Poses2DDistance(local_plan_.plan.poses.at(i), local_plan_.plan.poses.at(j));
    if (dist_sum > check_dist) return true;
  }
  return false;
}

bool LocalPlanner::IsPlanBlocked(
  const float check_dist, const bool plan_connected,
  size_t& local_plan_end, size_t& ref_plan_end, bool& is_collid_soon) {
  float sum = 0.f;
  unsigned char base_collision_cost = security_detector_->base_collision_cost;
  unsigned char head_collision_cost = security_detector_->head_collision_cost;
  if (!local_plan_.plan.poses.empty()) {
    for (size_t i = local_plan_.idx; i < local_plan_.plan.poses.size()-1; i ++) {
      if (IsRobotCollision(local_plan_.plan.poses.at(i+1), base_collision_cost, head_collision_cost)) {
        ROS_WARN("[LP] local plan collision !");
        // ClearLocalPlan();
        local_plan_end = 0;
        is_collid_soon = true;
        return true;
      }
      sum += Poses2DDistance(
        local_plan_.plan.poses.at(i), local_plan_.plan.poses.at(i+1));
      if (sum >= check_dist) {
        local_plan_end = i+1;
        ROS_INFO("[LP] local plan safe.");
        return false;
      }
      local_plan_end = i+1;
    }
  }

  if (plan_connected || local_plan_.plan.poses.empty()) {
    for (size_t i = ref_plan_.idx; i < ref_plan_.plan.poses.size()-1; i ++) {
      geometry_msgs::PoseStamped tp;
      PoseTransform(plan_to_map_transform_, ref_plan_.plan.poses.at(i+1), tp);
      if (IsRobotCollision(tp, base_collision_cost, head_collision_cost)) {
        ROS_INFO("[LP] ref plan collision !");
        is_collid_soon = true;
        return true;
      }
      sum += Poses2DDistance(
        ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i+1));
      if (sum >= check_dist) {
        ref_plan_end = i+1;
        ROS_INFO("[LP] local plan connect ref plan, plan safe.");
        return false;
      }
      ref_plan_end = i+1;
    }
    ROS_INFO("[LP] plan connect safe.");
    return false;
  }
  // 局部路径快走完了
  if (!local_plan_.plan.poses.empty()) {
    ROS_INFO("[LP] local plan is too short.");
    return true;
  }
  // 在参考路径上且接近终点
  return false;
}

void LocalPlanner::CheckPlanConnected(const Eigen::Vector3d trajectory_end) {
  if (!refplan_sample_index_.empty()) {
    for (auto j : refplan_sample_index_) {
      auto pose = ref_plan_.plan.poses.at(j);
      geometry_msgs::PoseStamped pose_in_robot_frame;
      PoseTransform(plan_to_robot_transform_, pose, pose_in_robot_frame);
      if (Poses2DDistance(pose_in_robot_frame, trajectory_end) < 0.1) {
        ROS_INFO("[LP] check plan connected !");
        ref_plan_.idx = j;
        plan_connected_ = true;
        return;
      }
    }
  }
  plan_connected_ = false;
}

void LocalPlanner::UpdatePubPlanIndex(size_t& local_plan_end, size_t& ref_plan_end,
  const bool plan_connected, double ref_plan_dist)
{
  local_plan_end = local_plan_.plan.poses.size()-1;
  double sum = 0.f;
  if (plan_connected) {
    for (size_t i = ref_plan_.idx, j = i; i < ref_plan_.plan.poses.size();j = i, i ++) {
      sum += Poses2DDistance(ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(j));
      if (sum >= ref_plan_dist) break;
      ref_plan_end = i;
    }
  }
}

void LocalPlanner::MergePlan(const bool plan_connected,
  const size_t local_plan_end, const size_t ref_plan_end, nav_msgs::Path& plan) {
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = ros::Time::now();
  if (!local_plan_.plan.poses.empty() && local_plan_end > local_plan_.idx) {
    plan.poses.insert(plan.poses.begin(),
      local_plan_.plan.poses.begin()+local_plan_.idx,
      local_plan_.plan.poses.begin()+local_plan_end);
  }
  if ((plan_connected || local_plan_.plan.poses.empty()) &&
    ref_plan_end > ref_plan_.idx) {
    geometry_msgs::PoseStamped pose;
    pose.header = plan.header;
    for (size_t i = ref_plan_.idx; i <= ref_plan_end; i ++) {
      PoseTransform(plan_to_map_transform_, ref_plan_.plan.poses.at(i), pose);
      plan.poses.push_back(pose);
    }
  }
}

void LocalPlanner::Trajectory2Plan(
  const std::vector<Eigen::Vector3d>& trajectory, nav_msgs::Path& plan) {
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header = plan.header;
  for (const auto& p : trajectory) {
    PoseTransform(robot_to_map_transform_, p, pose);
    plan.poses.push_back(pose);
  }
}

void LocalPlanner::VizGoal(const Eigen::Vector3d& goal) {
  geometry_msgs::PoseStamped p;
  p.header.frame_id = costmap_ros_->getBaseFrameID();
  p.header.stamp = ros::Time::now();
  p.pose.position.x = goal(0);
  p.pose.position.y = goal(1);
  p.pose.position.z = 0;
  p.pose.orientation = tf::createQuaternionMsgFromYaw(goal(2));
  goal_in_robot_frame_pub_.publish(p);
}

void LocalPlanner::VizGoalSampling(const std::vector<Eigen::Vector3d>& states) {
  geometry_msgs::PoseArray array;
  array.header.stamp = ros::Time::now();
  array.header.frame_id = costmap_ros_->getBaseFrameID();
  geometry_msgs::Pose p;
  for (auto s : states) {
    p.position.x = s[0]; p.position.y = s[1];
    auto q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, s[2]);
    p.orientation = q;
    array.poses.push_back(p);
  }
  goal_sampling_pub_.publish(array);
}

void LocalPlanner::VizCheckPlan(const bool plan_connected,
  const size_t local_plan_end, const size_t ref_plan_end) {
  nav_msgs::Path plan;
  plan.header.frame_id = costmap_ros_->getGlobalFrameID();
  plan.header.stamp = ros::Time::now();
  if (local_plan_end > local_plan_.idx) {
    plan.poses.insert(plan.poses.begin(),
      local_plan_.plan.poses.begin()+local_plan_.idx,
      local_plan_.plan.poses.begin()+local_plan_end+1);
  }
  if (ref_plan_end > ref_plan_.idx) {
    geometry_msgs::PoseStamped p;
    p.header = plan.header;
    for (size_t i = ref_plan_.idx; i <= ref_plan_end; i ++) {
      PoseTransform(plan_to_map_transform_, ref_plan_.plan.poses.at(i), p);
      plan.poses.push_back(p);
    }
  }
  check_plan_pub_.publish(plan);
}

void LocalPlanner::VizCostTrajectories(
  const std::map<double, MotionModelDiffDrive::Trajectory>& trajectories,
  const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  int count = 0;
  const int size = trajectories.size();
  int half = trajectories.size() / 2;
  if (half < 1) half = 1;
  double sr = 0.f, sg = 0.f, sb = 1.f;
  double mr = 0.f, mg = 1.f, mb = 0.f;
  double er = 1.f, eg = 0.f, eb = 0.f;
  double dsm_r = (mr-sr)/half, dsm_g = (mg-sg)/half, dsm_b = (mb-sb)/half;
  double dme_r = (er-mr)/half, dme_g = (eg-mg)/half, dme_b = (eb-mb)/half;
  for (auto pair : trajectories) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = cfg_->ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    if (count < size / 2) {
      v_trajectory.color.r = sr + dsm_r * count; v_trajectory.color.g = sg + dsm_g * count;
      v_trajectory.color.b = sb + dsm_b * count; v_trajectory.color.a = 1.0;
    } else {
      v_trajectory.color.r = mr + dme_r * (count-half); v_trajectory.color.g = mg + dme_g * (count-half);
      v_trajectory.color.b = mb + dme_b * (count-half); v_trajectory.color.a = 1.0;
    }
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.pose.orientation.w = 1.0;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Point p;
    for (const auto& pose : pair.second.trajectory) {
      p.x = pose(0); p.y = pose(1);
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
    count ++;
  }
  pub.publish(v_trajectories);
}

void LocalPlanner::ClearCostTrajectories(const ros::Publisher& pub)
{
  int clear_size = cfg_->PARALLEL_N * cfg_->REF_PLAN_SAMPLE_NUM * 4;
  visualization_msgs::MarkerArray v_trajectories;
  for (int count = 0;count<clear_size;count++) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = cfg_->ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::DELETE;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.color.a = 0;
    v_trajectories.markers.push_back(v_trajectory);
  }
  pub.publish(v_trajectories);
}

double LocalPlanner::GetTargetV(const nav_msgs::Odometry& cur_odom)
{
  if (cfg_->TARGET_V_FREE) {
    double v = cur_odom.twist.twist.linear.x + 0.1; // 0.2
    v = std::min(1.0, std::max(0.3, v));
    return v;
  }
  return cfg_->TARGET_VELOCITY;
}

void LocalPlanner::WaitForOdom()
{
  while (!odom_udpated_) {
    auto odom = cur_odom_;
    double time_diff = ros::Time::now().toSec() - odom.header.stamp.toSec();
    if (time_diff < 0.5) { // 检查 odom 数据实时性
      odom_udpated_ = true;
      break;
    } else {
      ROS_WARN_STREAM_THROTTLE(1.0,"local plan wait for odom ... ");
    }
  }
}

void LocalPlanner::VizMarker(const ros::Publisher& pub,
  const geometry_msgs::PoseStamped& pose, const int type, const float size,
  const float r, const float g, const float b, const float a)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = pose.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "basic_shapes";
  marker.id = 0;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = size;
  marker.scale.y = size;
  marker.scale.z = size;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.color.a = a;
  marker.pose.position.x = pose.pose.position.x;
  marker.pose.position.y = pose.pose.position.y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.lifetime = ros::Duration();
  pub.publish(marker);
}

bool LocalPlanner::Update(bool& is_lost)
{
  UpdateMap();
  if (!UpdateFrame()) return false;
  if (!UpdateRobotPose()) return false;
  if (!UpdateLocalPlanIndex(cfg_->LOCAL_REACH_RANG)) {
    if (UpdateRefPlanIndex(cfg_->REF_REACH_RANG)) {
      ROS_INFO("[LP] run on ref plan");
      SetSamplingIndex(ref_plan_.idx);
      ClearLocalPlan();
    } else {
      is_lost = true;
    }
  }

  UpdateSamplingIndex();
  return true;
}

size_t LocalPlanner::Sampling(Eigen::Vector3d& goal , std::vector<Eigen::Vector3d>& states)
{
  std::vector<Eigen::Vector3d> ref_plan_sample;
  ClearRefplanSample();
  ref_plan_sample = RefForwardSampling(cfg_->REF_PLAN_SAMPLE_NUM);
  ROS_INFO("[LP] sampling size: %lu", ref_plan_sample.size());

  std::vector<Eigen::Vector3d> states_in_ref_plan;
  Parallel_Sampling(ref_plan_sample, states_in_ref_plan);
  size_t ref_sampling_num =  Reference_Sampling(ref_plan_sample, states_in_ref_plan);
  ROS_INFO("[LP] goal sampling size: %lu", states_in_ref_plan.size());

  // 采样点转换到机器坐标系下
  PoseTransform(plan_to_robot_transform_, ref_plan_sample.front(), goal);
  VizGoal(goal);
  states.reserve(states_in_ref_plan.size());
  Eigen::Vector3d state;
  if (!states_in_ref_plan.empty()) {
    for (auto p : states_in_ref_plan) {
      PoseTransform(plan_to_robot_transform_, p, state);
      if (state.segment(0,2).norm() > cfg_->SAMPLE_CLEAR_RADIUS)
        states.emplace_back(state);
    }
    VizGoalSampling(states);
  }

  return ref_sampling_num;
}

bool LocalPlanner::GenerateTrajectoriesToSample(const std::vector<Eigen::Vector3d>& states,
  const size_t ref_sample_num, std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
  double start = ros::Time::now().toSec();

  auto odom = cur_odom_;
  double time_diff = start - odom.header.stamp.toSec();
  if (time_diff > 0.5) { // 检查 odom 数据实时性
    ROS_ERROR("[LP] odom msg is out of date. time_diff:%f, limit is 0.5s.", time_diff);
    return false;
  }

  double target_velocity = GetTargetV(odom);
  double timeout = 1.f / cfg_->HZ / states.size();
  slp_.set_trajectory_config(ref_sample_num,
  1.3*timeout, cfg_->REF_OPT_DIST_ERROR, cfg_->REF_OPT_YAW_ERROR,
  timeout, cfg_->AVOID_OPT_DIST_ERROR, cfg_->AVOID_OPT_YAW_ERROR);
  slp_.generate_trajectories(
    states, odom.twist.twist.linear.x, odom.twist.twist.angular.z, target_velocity, trajectories);
  if (cfg_->ENABLE_SHARP_TRAJECTORY) {
    slp_.generate_trajectories(
      states, odom.twist.twist.linear.x, odom.twist.twist.angular.z + cfg_->MAX_D_YAWRATE / cfg_->HZ,
      target_velocity, trajectories);
    slp_.generate_trajectories(
      states, odom.twist.twist.linear.x, odom.twist.twist.angular.z - cfg_->MAX_D_YAWRATE / cfg_->HZ,
      target_velocity, trajectories);
  }

  ROS_INFO("[LP] state lattice planner generate plans (%lu) speed time %f s.",
    trajectories.size(), ros::Time::now().toSec() - start);
  return true;
}

bool LocalPlanner::GenerateDubinsToSample(const std::vector<Eigen::Vector3d>& states,
  const size_t ref_sample_num, std::vector<MotionModelDiffDrive::Trajectory>& trajectories)
{
  Eigen::Vector3d s(0.0, 0.0, 0.0);
  Dubins dubins;
  double radius = cfg_->DUBINS_RADIUS;
  double step = 0.1;
  for (auto terminal : states) {
    Eigen::Vector3d t(terminal[0], terminal[1], terminal[2]);
    dubins.SetStart(s);
    dubins.SetTerminal(t);
    dubins.SetRadius(radius);
    std::vector<Eigen::Vector3d> poses;
    dubins.GetPath(step, poses);
    MotionModelDiffDrive::Trajectory trajectory;
    trajectory.trajectory = poses;
    trajectories.push_back(trajectory);
  }

  return true;
}

bool LocalPlanner::GetBetterTrajectories(
  const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const Eigen::Vector3d& goal)
{
  if (trajectories.empty()) return false;
  // 按代价从小到大排序
  std::map<double, MotionModelDiffDrive::Trajectory> cost_trajectory_map;
  critic_->sort_trajectories(trajectories, goal, cost_trajectory_map);
  ClearCostTrajectories(cost_trajectories_pub_);
  VizCostTrajectories(cost_trajectory_map, cost_trajectories_pub_);

  auto secure_iter = GetSecurePlan(cost_trajectory_map);
  if (secure_iter != cost_trajectory_map.end()) {
    auto plan = secure_iter->second;
    nav_msgs::Path plan_in_map;
    Trajectory2Plan(plan.trajectory, plan_in_map);
    local_plan_pub_.publish(plan_in_map);
    SetLocalPlan(plan_in_map);
    CheckPlanConnected(plan.trajectory.back());
    LinearInsert2RefPlan(plan_connected_, cfg_->AVOID_OPT_DIST_ERROR*1.3);
    ResetSamplingJumpDist();
    return true;
  }

  ROS_WARN("[LP] all trajectories collision !");
  return false;
}

std::map<double, MotionModelDiffDrive::Trajectory>::const_iterator
LocalPlanner::GetSecurePlan(const std::map<double, MotionModelDiffDrive::Trajectory>& cost_trajectory_map)
{
  std::vector<char> collision_cost_offset_list;
  collision_cost_offset_list.push_back(2*security_detector_->collision_cost_offset);
  collision_cost_offset_list.push_back(security_detector_->collision_cost_offset);
  collision_cost_offset_list.push_back(0);
  unsigned char base_collision_cost = security_detector_->base_security_cost;
  unsigned char head_collision_cost = security_detector_->head_security_cost;
  Eigen::Vector3d pose_in_map;
  int jump = 2;
  for (auto cost : collision_cost_offset_list) { // 碰撞检测值从低到高，尽量选离障碍物远的路径
    for (auto it = cost_trajectory_map.begin(); it != cost_trajectory_map.end(); it ++) {
      bool collisioned = false;
      double dist = 0.0;
      for (size_t i = 0, j = 0; i < it->second.trajectory.size()-jump;j = i, i += jump) {
        dist += std::hypot(it->second.trajectory.at(i)(0)-it->second.trajectory.at(j)(0),
                           it->second.trajectory.at(i)(1)-it->second.trajectory.at(j)(1));
        if (dist >= security_detector_->security_ignore_dist) {
          PoseTransform(robot_to_map_transform_, it->second.trajectory.at(i), pose_in_map);
          if (IsRobotCollision(pose_in_map, base_collision_cost-cost,head_collision_cost-cost)) {
            collisioned = true;
            break;
          }          
        }
      }
      if (!collisioned) return it;
    }
  }
  return cost_trajectory_map.end();
}

size_t LocalPlanner::LinearInsert2RefPlan(const bool plan_connected, const double insert_dist)
{
  if (plan_connected && local_plan_.plan.poses.size() > 0) {
    double s = 0.f;
    size_t ri = local_plan_.plan.poses.size()-1;
    for (; ri > 0; ri --) {
      s += Poses2DDistance(local_plan_.plan.poses.at(ri), local_plan_.plan.poses.at(ri-1));
      if (s >= insert_dist) break;
    }
    ri = ri == 0 ? 1 : ri;

    geometry_msgs::PoseStamped connected_node_in_map;
    geometry_msgs::PoseStamped rp = ref_plan_.plan.poses.at(ref_plan_.idx);
    PoseTransform(plan_to_map_transform_, rp, connected_node_in_map);
    ROS_DEBUG("[LP] ref plan connected node in map [%f, %f, %f]",
      connected_node_in_map.pose.position.x, connected_node_in_map.pose.position.y,
      tf2::getYaw(connected_node_in_map.pose.orientation));

    size_t num = local_plan_.plan.poses.size() - ri;
    double d = Poses2DDistance(local_plan_.plan.poses.at(ri-1), connected_node_in_map);
    double y = angles::shortest_angular_distance(
      tf2::getYaw(local_plan_.plan.poses.at(ri-1).pose.orientation),
      tf2::getYaw(connected_node_in_map.pose.orientation));
    double dd = d / num;
    double dy = y / num;
    geometry_msgs::PoseStamped p = local_plan_.plan.poses.at(ri-1);
    double orient = atan2(connected_node_in_map.pose.position.y - p.pose.position.y,
                          connected_node_in_map.pose.position.x - p.pose.position.x);
    std::vector<geometry_msgs::PoseStamped> poses;
    poses.reserve(num);
    for (size_t n = 0; n < num; n ++) {
      p.pose.position.x += dd * cos(orient);
      p.pose.position.y += dd * sin(orient);
      double yaw = tf2::getYaw(p.pose.orientation) + dy;
      p.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
      ROS_DEBUG("[LP] insert p [%f, %f, %f]", p.pose.position.x, p.pose.position.y, yaw);
      poses.emplace_back(p);
    }
    local_plan_.plan.poses.erase(local_plan_.plan.poses.begin()+ri-1, local_plan_.plan.poses.end());
    local_plan_.plan.poses.insert(local_plan_.plan.poses.end(), poses.begin(), poses.end());
    return num;
  }
  return 0;
}

bool LocalPlanner::IsApproachTerminal()
{
  std::unique_lock<std::mutex> lock(wait_config_mutex_);
  double remain = 0.0f;
  for (size_t i = sampling_idx_.idx, j = i; i < ref_plan_.plan.poses.size();j = i, i ++) {
    remain += Poses2DDistance(ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(j));
    if (remain >= cfg_->TERMINAL_CHECK) return false;
  }
  return true;
}

bool LocalPlanner::Wait(bool is_collid_soon)
{
  if (IsApproachTerminal()) {
    std::unique_lock<std::mutex> lock(wait_config_mutex_);
    dispatcher_.SetWait(ros::Duration(cfg_->TERMINAL_WAIT));
  }
  if (is_collid_soon) {
    ROS_WARN("[LP] check collid soon !");
    dispatcher_.Tick();
  } else {
    // if (local_plan_.plan.poses.empty()) {
    //   dispatcher_.Wakeup();
    // }
  }
  return dispatcher_.Wait();
}

bool LocalPlanner::CheckTerminal(const size_t ref_plan_end)
{
  if (ref_plan_end == ref_plan_.plan.poses.size()-1) {
    if (!final_plan_) {
      ROS_INFO("[LP] pub final plan !");
    }
    final_plan_ = true;
  }
  return final_plan_;
}

bool LocalPlanner::HasPubFinalPlan()
{
  return final_plan_;
}

bool LocalPlanner::PubLocalPlan()
{
  if (ros::Time::now()-pub_timer_ >= ros::Duration(1/cfg_->PUB_HZ)) {
    return true;
  }
  return false;
}


} // namespace local_planner
