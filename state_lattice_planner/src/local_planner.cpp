/**
 * @copyright Copyright (c) {2022} LZY
 * @author LZY (linziyan@yijiahe.com)
 * @date 2023-11-08
 * @brief 
 */

#include <local_planner/local_planner.h>
#include <local_planner/fast_triangular_fun.h>
#include <iostream>

namespace local_planner {

// 查表实现快速三角函数，屏蔽定义使用原函数
#define sin(x) fast_triangular::fast_sin((x))
#define cos(x) fast_triangular::fast_cos((x))
#define M_2PI fast_triangular::M_2PI

LocalPlanner::LocalPlanner(std::shared_ptr<navit_costmap::Costmap2DROS>& costmap_ros):
  local_nh_("~/local_planner/") {
  costmap_ros_ = costmap_ros;
  LoadParams();

  slp_.set_sampling_params(StateLatticePlanner::SamplingParams(N_P, N_H, MAX_ALPHA, MAX_PSI));
  slp_.set_optimization_params(MAX_ITERATION, OPTIMIZATION_TOLERANCE);
  slp_.set_vehicle_params(WHEEL_RADIUS, TREAD);
  slp_.set_motion_params(MAX_ACCELERATION, MAX_YAWRATE, MAX_D_YAWRATE);
  slp_.set_target_velocity(TARGET_VELOCITY);

  candidate_trajectories_pub_ = 
    local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
  candidate_trajectories_no_collision_pub_ =
    local_nh_.advertise<visualization_msgs::MarkerArray>("candidate_trajectories/no_collision", 1);
  selected_trajectory_pub_ =
    local_nh_.advertise<visualization_msgs::Marker>("selected_trajectory", 1);
  goal_sampling_pub_ = local_nh_.advertise<geometry_msgs::PoseArray>("goal_sampling", 1);
  goal_in_robot_frame_pub_ = local_nh_.advertise<geometry_msgs::PoseStamped>("goal_in_robot_frame", 1);
  check_plan_pub_ = local_nh_.advertise<nav_msgs::Path>("check_plan", 1);
  local_plan_pub_ = local_nh_.advertise<nav_msgs::Path>("local_plan", 1);
  odom_sub_ = nh_.subscribe("/odom", 1, &LocalPlanner::OdomCallback, this);

  critic_ = std::make_shared<StateLatticePlanner::Critic>(
    DIST_ERR, YAW_ERR, ANGULAR_ERR, DIST_SCALE, YAW_SCALE, ANGULAR_SCALE);
  slp_.load_lookup_table(LOOKUP_TABLE_FILE_NAME, xyyaw_table_);
}

void LocalPlanner::SetPlan(const nav_msgs::Path& plan) {
  if (plan.poses.empty()) {
    ROS_ERROR("[LP] plan can't be empty !");
    throw "plan can't empty !";
  }
  ref_plan_.plan = plan;
  ref_plan_.idx = 0;
  ClearLocalPlan();
  if (plan.header.frame_id == "") {
    ROS_WARN("[LP] global plan frame id is empty, set frame id [map].");
    ref_plan_.plan.header.frame_id = "map";
  }
}

bool LocalPlanner::IsFinished() {
  if (!ref_plan_.plan.poses.empty()) {
    bool finished = ref_plan_.idx == ref_plan_.plan.poses.size()-1;
    if (finished) {
      ROS_INFO("[LP] finish ref plan.");
      ClearLocalPlan();
      // ClearRefPlan();
    }
    return finished;
  }
  return false;
}

bool LocalPlanner::GetLocalPlan(nav_msgs::Path& plan) {
  if (ref_plan_.plan.poses.empty()) return true;
  
  // 更新基础信息
  static int last_trajectory_num = 0;
  double start = ros::Time::now().toSec();
  bool run_on_ref_plan = false;
  UpdateMap();
  ROS_INFO("[LP] state lattice planner copy map speed time %f s.", ros::Time::now().toSec() - start);
  if (!UpdateFrame()) return false;
  ROS_INFO("[LP] state lattice planner update frame speed time %f s.", ros::Time::now().toSec() - start);
  if (!UpdateRobotPose()) return false;
  ROS_INFO("[LP] state lattice planner update robot in map speed time %f s.", ros::Time::now().toSec() - start);
  if (!UpdateLocalPlanIndex(LOCAL_REACH_RANG)) {
    if (UpdateRefPlanIndex(REF_REACH_RANG)) {
      ROS_INFO("[LP] run on ref plan");
      run_on_ref_plan = true;
      plan_connected_ = true;
    }
  }
  ROS_INFO("[LP] state lattice planner update index speed time %f s.", ros::Time::now().toSec() - start);

  // 目标点采样，基于 ref plan 坐标系下
  bool plan_safe = false;
  size_t local_plan_end = 0, ref_plan_end = 0;
  Eigen::Vector3d goal_on_ref_plan, goal_back_on_ref_plan;
  if (IsNeedForwardSampling(
    CHECK_DISTANCE, plan_connected_, local_plan_end, ref_plan_end)) {
    ROS_INFO("[LP] ref path samping forward.");
    goal_on_ref_plan = RefForwardSampling(SAMPLE_STEP);
    goal_back_on_ref_plan = RefForwardSampling(PARALLEL_B, false);
  } else {
    ROS_INFO("[LP] ref path samping inplace.");
    goal_on_ref_plan = RefForwardSampling(0);
    goal_back_on_ref_plan = RefForwardSampling(PARALLEL_B, false);
    plan_safe = true;
  }
  VizCheckPlan(plan_connected_, local_plan_end, ref_plan_end);
  ROS_INFO("[LP] state lattice planner check path speed time %f s.", ros::Time::now().toSec() - start);

  std::vector<Eigen::Vector3d> states_in_ref_plan;
  if (!plan_safe || !run_on_ref_plan) {
    ROS_INFO("[LP] parallel lookback sampling.");
    Parallel_Sampling(goal_on_ref_plan, states_in_ref_plan);
    Parallel_Sampling(goal_back_on_ref_plan, states_in_ref_plan);
    Lookback_Sampling(LOOKBACK, PARALLEL_B, states_in_ref_plan);
  }
  ROS_INFO("[LP] goal sampling size: %lu", states_in_ref_plan.size());
  ROS_INFO("[LP] state lattice planner sampling speed time %f s.", ros::Time::now().toSec() - start);

  // 机器在参考路径上并且无碰撞，直接按参考路径走
  if (plan_connected_ && plan_safe) {
    ROS_INFO("[LP] local plan end: %lu, ref plan end: %lu", local_plan_end, ref_plan_end);
    MergePlan(plan_connected_, local_plan_end, ref_plan_end, plan); 
    local_plan_pub_.publish(plan);
    ClearLookbackSample();
    ROS_INFO("[LP] state lattice planner follow speed time%f s.", ros::Time::now().toSec() - start);

    // for clear
    std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
    VizTrajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub_);
    VizTrajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub_);
    VizTrajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub_);
    return true; 
  }

  // if (states_in_ref_plan.empty()) return true;
  
  // 采样点转换到机器坐标系下
  Eigen::Vector3d goal;
  PoseTransform(plan_in_robot_frame_, goal_on_ref_plan, goal);
  VizGoal(goal);
  std::vector<Eigen::Vector3d> states;
  states.reserve(states_in_ref_plan.size());
  Eigen::Vector3d state;
  for (auto p : states_in_ref_plan) {
    PoseTransform(plan_in_robot_frame_, p, state);
    states.emplace_back(state);
  }
  VizGoalSampling(states);
  ROS_INFO("[LP] state lattice planner sample to robot frame speed time %f s.", ros::Time::now().toSec() - start);

  // state lattice planner 规划
  auto odom = cur_odom_;
  // odom.twist.twist.linear.x = 0.8;
  double time_diff = start - odom.header.stamp.toSec();
  if (time_diff > 0.5) { // 检查 odom 数据实时性
    ROS_ERROR("[LP] odom msg is out of date. time_diff:%f, limit is 0.5s.", time_diff);
    return false;
  }

  bool turn_flag = false;
  double relative_direction = atan2(goal(1), goal(0));
  if (goal.segment(0, 2).norm() < 1.5) { // 机器接近参考路径的目标点了，不需要做局部规划
    // ClearLocalPlan();
  } else if (fabs(relative_direction) > TURN_DIRECTION_THRESHOLD) { // 目标点在机器后方，不好规划了
    if(fabs(goal(2)) > TURN_DIRECTION_THRESHOLD){
      turn_flag = true;
    }
  }

  slp_.set_target_velocity(1.0);
  double target_velocity = slp_.get_target_velocity(goal_on_ref_plan);
  std::vector<MotionModelDiffDrive::Trajectory> trajectories;
  bool generated = slp_.generate_trajectories(
    states, odom.twist.twist.linear.x, odom.twist.twist.angular.z, target_velocity, trajectories);
  if (ENABLE_SHARP_TRAJECTORY) {
    generated |= slp_.generate_trajectories(
      states, odom.twist.twist.linear.x, odom.twist.twist.angular.z + MAX_D_YAWRATE / HZ,
      target_velocity, trajectories);
    generated |= slp_.generate_trajectories(
      states, odom.twist.twist.linear.x, odom.twist.twist.angular.z - MAX_D_YAWRATE / HZ,
      target_velocity, trajectories);
  }
  ROS_INFO("[LP] state lattice planner generate plan speed time %f s.", ros::Time::now().toSec() - start);

  if (generated) {
    UpdateFrame();
    VizTrajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub_);
    std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
    unsigned char collision_cost = COLLISION_COST;
    Eigen::Vector3d pose_in_map;
    int jump = 3;
    for (const auto& trajectory : trajectories) {
      bool collision = false;
      for (int i = 0; i < trajectory.trajectory.size()-jump; i += jump) {
        // auto p = trajectory.trajectory.at(i);
        // p(2) = atan2(trajectory.trajectory.at(i+jump)(1)-trajectory.trajectory.at(i)(1),
        //              trajectory.trajectory.at(i+jump)(0)-trajectory.trajectory.at(i)(0));
        // PoseTransform(robot_in_map_frame_, p, pose_in_map);
        PoseTransform(robot_in_map_frame_, trajectory.trajectory.at(i), pose_in_map);
        if (IsRobotCollision(pose_in_map, collision_cost)) {
          collision = true;
          break;
        }
      }
      if (!collision) candidate_trajectories.push_back(trajectory);
    }
    ROS_INFO("[LP] state lattice planner check collision speed time %f s.", ros::Time::now().toSec() - start);
    ROS_INFO("[LP] trajectories size:%lu, candidate_trajectories:%lu",
      trajectories.size(), candidate_trajectories.size());

    if(candidate_trajectories.size() > 0) {
      // 在无碰撞轨迹中挑选出最佳轨迹
      VizTrajectories(
        candidate_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub_);
      MotionModelDiffDrive::Trajectory trajectory;
      critic_->pickup_trajectory(candidate_trajectories, goal, trajectory);
      VizTrajectory(trajectory, 1, 0, 0, selected_trajectory_pub_);
      Trajectory2Plan(trajectory.trajectory, plan);
      local_plan_pub_.publish(plan);
      SetLocalPlan(plan);
      CheckPlanConnected(trajectory.trajectory.back());
      ROS_INFO("[LP] state lattice planner avoid speed time %f s.", ros::Time::now().toSec() - start);
    } else { // 原有轨迹有碰撞都不能用了，机器暂停
      ROS_WARN("[LP] local trajectories all collision !");
      // for clear
      std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
      VizTrajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub_);
      VizTrajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub_);
      VizTrajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub_);
      return false;  
    }
  } else { // 没生成路径
    ROS_WARN("[LP] can not find local plan !");
    // for clear
    std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
    VizTrajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub_);
    VizTrajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub_);
    VizTrajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub_);
    return false;   
  }

  return true;
}


// -------------------- private --------------------
void LocalPlanner::OdomCallback(const nav_msgs::OdometryConstPtr& msg) {
  cur_odom_ = *msg;
}

void LocalPlanner::LoadParams() {
  local_nh_.param("HZ", HZ, {20});
  local_nh_.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
  local_nh_.param("N_P", N_P, {10});
  local_nh_.param("N_H", N_H, {3});
  local_nh_.param("MAX_ALPHA", MAX_ALPHA, {M_PI / 3.0});
  local_nh_.param("MAX_PSI", MAX_PSI, {M_PI / 6.0});
  local_nh_.param("N_S", N_S, {1000});
  local_nh_.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
  local_nh_.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
  local_nh_.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
  local_nh_.param("MAX_ITERATION", MAX_ITERATION, {100});
  local_nh_.param("OPTIMIZATION_TOLERANCE", OPTIMIZATION_TOLERANCE, {0.1});
  local_nh_.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
  local_nh_.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
  local_nh_.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {11.6});
  local_nh_.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
  local_nh_.param("TREAD", TREAD, {0.5});
  local_nh_.param("IGNORABLE_OBSTACLE_RANGE", IGNORABLE_OBSTACLE_RANGE, {1.0});
  local_nh_.param("VERBOSE", VERBOSE, {false});
  local_nh_.param("CONTROL_DELAY", CONTROL_DELAY, {1});
  local_nh_.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {M_PI/4.0});
  local_nh_.param("ENABLE_SHARP_TRAJECTORY", ENABLE_SHARP_TRAJECTORY, {false});
  local_nh_.param("ENABLE_CONTROL_SPACE_SAMPLING", ENABLE_CONTROL_SPACE_SAMPLING, {false});
  local_nh_.param("DRIVE", DRIVE, {true});
  local_nh_.param("ACKERMANN", ACKERMANN, {false});
  local_nh_.param("WHEEL_BASE", WHEEL_BASE, {0.95});
  local_nh_.param("NEW_GOAL_SAMPLING", NEW_GOAL_SAMPLING, {false});
  local_nh_.param("NGS_R", NGS_R, {1.5});
  local_nh_.param("NGS_NA", NGS_NA, {8});
  local_nh_.param("NGS_NR", NGS_NR, {3});
  local_nh_.param("NGS_GY", NGS_GY, {0.785398});
  local_nh_.param("NGS_NY", NGS_NY, {3});
  local_nh_.param("SAMPLE_STEP", SAMPLE_STEP, {2.0});
  local_nh_.param("PARALLEL_R", PARALLEL_R, {5.0});
  local_nh_.param("PARALLEL_N", PARALLEL_N, {2});
  local_nh_.param("PARALLEL_B", PARALLEL_B, {3.0});
  local_nh_.param("SMAPLE_MAX_DIS", SMAPLE_MAX_DIS, {5.0});
  local_nh_.param("LOOKBACK", LOOKBACK, {1.5});
  local_nh_.param("HEAD", HEAD, {-1});
  local_nh_.param("COLLISION_COST", COLLISION_COST, {0});
  local_nh_.param("DIST_ERR", DIST_ERR, {6.0});
  local_nh_.param("YAW_ERR", YAW_ERR, {0.785398});
  local_nh_.param("ANGULAR_ERR", ANGULAR_ERR, {2.707093});
  local_nh_.param("DIST_SCALE", DIST_SCALE, {60.0});
  local_nh_.param("YAW_SCALE", YAW_SCALE, {20.0});
  local_nh_.param("ANGULAR_SCALE", ANGULAR_SCALE, {30.0});
  local_nh_.param("CHECK_DISTANCE", CHECK_DISTANCE, {5.0});
  local_nh_.param("LOCAL_REACH_RANG", LOCAL_REACH_RANG, {0.8});
  local_nh_.param("REF_REACH_RANG", REF_REACH_RANG, {0.5});
}

bool LocalPlanner::UpdateFrame() {
  std::string map_frame = costmap_ros_->getGlobalFrameID();
  std::string robot_frame = costmap_ros_->getBaseFrameID();
  std::string plan_frame = ref_plan_.plan.header.frame_id;
  // ROS_INFO("[LP] map frame: %s, robot frame: %s, plan frame: %s",
  //   map_frame.c_str(), robot_frame.c_str(), plan_frame.c_str());

  geometry_msgs::PoseStamped tgt_link;
  geometry_msgs::PoseStamped src_link;
  src_link.header.frame_id = plan_frame;
  src_link.header.stamp = ros::Time::now();
  src_link.pose.position.x = 0;
  src_link.pose.position.y = 0;
  src_link.pose.position.z = 0;
  src_link.pose.orientation.w = 1;

  // ref_plan -> map frame
  try{
    src_link.header.frame_id = plan_frame;
    listener_.transformPose(map_frame, src_link, tgt_link);
    // ROS_INFO("[LP] ref plan O in map [%f, %f, %f]",
    //   tgt_link.pose.position.x, tgt_link.pose.position.y, tf::getYaw(tgt_link.pose.orientation));
  }catch(tf::TransformException ex){
    ROS_ERROR("[LP] tf listen ref plan -> map failed, %s", ex.what());
    return false;
  }

  plan_in_map_frame_(0) = tgt_link.pose.position.x;
  plan_in_map_frame_(1) = tgt_link.pose.position.y;
  plan_in_map_frame_(2) = tf::getYaw(tgt_link.pose.orientation);

  // ref_plan -> robot frame
  try{
    src_link.header.frame_id = plan_frame;
    listener_.transformPose(robot_frame, src_link, tgt_link);
    // ROS_INFO("[LP] ref plan O in robot [%f, %f, %f]",
    //   tgt_link.pose.position.x, tgt_link.pose.position.y, tf::getYaw(tgt_link.pose.orientation));
  }catch(tf::TransformException ex){
    ROS_ERROR("[LP] tf listen ref plan -> robot failed, %s", ex.what());
    return false;
  }

  plan_in_robot_frame_(0) = tgt_link.pose.position.x;
  plan_in_robot_frame_(1) = tgt_link.pose.position.y;
  plan_in_robot_frame_(2) = tf::getYaw(tgt_link.pose.orientation);

  // robot -> map frame
  try{
    src_link.header.frame_id = robot_frame;
    listener_.transformPose(map_frame, src_link, tgt_link);
    // ROS_INFO("[LP] robot O in map [%f, %f, %f]",
    //   tgt_link.pose.position.x, tgt_link.pose.position.y, tf::getYaw(tgt_link.pose.orientation));
  }catch(tf::TransformException ex){
    ROS_ERROR("[LP] tf listen robot -> map failed, %s", ex.what());
    return false;
  }

  robot_in_map_frame_(0) = tgt_link.pose.position.x;
  robot_in_map_frame_(1) = tgt_link.pose.position.y;
  robot_in_map_frame_(2) = tf::getYaw(tgt_link.pose.orientation);

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

bool LocalPlanner::UpdateLocalPlanIndex(const double rang) {
  bool find = false;
  for (size_t i = local_plan_.idx; i < local_plan_.plan.poses.size(); i ++) {
    float d = Poses2DDistance(robot_pose_, local_plan_.plan.poses.at(i));
    if (d < rang) {
      find = true;
      local_plan_.idx = i;
    }
    else if (find) return true;
  }

  return false;
}

bool LocalPlanner::UpdateRefPlanIndex(const double rang) {
  bool find = false;
  geometry_msgs::PoseStamped p;
  int num = 200;
  for (size_t i = ref_plan_.idx; i < ref_plan_.plan.poses.size(); i ++) {
    PoseTransform(plan_in_map_frame_, ref_plan_.plan.poses.at(i), p);
    float d = Poses2DDistance(robot_pose_, p);
    if (d < rang) {
      find = true;
      ref_plan_.idx = i;
    } else if (find) {
      break;
    }
    if (num-- < 0) break;
  }

  if (find) return true;
  return false;
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

Eigen::Vector3d LocalPlanner::RefForwardSampling(const float step, const bool update_idx) {
  size_t idx = ref_plan_.idx;
  if (step > 1e-3) {
    float sum = 0.f;
    Eigen::Vector3d p;
    for (size_t i = ref_plan_.idx; i < ref_plan_.plan.poses.size()-1; i ++) {
      PoseTransform(plan_in_map_frame_, ref_plan_.plan.poses.at(i), p);
      float rang = Poses2DDistance(robot_pose_, p);
      if (rang >= SMAPLE_MAX_DIS && update_idx) {
        idx = i;
        sum = step+1;
        break;
      }
      sum += Poses2DDistance(
        ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i+1));
      if (sum >= step) {
        idx = i+1;
        break;
      }
    }
    if (sum < step) idx = ref_plan_.plan.poses.size()-1;
  }

  if(update_idx) ref_plan_.idx = idx;
  Eigen::Vector3d goal(
    ref_plan_.plan.poses.at(idx).pose.position.x,
    ref_plan_.plan.poses.at(idx).pose.position.y,
    tf::getYaw(ref_plan_.plan.poses.at(idx).pose.orientation));
  return goal;
}

void LocalPlanner::Parallel_Sampling(
  const Eigen::Vector3d goal, std::vector<Eigen::Vector3d>& states) {
  double yaw_offset = M_PI/6;
  double radius = PARALLEL_R;
  std::vector<double> R;
  for (int i = 1; i <= PARALLEL_N; i ++) {
    R.push_back(i * PARALLEL_R / PARALLEL_N);  
  }
  for (auto r : R) {
    Eigen::Vector3d sl(goal(0) + r * cos(goal(2)+M_PI/2),
      goal(1) + r * sin(goal(2)+M_PI/2), goal(2));
    states.push_back(sl);
    sl(2) = sl(2) - yaw_offset;
    states.push_back(sl);
    Eigen::Vector3d sr(goal(0) + r * cos(goal(2)-M_PI/2),
      goal(1) + r * sin(goal(2)-M_PI/2), goal(2));
    states.push_back(sr);
    sr(2) = sr(2) + yaw_offset;
    states.push_back(sr);
  }
}

void LocalPlanner::Lookback_Sampling(
  const double back_step, const double forward_step,
  std::vector<Eigen::Vector3d>& states) {
  Eigen::Vector3d goal(
    ref_plan_.plan.poses.at(ref_plan_.idx).pose.position.x,
    ref_plan_.plan.poses.at(ref_plan_.idx).pose.position.y,
    tf::getYaw(ref_plan_.plan.poses.at(ref_plan_.idx).pose.orientation));
  states.push_back(goal);
  lookback_sample_index_.push_back(ref_plan_.idx);
  double yaw_offset = M_PI/6;
  auto tgl = goal; tgl(2) = tgl(2) + yaw_offset; states.push_back(tgl);
  auto tgr = goal; tgr(2) = tgr(2) - yaw_offset; states.push_back(tgr);
  float sum = 0.f;  
  if (back_step > 0) {
    for (int i = ref_plan_.idx; i > 0; i --) {
      sum += Poses2DDistance(
        ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i-1));
      if (sum >= back_step) {
        Eigen::Vector3d back(
          ref_plan_.plan.poses.at(i-1).pose.position.x,
          ref_plan_.plan.poses.at(i-1).pose.position.y,
          tf::getYaw(ref_plan_.plan.poses.at(i-1).pose.orientation));
        states.push_back(back);
        lookback_sample_index_.push_back(i-1);
        break;
      }
    }    
  }
  sum = 0;
  int forward_idx = -1;
  for (int i = ref_plan_.idx; i < ref_plan_.plan.poses.size()-1; i ++) {
    sum += Poses2DDistance(
      ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i+1));
    forward_idx = i+1;      
    if (sum >= forward_step) break;
  }
  if (forward_idx > 0) {
    Eigen::Vector3d forward(
      ref_plan_.plan.poses.at(forward_idx).pose.position.x,
      ref_plan_.plan.poses.at(forward_idx).pose.position.y,
      tf::getYaw(ref_plan_.plan.poses.at(forward_idx).pose.orientation));
    states.push_back(forward);
    lookback_sample_index_.push_back(forward_idx);
    auto tfl = forward; tfl(2) = tfl(2) + yaw_offset; states.push_back(tfl);
    auto tfr = forward; tfr(2) = tfr(2) - yaw_offset; states.push_back(tfr);
  }
}

void LocalPlanner::PoseTransform(const Eigen::Vector3d& transformer,
  const geometry_msgs::PoseStamped& src_pose, geometry_msgs::PoseStamped& tgt_pose) {
  double x_offset = cos(transformer(2)) * src_pose.pose.position.x -
                    sin(transformer(2)) * src_pose.pose.position.y;
  tgt_pose.pose.position.x = transformer(0) + x_offset;
  double y_offset = sin(transformer(2)) * src_pose.pose.position.x +
                    cos(transformer(2)) * src_pose.pose.position.y;
  tgt_pose.pose.position.y = transformer(1) + y_offset;
  tgt_pose.pose.position.z = 0;
  double yaw = transformer(2) + tf::getYaw(src_pose.pose.orientation);
  if (yaw > M_PI) yaw = yaw - M_2PI;
  else if (yaw < -M_PI) yaw = yaw + M_2PI;
  tgt_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

void LocalPlanner::PoseTransform(const Eigen::Vector3d& transformer,
  const Eigen::Vector3d& src_pose, Eigen::Vector3d& tgt_pose) {
  double x_offset = cos(transformer(2)) * src_pose(0) -
                    sin(transformer(2)) * src_pose(1);
  tgt_pose(0) = transformer(0) + x_offset;
  double y_offset = sin(transformer(2)) * src_pose(0) +
                    cos(transformer(2)) * src_pose(1);
  tgt_pose(1) = transformer(1) + y_offset;
  double yaw = transformer(2) + src_pose(2);
  if (yaw > M_PI) yaw = yaw - M_2PI;
  else if (yaw < -M_PI) yaw = yaw + M_2PI;
  tgt_pose(2) = yaw;
}

void LocalPlanner::PoseTransform(const Eigen::Vector3d& transformer,
  const Eigen::Vector3d& src_pose, geometry_msgs::PoseStamped& tgt_pose) {
  double x_offset = cos(transformer(2)) * src_pose(0) -
                    sin(transformer(2)) * src_pose(1);
  tgt_pose.pose.position.x = transformer(0) + x_offset;
  double y_offset = sin(transformer(2)) * src_pose(0) +
                    cos(transformer(2)) * src_pose(1);
  tgt_pose.pose.position.y = transformer(1) + y_offset;
  tgt_pose.pose.position.z = 0;
  double yaw = transformer(2) + src_pose(2);
  if (yaw > M_PI) yaw = yaw - M_2PI;
  else if (yaw < -M_PI) yaw = yaw + M_2PI;
  tgt_pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
}

void LocalPlanner::PoseTransform(const Eigen::Vector3d& transformer,
  const geometry_msgs::PoseStamped& src_pose, Eigen::Vector3d& tgt_pose) {
  double x_offset = cos(transformer(2)) * src_pose.pose.position.x -
                    sin(transformer(2)) * src_pose.pose.position.y;
  tgt_pose(0) = transformer(0) + x_offset;
  double y_offset = sin(transformer(2)) * src_pose.pose.position.x +
                    cos(transformer(2)) * src_pose.pose.position.y;
  tgt_pose(1) = transformer(1) + y_offset;
  double yaw = transformer(2) + tf::getYaw(src_pose.pose.orientation);
  if (yaw > M_PI) yaw = yaw - M_2PI;
  else if (yaw < -M_PI) yaw = yaw + M_2PI;
  tgt_pose(2) = yaw;
}

bool LocalPlanner::IsRobotCollision(const geometry_msgs::PoseStamped& pose,
  const unsigned int collision_cost) {
  auto IsCollision = [&](const geometry_msgs::PoseStamped& p) ->bool {
    unsigned int mx, my;
    if (costmap_->worldToMap(p.pose.position.x, p.pose.position.y, mx, my)) {
      if (costmap_->getCost(mx, my) >= collision_cost) return true;
      return false;
    } else {
      return true;
    }
  };

  if (IsCollision(pose)) {
    return true;
  } else {
    auto head = pose;
    double yaw = tf::getYaw(pose.pose.orientation);
    head.pose.position.x += HEAD * cos(yaw);
    head.pose.position.y += HEAD * sin(yaw);
    if (IsCollision(head)) return true;
    return false;
  }
}

bool LocalPlanner::IsRobotCollision(const Eigen::Vector3d& pose,
                                    const unsigned int collision_cost) {
  auto IsCollision = [&](const Eigen::Vector3d& p) ->bool {
    unsigned int mx, my;
    if (costmap_->worldToMap(p(0), p(1), mx, my)) {
      if (costmap_->getCost(mx, my) >= collision_cost) return true;
      return false;
    } else {
      return true;
    }
  };

  if (IsCollision(pose)) {
    return true;
  } else {
    auto head = pose;
    head(0) += HEAD * cos(head(2));
    head(1) += HEAD * sin(head(2));
    if (IsCollision(head)) return true;
    return false;
  }
}

bool LocalPlanner::IsNeedForwardSampling(
  const float check_dist, const bool plan_connected,
  size_t& local_plan_end, size_t& ref_plan_end) {
  float sum = 0.f;
  unsigned char collision_cost = COLLISION_COST;
  if (!local_plan_.plan.poses.empty()) {
    for (int i = local_plan_.idx; i < local_plan_.plan.poses.size()-1; i ++) {
      if (IsRobotCollision(local_plan_.plan.poses.at(i+1), collision_cost)) {
        ROS_WARN("[LP] local plan collision !");
        ClearLocalPlan();
        local_plan_end = 0;
        return true;
      }
      sum += Poses2DDistance(
        local_plan_.plan.poses.at(i), local_plan_.plan.poses.at(i+1));
      if (sum >= check_dist) {
        local_plan_end = i+1;
        return false;
      }
      local_plan_end = i+1;
    }    
  }

  if (plan_connected || local_plan_.plan.poses.empty()) {
    for (int i = ref_plan_.idx; i < ref_plan_.plan.poses.size()-1; i ++) {
      geometry_msgs::PoseStamped tp;
      PoseTransform(plan_in_map_frame_, ref_plan_.plan.poses.at(i+1), tp);
      if (IsRobotCollision(tp, collision_cost)) {
        ROS_INFO("[LP] ref plan collision !");
        return true;
      }
      sum += Poses2DDistance(
        ref_plan_.plan.poses.at(i), ref_plan_.plan.poses.at(i+1));
      if (sum >= check_dist) {
        ref_plan_end = i+1;
        return false;
      }
      ref_plan_end = i+1;
    }
    return false;
  }

  if (!local_plan_.plan.poses.empty()) {
    ROS_INFO("[LP] local plan is too short.");
    return true;    
  }

  return false;    
}

void LocalPlanner::CheckPlanConnected(const Eigen::Vector3d trajectory_end) {
  if (!lookback_sample_index_.empty()) {
    for (auto j : lookback_sample_index_) {
      auto pose = ref_plan_.plan.poses.at(j);
      geometry_msgs::PoseStamped pose_in_robot_frame;
      PoseTransform(plan_in_robot_frame_, pose, pose_in_robot_frame);
      if (Poses2DDistance(pose_in_robot_frame, trajectory_end) < 0.1) {    
        ref_plan_.idx = j;
        plan_connected_ = true;        
        return;
      }
    }    
  }
  plan_connected_ = false;
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
    for (int i = ref_plan_.idx; i <= ref_plan_end; i ++) {
      PoseTransform(plan_in_map_frame_, ref_plan_.plan.poses.at(i), pose);
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
    PoseTransform(robot_in_map_frame_, p, pose);
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
      PoseTransform(plan_in_map_frame_, ref_plan_.plan.poses.at(i), p);
      plan.poses.push_back(p);
    }
  } 
  check_plan_pub_.publish(plan);
}

void LocalPlanner::VizTrajectories(
  const std::vector<MotionModelDiffDrive::Trajectory>& trajectories,
  const double r, const double g, const double b,
  const int trajectories_size, const ros::Publisher& pub)
{
  visualization_msgs::MarkerArray v_trajectories;
  int count = 0;
  const int size = trajectories.size();
  for (;count<size;count++) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r; v_trajectory.color.g = g;
    v_trajectory.color.b = b; v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectory.pose.orientation.w = 1.0;
    v_trajectory.scale.x = 0.02;
    geometry_msgs::Point p;
    for (const auto& pose : trajectories[count].trajectory) {
      p.x = pose(0); p.y = pose(1);
      v_trajectory.points.push_back(p);
    }
    v_trajectories.markers.push_back(v_trajectory);
  }
  for (;count<trajectories_size;) {
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::DELETE;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.id = count;
    v_trajectories.markers.push_back(v_trajectory);
    count++;
  }
  pub.publish(v_trajectories);
}

void LocalPlanner::VizTrajectory(
  const MotionModelDiffDrive::Trajectory& trajectory,
  const double r, const double g, const double b, const ros::Publisher& pub)
{
  visualization_msgs::Marker v_trajectory;
  v_trajectory.header.frame_id = ROBOT_FRAME;
  v_trajectory.header.stamp = ros::Time::now();
  v_trajectory.color.r = r; v_trajectory.color.g = g;
  v_trajectory.color.b = b; v_trajectory.color.a = 0.8;
  v_trajectory.ns = pub.getTopic();
  v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
  v_trajectory.action = visualization_msgs::Marker::ADD;
  v_trajectory.lifetime = ros::Duration();
  v_trajectory.pose.orientation.w = 1.0;
  v_trajectory.pose.position.z = 0.1;
  v_trajectory.scale.x = 0.10;
  geometry_msgs::Point p;
  for (const auto& pose : trajectory.trajectory) {
    p.x = pose(0);
    p.y = pose(1);
    v_trajectory.points.push_back(p);
  }
  pub.publish(v_trajectory);
}


} // namespace local_planner
