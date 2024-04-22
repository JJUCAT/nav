#include "state_lattice_planner/state_lattice_planner_ros.h"

StateLatticePlannerROS::StateLatticePlannerROS(void)
:local_nh("~")
{
    local_nh.param("HZ", HZ, {20});
    local_nh.param("ROBOT_FRAME", ROBOT_FRAME, {"base_link"});
    local_nh.param("N_P", N_P, {10});
    local_nh.param("N_H", N_H, {3});
    local_nh.param("MAX_ALPHA", MAX_ALPHA, {M_PI / 3.0});
    local_nh.param("MAX_PSI", MAX_PSI, {M_PI / 6.0});
    local_nh.param("N_S", N_S, {1000});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});
    local_nh.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    local_nh.param("MAX_ITERATION", MAX_ITERATION, {100});
    local_nh.param("OPTIMIZATION_TOLERANCE", OPTIMIZATION_TOLERANCE, {0.1});
    local_nh.param("MAX_YAWRATE", MAX_YAWRATE, {0.8});
    local_nh.param("MAX_D_YAWRATE", MAX_D_YAWRATE, {2.0});
    local_nh.param("MAX_WHEEL_ANGULAR_VELOCITY", MAX_WHEEL_ANGULAR_VELOCITY, {11.6});
    local_nh.param("WHEEL_RADIUS", WHEEL_RADIUS, {0.125});
    local_nh.param("TREAD", TREAD, {0.5});
    local_nh.param("IGNORABLE_OBSTACLE_RANGE", IGNORABLE_OBSTACLE_RANGE, {1.0});
    local_nh.param("VERBOSE", VERBOSE, {false});
    local_nh.param("CONTROL_DELAY", CONTROL_DELAY, {1});
    local_nh.param("TURN_DIRECTION_THRESHOLD", TURN_DIRECTION_THRESHOLD, {M_PI/4.0});
    local_nh.param("ENABLE_SHARP_TRAJECTORY", ENABLE_SHARP_TRAJECTORY, {false});
    local_nh.param("ENABLE_CONTROL_SPACE_SAMPLING", ENABLE_CONTROL_SPACE_SAMPLING, {false});
    local_nh.param("DRIVE", DRIVE, {true});
    local_nh.param("ACKERMANN", ACKERMANN, {false});
    local_nh.param("WHEEL_BASE", WHEEL_BASE, {0.95});
    local_nh.param("NEW_GOAL_SAMPLING", NEW_GOAL_SAMPLING, {false});
    local_nh.param("NGS_R", NGS_R, {1.5});
    local_nh.param("NGS_NA", NGS_NA, {8});
    local_nh.param("NGS_NR", NGS_NR, {3});
    local_nh.param("NGS_GY", NGS_GY, {0.785398});
    local_nh.param("NGS_NY", NGS_NY, {3});
    local_nh.param("HEAD", HEAD, {-1});
    local_nh.param("COLLISION_COST", COLLISION_COST, {0});
    local_nh.param("DIST_ERR", DIST_ERR, {6.0});
    local_nh.param("YAW_ERR", YAW_ERR, {0.785398});
    local_nh.param("ANGULAR_ERR", ANGULAR_ERR, {2.707093});
    local_nh.param("DIST_SCALE", DIST_SCALE, {60.0});
    local_nh.param("YAW_SCALE", YAW_SCALE, {20.0});
    local_nh.param("ANGULAR_SCALE", ANGULAR_SCALE, {30.0});

    std::cout << "HZ: " << HZ << std::endl;
    std::cout << "ROBOT_FRAME: " << ROBOT_FRAME << std::endl;
    std::cout << "N_P: " << N_P << std::endl;
    std::cout << "N_H: " << N_H << std::endl;
    std::cout << "MAX_ALPHA: " << MAX_ALPHA << std::endl;
    std::cout << "MAX_PSI: " << MAX_PSI << std::endl;
    std::cout << "N_S: " << N_S << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
    std::cout << "LOOKUP_TABLE_FILE_NAME: " << LOOKUP_TABLE_FILE_NAME << std::endl;
    std::cout << "MAX_ITERATION: " << MAX_ITERATION << std::endl;
    std::cout << "OPTIMIZATION_TOLERANCE: " << OPTIMIZATION_TOLERANCE << std::endl;
    std::cout << "MAX_YAWRATE: " << MAX_YAWRATE << std::endl;
    std::cout << "MAX_D_YAWRATE: " << MAX_D_YAWRATE << std::endl;
    std::cout << "MAX_WHEEL_ANGULAR_VELOCITY: " << MAX_WHEEL_ANGULAR_VELOCITY << std::endl;
    std::cout << "WHEEL_RADIUS: " << WHEEL_RADIUS << std::endl;
    std::cout << "TREAD: " << TREAD << std::endl;
    std::cout << "IGNORABLE_OBSTACLE_RANGE: " << IGNORABLE_OBSTACLE_RANGE << std::endl;
    std::cout << "VERBOSE: " << VERBOSE << std::endl;
    std::cout << "CONTROL_DELAY: " << CONTROL_DELAY << std::endl;
    std::cout << "TURN_DIRECTION_THRESHOLD: " << TURN_DIRECTION_THRESHOLD << std::endl;
    std::cout << "ENABLE_SHARP_TRAJECTORY: " << ENABLE_SHARP_TRAJECTORY << std::endl;
    std::cout << "ENABLE_CONTROL_SPACE_SAMPLING: " << ENABLE_CONTROL_SPACE_SAMPLING << std::endl;

    planner.set_sampling_params(StateLatticePlanner::SamplingParams(N_P, N_H, MAX_ALPHA, MAX_PSI));
    planner.set_optimization_params(MAX_ITERATION, OPTIMIZATION_TOLERANCE);
    planner.set_vehicle_params(WHEEL_RADIUS, TREAD);
    planner.set_motion_params(MAX_ACCELERATION, MAX_YAWRATE, MAX_D_YAWRATE);
    planner.set_target_velocity(TARGET_VELOCITY);

    best_trajectory_pub = local_nh.advertise<nav_msgs::Path>("best_trajectory", 1);
    xyyaw_table_pub = local_nh.advertise<sensor_msgs::PointCloud>("xyyaw_table", 1);
    goal_pub = local_nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
    goal_sampling_pub = local_nh.advertise<geometry_msgs::PoseArray>("goal_sampling", 1);
    velocity_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    candidate_trajectories_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories", 1);
    candidate_trajectories_no_collision_pub = local_nh.advertise<visualization_msgs::MarkerArray>("candidate_trajectories/no_collision", 1);
    selected_trajectory_pub = local_nh.advertise<visualization_msgs::Marker>("selected_trajectory", 1);

    local_goal_sub = nh.subscribe("/sim_goal", 1, &StateLatticePlannerROS::local_goal_callback, this);
    local_map_sub = nh.subscribe("/navit_controller_node/controller_costmap/costmap", 1, &StateLatticePlannerROS::local_map_callback, this);
    odom_sub = nh.subscribe("/odom", 1, &StateLatticePlannerROS::odom_callback, this);
    target_velocity_sub = nh.subscribe("/target_velocity", 1, &StateLatticePlannerROS::target_velocity_callback, this);

    local_goal_subscribed = false;
    local_map_updated = false;
    odom_updated = false;

    critic_ = std::make_shared<StateLatticePlanner::Critic>(
      DIST_ERR, YAW_ERR, ANGULAR_ERR, DIST_SCALE, YAW_SCALE, ANGULAR_SCALE);
    planner.load_lookup_table(LOOKUP_TABLE_FILE_NAME, xyyaw_table_);
    init_xyyaw_table();
}

void StateLatticePlannerROS::local_goal_callback(const geometry_msgs::PoseStampedConstPtr& msg)
{
    local_goal = *msg;
    try{
        // 转换到 odom 坐标系下
        listener.transformPose("/odom", ros::Time(0), local_goal, local_goal.header.frame_id, local_goal);
        local_goal_subscribed = true;
        goal_pub.publish(*msg);
    }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
    }
}

void StateLatticePlannerROS::local_map_callback(const nav_msgs::OccupancyGridConstPtr& msg)
{
    local_map = *msg;
    local_map_updated = true;
}

void StateLatticePlannerROS::odom_callback(const nav_msgs::OdometryConstPtr& msg)
{
    current_velocity = msg->twist.twist;
    odom_updated = true;
}

void StateLatticePlannerROS::target_velocity_callback(const geometry_msgs::TwistConstPtr& msg)
{
    if(msg->linear.x > 0.0){
        TARGET_VELOCITY = msg->linear.x;
        std::cout << "\033[31mtarget velocity was updated to " << TARGET_VELOCITY << "[m/s]\033[0m" << std::endl;
    }
}

void StateLatticePlannerROS::process(void)
{
  ros::Rate loop_rate(HZ);
  xyyaw_table_pub.publish(pc_table_);
  while(ros::ok()){
    bool goal_transformed = false;
    geometry_msgs::PoseStamped local_goal_base_link;
    if(local_goal_subscribed){
      try{
        listener.transformPose(ROBOT_FRAME, ros::Time(0), local_goal, local_goal.header.frame_id, local_goal_base_link);
        goal_transformed = true;
      }catch(tf2::TransformException& ex){
        std::cout << ex.what() << std::endl;
      }
    }
    if(local_goal_subscribed && local_map_updated && odom_updated && goal_transformed) {
      std::cout << "=== state lattice planner ===" << std::endl;
      xyyaw_table_pub.publish(pc_table_);
      double start = ros::Time::now().toSec();
      static int last_trajectory_num = 0;
      // std::cout << "local goal: \n" << local_goal_base_link << std::endl;
      // std::cout << "current_velocity: \n" << current_velocity << std::endl;
      Eigen::Vector3d goal(local_goal_base_link.pose.position.x, local_goal_base_link.pose.position.y,
        tf::getYaw(local_goal_base_link.pose.orientation));
      std::vector<Eigen::Vector3d> states;
      double target_velocity = planner.get_target_velocity(goal); // 根据 goal 位置选择前进还是后退
      // if (NEW_GOAL_SAMPLING) new_goal_sampling(goal, 0.3, states);
      if (NEW_GOAL_SAMPLING) parallel_goal_sampling(goal, 0.3, states);
      else planner.generate_biased_polar_states(N_S, goal, target_velocity, states);
      viz_goal_sampling(states);
      std::vector<MotionModelDiffDrive::Trajectory> trajectories;
      bool generated = planner.generate_trajectories(
        states, current_velocity.linear.x, current_velocity.angular.z, target_velocity, trajectories);
      if(ENABLE_SHARP_TRAJECTORY){ // 当前角速度加快
        generated |= planner.generate_trajectories(
          states, current_velocity.linear.x, current_velocity.angular.z + MAX_D_YAWRATE / HZ,
          target_velocity, trajectories);
        generated |= planner.generate_trajectories(
          states, current_velocity.linear.x, current_velocity.angular.z - MAX_D_YAWRATE / HZ,
          target_velocity, trajectories);
      }
      bool turn_flag = false;
      double relative_direction = atan2(local_goal_base_link.pose.position.y, local_goal_base_link.pose.position.x);
      if(goal.segment(0, 2).norm() < 0.1){ // 离目标欧氏距离小于 0.1 不需要生成路径
        generated = false;
      }else if(fabs(relative_direction) > TURN_DIRECTION_THRESHOLD){
        if(fabs(goal(2)) > TURN_DIRECTION_THRESHOLD){
          generated = false;
          turn_flag = true;
        }
      }
      if(generated) {
        // visualize_trajectories(trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);

        std::cout << "check candidate trajectories" << std::endl;
        std::vector<MotionModelDiffDrive::Trajectory> candidate_trajectories;
        state_lattice_planner::ObstacleMap<int> obstacle_map;
        get_obstacle_map(local_map, obstacle_map);
        update_pose_in_map(obstacle_map.get_map_frame());
        obstacle_map.set_yaw_diff(robot_in_map_(2));
        for(const auto& trajectory : trajectories){
          bool head_collision = false;
          if (HEAD > 0) { // 检查车头的碰撞
            std::vector<Eigen::Vector3d> head_trajectory;
            planner.generate_head_trajectory(trajectory.trajectory, HEAD, head_trajectory);
            head_collision = planner.check_collision(obstacle_map, head_trajectory, COLLISION_COST);
          }
          if(!planner.check_collision(obstacle_map, trajectory.trajectory, COLLISION_COST) &&
             !head_collision){
            candidate_trajectories.push_back(trajectory);
          }
        }
        std::cout << "trajectories: " << trajectories.size() << std::endl;
        std::cout << "candidate_trajectories: " << candidate_trajectories.size() << std::endl;
        // 挑选无碰撞的轨迹，//TODO@LMR为什么要两次
        // if(candidate_trajectories.empty()){
        //   // if no candidate trajectories
        //   // collision checking with relaxed restrictions
        //   for(const auto& trajectory : trajectories){
        //     if(!planner.check_collision(obstacle_map, trajectory.trajectory, IGNORABLE_OBSTACLE_RANGE)){
        //       candidate_trajectories.push_back(trajectory);
        //     }
        //   }
        //   std::cout << "candidate_trajectories(ignore far obstacles): " << candidate_trajectories.size() << std::endl;
        // }
        // std::cout << "candidate time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

        if(candidate_trajectories.size() > 0) {
          // 在无碰撞轨迹中挑选出最佳轨迹
          visualize_trajectories(
            candidate_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);

          std::cout << "pickup a optimal trajectory from candidate trajectories" << std::endl;
          MotionModelDiffDrive::Trajectory trajectory;
          // planner.pickup_trajectory(candidate_trajectories, goal, trajectory);
          critic_->pickup_trajectory(candidate_trajectories, goal, trajectory);
          visualize_trajectory(trajectory, 1, 0, 0, selected_trajectory_pub);
          PubBestTrajectory(trajectory.trajectory, "odom");
          std::cout << "pickup time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;

          // int size = trajectory.trajectory.size();
          // for(int i=0;i<size;i++){
          //   std::cout << trajectory.trajectory[i].transpose() << ", " <<
          //   trajectory.velocities[i] << "[m/s], " << trajectory.angular_velocities[i] << "[rad/s]" << std::endl;
          // }

          std::cout << "publish velocity" << std::endl;
          geometry_msgs::Twist cmd_vel;
          double calculation_time = ros::Time::now().toSec() - start;
          // 在轨迹点上选择控制节点
          int delayed_control_index = std::min(
            std::ceil(calculation_time * HZ) + CONTROL_DELAY, (double)trajectory.trajectory.size());
          if((int)trajectory.trajectory.size() < CONTROL_DELAY){
              delayed_control_index = std::ceil(calculation_time * HZ);
          }
          std::cout << calculation_time << ", " << delayed_control_index << std::endl;
          cmd_vel.linear.x = trajectory.velocities[delayed_control_index];
          cmd_vel.angular.z = trajectory.angular_velocities[delayed_control_index];
          if (ACKERMANN) cmd_vel = diff_cmd2ackermann_cmd(cmd_vel);
          if (DRIVE) velocity_pub.publish(cmd_vel);
          // std::cout << "published velocity: \n" << cmd_vel << std::endl;
          local_map_updated = false;
          odom_updated = false;
        } else { // 原有轨迹有碰撞都不能用了，机器暂停
          std::cout << "\033[91mERROR: trajectory collision !\033[00m" << std::endl;
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0;
          cmd_vel.angular.z = 0;
          if (ACKERMANN) cmd_vel = diff_cmd2ackermann_cmd(cmd_vel);
          if (DRIVE) velocity_pub.publish(cmd_vel);
          // for clear
          std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
          visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
          visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
          visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
        }
      } else {
        //TODO@LMR 原地旋转，阿克曼需要禁止
        std::cout << "\033[91mERROR: no optimized trajectory was generated\033[00m" << std::endl;
        std::cout << "\033[91mturn for local goal\033[00m" << std::endl;
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = 0;
        if(!turn_flag) { // 没有生成轨迹，要转向目标点朝向，//TODO@LMR有必要吗
          cmd_vel.angular.z = std::min(std::max(goal(2), -MAX_YAWRATE), MAX_YAWRATE);
        } else { // 转向目标点
          cmd_vel.angular.z = std::min(std::max(relative_direction, -MAX_YAWRATE), MAX_YAWRATE);
        }
        if (ACKERMANN) cmd_vel = diff_cmd2ackermann_cmd(cmd_vel);
        if (DRIVE) velocity_pub.publish(cmd_vel);
        // for clear
        std::vector<MotionModelDiffDrive::Trajectory> clear_trajectories;
        visualize_trajectories(clear_trajectories, 0, 1, 0, last_trajectory_num, candidate_trajectories_pub);
        visualize_trajectories(clear_trajectories, 0, 0.5, 1, last_trajectory_num, candidate_trajectories_no_collision_pub);
        visualize_trajectory(MotionModelDiffDrive::Trajectory(), 1, 0, 0, selected_trajectory_pub);
      }
      last_trajectory_num = trajectories.size();
      std::cout << "final time: " << ros::Time::now().toSec() - start << "[s]" << std::endl;
    }else{
      // if(!local_goal_subscribed){
      //   std::cout << "waiting for local goal" << std::endl;
      // }
      if(!local_map_updated){
        std::cout << "waiting for local map" << std::endl;
      }
      if(!odom_updated){
        std::cout << "waiting for odom" << std::endl;
      }
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}

void StateLatticePlannerROS::visualize_trajectories(const std::vector<MotionModelDiffDrive::Trajectory>& trajectories, const double r, const double g, const double b, const int trajectories_size, const ros::Publisher& pub)
{
    visualization_msgs::MarkerArray v_trajectories;
    int count = 0;
    const int size = trajectories.size();
    for(;count<size;count++){
        visualization_msgs::Marker v_trajectory;
        v_trajectory.header.frame_id = ROBOT_FRAME;
        v_trajectory.header.stamp = ros::Time::now();
        v_trajectory.color.r = r;
        v_trajectory.color.g = g;
        v_trajectory.color.b = b;
        v_trajectory.color.a = 0.8;
        v_trajectory.ns = pub.getTopic();
        v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
        v_trajectory.action = visualization_msgs::Marker::ADD;
        v_trajectory.lifetime = ros::Duration();
        v_trajectory.id = count;
        v_trajectory.pose.orientation.w = 1.0;
        v_trajectory.scale.x = 0.02;
        geometry_msgs::Point p;
        for(const auto& pose : trajectories[count].trajectory){
            p.x = pose(0);
            p.y = pose(1);
            v_trajectory.points.push_back(p);
        }
        v_trajectories.markers.push_back(v_trajectory);
    }
    for(;count<trajectories_size;){
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

void StateLatticePlannerROS::visualize_trajectory(const MotionModelDiffDrive::Trajectory& trajectory, const double r, const double g, const double b, const ros::Publisher& pub)
{
    visualization_msgs::Marker v_trajectory;
    v_trajectory.header.frame_id = ROBOT_FRAME;
    v_trajectory.header.stamp = ros::Time::now();
    v_trajectory.color.r = r;
    v_trajectory.color.g = g;
    v_trajectory.color.b = b;
    v_trajectory.color.a = 0.8;
    v_trajectory.ns = pub.getTopic();
    v_trajectory.type = visualization_msgs::Marker::LINE_STRIP;
    v_trajectory.action = visualization_msgs::Marker::ADD;
    v_trajectory.lifetime = ros::Duration();
    v_trajectory.pose.orientation.w = 1.0;
    v_trajectory.pose.position.z = 0.1;
    v_trajectory.scale.x = 0.10;
    geometry_msgs::Point p;
    for(const auto& pose : trajectory.trajectory){
        p.x = pose(0);
        p.y = pose(1);
        v_trajectory.points.push_back(p);
    }
    pub.publish(v_trajectory);
}

geometry_msgs::Twist StateLatticePlannerROS::diff_cmd2ackermann_cmd(const geometry_msgs::Twist& cmd_vel)
{
  double r = cmd_vel.linear.x / cmd_vel.angular.z;
  double s = atan(WHEEL_BASE / std::sqrt(r*r + WHEEL_BASE*WHEEL_BASE));
  s *= cmd_vel.angular.z >= 0 ? 1 : -1;
  geometry_msgs::Twist ackermann_cmd_vel = cmd_vel;
  ackermann_cmd_vel.angular.z = s;
  return ackermann_cmd_vel;
}

void StateLatticePlannerROS::init_xyyaw_table()
{
  pc_table_.header.stamp = ros::Time::now();
  pc_table_.header.frame_id = "base_link";
  geometry_msgs::Point32 p;
  for (auto pose : xyyaw_table_) {
    p.x = pose[0];
    p.y = pose[1];
    pc_table_.points.push_back(p);
  }
}

void StateLatticePlannerROS::viz_goal_sampling(const std::vector<Eigen::Vector3d>& states)
{
  geometry_msgs::PoseArray array;
  array.header.stamp = ros::Time::now();
  array.header.frame_id = "base_link";
  geometry_msgs::Pose p;
  for (auto s : states) {
    p.position.x = s[0]; p.position.y = s[1]; 
    auto q = tf::createQuaternionMsgFromRollPitchYaw(0, 0, s[2]);
    p.orientation = q;
    array.poses.push_back(p);
  }
  goal_sampling_pub.publish(array);
}

void StateLatticePlannerROS::new_goal_sampling(
  const Eigen::Vector3d goal, const float ratio, std::vector<Eigen::Vector3d>& states)
{
  double d = goal.segment(0,2).norm();
  double radius = std::min(d * ratio, NGS_R);
  std::vector<double> R(NGS_NR);
  double sum = 0;
  for (int i = 1; i <= NGS_NR; i ++)
    sum += i * i;
  for (int i = 1; i <= NGS_NR; i ++)
    R.emplace_back(i*i*radius/sum);
  double angle_step = 2 * M_PI / NGS_NA;
  double yaw_step = 2 * NGS_GY / (NGS_NY-1);
  double yaw_min = goal(2)-NGS_GY, yaw_max = goal(2)+NGS_GY;
  for (auto r : R) {
    for (float a = 0; a < 2*M_PI; a += angle_step) {
      for (float y = yaw_min; y <= yaw_max; y += yaw_step) {
        y = y > M_PI ? y - 2*M_PI : y;
        y = y < -M_PI ? y + 2*M_PI : y;
        Eigen::Vector3d s(goal(0) + r * cos(goal(2)+a),
                          goal(1) + r * sin(goal(2)+a),
                          y);
        states.push_back(s);
      }
    }
  }
  states.push_back(goal);
}

void StateLatticePlannerROS::parallel_goal_sampling(
  const Eigen::Vector3d goal, const float ratio, std::vector<Eigen::Vector3d>& states)
{
  double d = goal.segment(0,2).norm();
  double radius = std::min(std::max(d * ratio, NGS_R), 2 * NGS_R);
  // double radius = std::min(d * ratio, NGS_R);
  std::vector<double> R(NGS_NR);
  double sum = 0;
  for (int i = 1; i <= NGS_NR; i ++)
    sum += i*i;
  for (int i = 1; i <= NGS_NR; i ++)
    R.emplace_back(i*i*radius/sum);
  double yaw_step, yaw_min, yaw_max;
  if (NGS_NY == 1) {
    yaw_min = yaw_max = goal(2);
    yaw_step = M_PI;
  } else {
    yaw_step = 2 * NGS_GY / (NGS_NY-1);
    yaw_min = goal(2)-NGS_GY, yaw_max = goal(2)+NGS_GY;
  }
  for (auto r : R) {
    for (float y = yaw_min; y <= yaw_max; y += yaw_step) {
      y = y > M_PI ? y - 2*M_PI : y;
      y = y < -M_PI ? y + 2*M_PI : y;
      Eigen::Vector3d sl(goal(0) + r * cos(goal(2)+M_PI/2),
                         goal(1) + r * sin(goal(2)+M_PI/2),
                         y);
      states.push_back(sl);
      Eigen::Vector3d sr(goal(0) + r * cos(goal(2)-M_PI/2),
                         goal(1) + r * sin(goal(2)-M_PI/2),
                         y);
      states.push_back(sr);
    }
  }
  states.push_back(goal);
}

void StateLatticePlannerROS::PubBestTrajectory(
  const std::vector<Eigen::Vector3d>& trajectory, const std::string target_frame_id)
{
  // std::cout << "robot frame:" << ROBOT_FRAME << ", target frame:" << target_frame_id << std::endl;
  geometry_msgs::PoseStamped target_link;
  geometry_msgs::PoseStamped base_link;
  base_link.header.frame_id = ROBOT_FRAME;
  base_link.header.stamp = ros::Time::now();
  base_link.pose.orientation.w = 1;
  try{
    listener.transformPose(target_frame_id, ros::Time(0), base_link, ROBOT_FRAME, target_link);
  }catch(tf2::TransformException& ex){
    std::cout << ex.what() << std::endl;
    return;
  }

  Eigen::Vector3d target(target_link.pose.position.x, target_link.pose.position.y,
    tf::getYaw(target_link.pose.orientation));

  nav_msgs::Path path;
  path.header.frame_id = target_frame_id;
  path.header.stamp = ros::Time::now();
  geometry_msgs::PoseStamped pose;
  pose.header = path.header;
  path.poses.reserve(trajectory.size());
  for (auto p : trajectory) {
    pose.pose.position.x = target(0) + cos(target(2)) * p(0) - sin(target(2)) * p(1);
    pose.pose.position.y = target(1) + sin(target(2)) * p(0) + cos(target(2)) * p(1);
    double yaw = target(2) + p(2);
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
    path.poses.emplace_back(pose);
  }
  best_trajectory_pub.publish(path);
}

void StateLatticePlannerROS::update_pose_in_map(const std::string map_frame_id)
{
  geometry_msgs::PoseStamped target_link;
  geometry_msgs::PoseStamped base_link;
  base_link.header.frame_id = ROBOT_FRAME;
  base_link.header.stamp = ros::Time::now();
  base_link.pose.orientation.w = 1;
  try{
    listener.transformPose(map_frame_id, ros::Time(0), base_link, ROBOT_FRAME, robot_pose_in_map_);
  }catch(tf2::TransformException& ex){
    std::cout << ex.what() << std::endl;
    return;
  }

  robot_in_map_(0) = robot_pose_in_map_.pose.position.x;
  robot_in_map_(1) = robot_pose_in_map_.pose.position.y;
  robot_in_map_(2) = tf::getYaw(robot_pose_in_map_.pose.orientation);
}

template<typename TYPE>
void StateLatticePlannerROS::get_obstacle_map(const nav_msgs::OccupancyGrid& input_map, state_lattice_planner::ObstacleMap<TYPE>& output_map)
{
    output_map.set_frame_id(input_map.header.frame_id);
    output_map.set_shape(input_map.info.width, input_map.info.height, input_map.info.resolution);
    output_map.data.clear();
    for(const auto& data : input_map.data){
        output_map.data.emplace_back(data);
    }
}
