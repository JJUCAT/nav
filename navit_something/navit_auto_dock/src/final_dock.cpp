#include <navit_auto_dock/final_dock.h>
#include <tf/tf.h>

namespace navit_auto_dock {
FinalDock::FinalDock(const std::string& name, 
                     std::shared_ptr<tf2_ros::Buffer> &tf,
                     ros::NodeHandle& nh):
      nh_(nh),
      tf_buffer_(tf),
      final_dock_as_(nh_, "/final_dock",
                     boost::bind(&FinalDock::executeCallback, this, _1), false),
      rotate_controller_loader_("navit_auto_dock",
                                "navit_auto_dock::plugins::RotateController"),
      final_dock_controller_loader_(
          "navit_auto_dock", "navit_auto_dock::plugins::FinalDockController") {
  ros::NodeHandle pnh(name);


  pnh.param("ref_terminal_x", ref_terminal_x_, ref_terminal_x_);
  pnh.param("ref_terminal_y", ref_terminal_y_, ref_terminal_y_);
  pnh.param("ref_terminal_theta", ref_terminal_theta_, ref_terminal_theta_);

  std::string rotate_controller_name = "RotatePidController";
  pnh.param("rotate_controller_name", rotate_controller_name, rotate_controller_name);
  try {
    rotate_controller_ =
        rotate_controller_loader_.createInstance(rotate_controller_name);
    rotate_controller_->initialize(
        rotate_controller_loader_.getName(rotate_controller_name),
        tf_buffer_);
    ROS_DEBUG_STREAM("loaded rotate pid controller: " << rotate_controller_name);
  } catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL("failed to load rotate controller %s", ex.what());
    exit(1);
  }

  std::string final_dock_controller_name = "PidController";
  pnh.param("final_dock_controller_name", final_dock_controller_name,
            final_dock_controller_name);
  // load final dock controller
  try {
    final_dock_controller_ = final_dock_controller_loader_.createInstance(
        final_dock_controller_name);
    final_dock_controller_->initialize(
        final_dock_controller_loader_.getName(final_dock_controller_name),
        tf_buffer_);
  } catch (const pluginlib::PluginlibException &ex) {
    ROS_FATAL("failed to load final dock controller %s", ex.what());
    exit(1);
  }

  odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
      "/odom", 1, boost::bind(&FinalDock::odomCallback, this, _1));
  cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  terminal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("terminal", 1);
  dock_ref_line_pub_ = nh_.advertise<nav_msgs::Path>("dock_ref_line", 1);
  final_dock_as_.start();
  ROS_INFO("Final dock action server started");
}

void FinalDock::executeCallback(const navit_msgs::FinalDockGoalConstPtr &goal) {
  navit_msgs::FinalDockFeedback feedback;
  navit_msgs::FinalDockResult result;

  result.docked = false;

  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped pose, dock;
  pose = goal->dock_pose;
  pose.header.stamp = ros::Time::now();

  ROS_INFO("[FD] FinalDock: Goal received");
  ROS_INFO("[FD] dock pose [%s][%f, %f, %f]",
    pose.header.frame_id.c_str(), pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
  toGlobalFrame(pose, dock);
  ROS_INFO("[FD] dock pose in map [%s][%f, %f, %f]",
    dock.header.frame_id.c_str(), dock.pose.position.x, dock.pose.position.y, tf2::getYaw(dock.pose.orientation));
  feedback.dock_pose_feedback = dock;

  // 由充电桩的感知位姿计算回充参考终点
  calculateTerminalFromDock(dock, target);
  terminal_pub_.publish(target);
  ROS_INFO("[FD] terminal pose [%s][%f, %f, %f]",
    target.header.frame_id.c_str() , target.pose.position.x, target.pose.position.y, tf2::getYaw(target.pose.orientation));
  // vizDockRefLine(dock, 8); // 显示回充参考线

  double theta_err_init = getThetaToDockRefLine(dock)/M_PI*180.f;
  double y_err_init = getDistanceToDockRefLine(dock);

  // TODO: validate goal
  rotate_in_place_=goal->rotate_in_place;
  
  ros::Rate r(20.0);

  if (rotate_in_place_) {
    rotate_controller_->setTargetPose(target);
    ROS_INFO("[FD] Rotating...");
    // blocking util rotate 180 deg
    if (!rotate_controller_->rotateToTarget(target)) {
      result.docked = false;
      final_dock_as_.setAborted(result, "failed to rotate");
      return;
    }
  }

  final_dock_controller_->setTargetPose(target);
  while (nh_.ok()) {
    if (final_dock_as_.isPreemptRequested())
    {
        publishZeroCmdVel();
        ROS_WARN("[FD] Preempt requested!");
        result.docked = false;
        final_dock_as_.setPreempted(result, "action is preempted");
        return;
    }

    if (final_dock_controller_->isGoalReached()) {
      double theta_err_finished = getThetaToDockRefLine(dock)/M_PI*180.f;
      double y_err_finished = getDistanceToDockRefLine(dock);
      ROS_WARN("[FD] ========== final dock finished ==========");
      ROS_WARN("[FD] y err init %f m, theta err init %f angular", y_err_init, theta_err_init);
      ROS_WARN("[FD] y err finished %f m, y head err finished %f m, theta err finished %f angular",
        y_err_finished, 0.95*sin(theta_err_finished*M_PI/180.f), theta_err_finished);
      ROS_WARN("[FD] adjust y err %f m, theta err %f angular",
        y_err_finished-y_err_init, theta_err_finished-theta_err_init);

      result.docked = true;
      ROS_INFO("[FD] FinalDock: Goal Reached!");
      cmd_pub_.publish(geometry_msgs::Twist());
      final_dock_as_.setSucceeded(result);
      return;
    }

    final_dock_controller_->computeVelocityCommands(cmd_vel_);
    cmd_pub_.publish(cmd_vel_);

    feedback.command = cmd_vel_;
    final_dock_as_.publishFeedback(feedback);

    double theta_err = getThetaToDockRefLine(dock);
    double dist = getDistanceToTarget(target);
    ROS_DEBUG("[FD] y err %f m, theta err %f angular, dist to target %f m",
      getDistanceToDockRefLine(dock), theta_err/M_PI*180.f, dist);
    r.sleep();
  }
}

void FinalDock::toGlobalFrame(const geometry_msgs::PoseStamped &local_pose,
                              geometry_msgs::PoseStamped &global_pose) {
  global_pose.header.frame_id = "odom";
  if (local_pose.header.frame_id != global_pose.header.frame_id) {
    try {
      tf_buffer_->transform(local_pose, global_pose,
global_pose.header.frame_id, ros::Duration(0.5));
      ROS_DEBUG("to global frame");
    } catch (tf2::TransformException &ex) {
      ROS_ERROR("Failed to transform from %s to %s", local_pose.header.frame_id.c_str(),
                global_pose.header.frame_id.c_str());
      ROS_WARN_STREAM("Failed to transform from "
                  << local_pose.header.frame_id << " to "
                  << global_pose.header.frame_id << ": " << ex.what());
    }
  } else {
    global_pose = local_pose;
  }
}

void FinalDock::calculateTerminalFromDock(
  const geometry_msgs::PoseStamped& dock, geometry_msgs::PoseStamped& terminal) {
  tf2::Transform dock_in_map, terminal_in_map;
  tf2::fromMsg(dock.pose, dock_in_map);

  tf2::Transform terminal_to_dock_tf;
  terminal_to_dock_tf.setOrigin(tf2::Vector3(
    ref_terminal_x_, ref_terminal_y_, 0.0));
  tf2::Quaternion q;  
  q.setRPY(0.0, 0.0, ref_terminal_theta_); 
  terminal_to_dock_tf.setRotation(q);
  
  terminal_in_map = dock_in_map*terminal_to_dock_tf;
  tf2::toMsg(terminal_in_map, terminal.pose);
  terminal.header=dock.header;
}

void FinalDock::vizDockRefLine(
  const geometry_msgs::PoseStamped& dock, const double len) {
  tf2::Transform dock_in_map, terminal_in_map;
  tf2::fromMsg(dock.pose, dock_in_map);

  tf2::Transform terminal_to_dock_tf;
  terminal_to_dock_tf.setOrigin(tf2::Vector3(-len, 0.0, 0.0));
  tf2::Quaternion q;  
  q.setRPY(0.0, 0.0, 0.0); 
  terminal_to_dock_tf.setRotation(q);
  
  terminal_in_map = dock_in_map*terminal_to_dock_tf;
  geometry_msgs::PoseStamped ref_line_end = dock;
  tf2::toMsg(terminal_in_map, ref_line_end.pose);
  ref_line_end.header=dock.header;

  nav_msgs::Path dock_ref_line;
  dock_ref_line.header=dock.header;
  dock_ref_line.poses.push_back(dock);
  dock_ref_line.poses.push_back(ref_line_end);
  dock_ref_line_pub_.publish(dock_ref_line);
}

void FinalDock::odomCallback(const nav_msgs::OdometryConstPtr &msg) {
  odom_ = *msg;
  odom_update_ = true;
}

double FinalDock::getDistanceToDockRefLine(const geometry_msgs::PoseStamped& dock) {
  if (!odom_update_) return -999.999;
  tf2::Transform odom_in_dock, dock_in_odom, robot_in_odom, robot_in_dock;
  tf2::fromMsg(dock.pose, dock_in_odom);
  odom_in_dock = dock_in_odom.inverse();

  tf2::fromMsg(odom_.pose.pose, robot_in_odom);
  
  robot_in_dock = odom_in_dock*robot_in_odom;
  geometry_msgs::PoseStamped robot;
  tf2::toMsg(robot_in_dock, robot.pose);
  return robot.pose.position.y;
}

double FinalDock::getThetaToDockRefLine(const geometry_msgs::PoseStamped& dock) {
  if (!odom_update_) return -999.999;
  double dock_yaw = tf::getYaw(dock.pose.orientation);
  double robot_yaw = tf::getYaw(odom_.pose.pose.orientation);
  return dock_yaw - robot_yaw;
}

double FinalDock::getDistanceToTarget(const geometry_msgs::PoseStamped& target) {
  if (!odom_update_) return -999.999;
  auto robot = odom_.pose.pose;
  return std::hypot(target.pose.position.x - robot.position.x,
                    target.pose.position.y - robot.position.y);
}

bool FinalDock::LogOdomFrameInMapFrame(geometry_msgs::PoseStamped& odom_in_map) {
  std::string local_frame = "odom";
  geometry_msgs::PoseStamped odom;
  odom.header.frame_id = local_frame;
  odom.header.stamp = ros::Time::now();
  tf2::Quaternion q; q.setRPY(0.0, 0.0, 0.0);
  odom.pose.orientation = tf2::toMsg(q);

  try {
    tf_buffer_->transform(odom, odom_in_map, "map");
    ROS_INFO("[FD] odom in map [%f, %f, %f]",
      odom_in_map.pose.position.x, odom_in_map.pose.position.y,
      tf2::getYaw(odom_in_map.pose.orientation));
  } catch (const tf2::TransformException& ex) {
    ROS_WARN("[FD] get odom in map failed !");
    return false;
  }
  return true;
}

}
