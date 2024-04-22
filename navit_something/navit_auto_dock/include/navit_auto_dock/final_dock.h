#ifndef FINAL_DOCK_H
#define FINAL_DOCK_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <navit_msgs/FinalDockAction.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>

#include <pluginlib/class_loader.hpp>

#include <navit_auto_dock/plugins/final_dock_controller.h>
#include <navit_auto_dock/plugins/rotate_controller.h>

#include <navit_collision_checker/collision_checker.h>

namespace navit_auto_dock {

class FinalDock {
  typedef actionlib::SimpleActionServer<navit_msgs::FinalDockAction>
      final_dock_as_t;
  typedef pluginlib::ClassLoader<plugins::RotateController>
      rotate_controller_loader_t;
  typedef pluginlib::ClassLoader<plugins::FinalDockController>
      final_dock_controller_loader_t;
  typedef boost::shared_ptr<plugins::RotateController> rotate_controller_t;
  typedef boost::shared_ptr<plugins::FinalDockController> final_dock_controller_t;

public:
  FinalDock(const std::string& name, 
            std::shared_ptr<tf2_ros::Buffer> &tf,
            ros::NodeHandle& nh);

private:
  void executeCallback(const navit_msgs::FinalDockGoalConstPtr &goal);
  void toGlobalFrame(const geometry_msgs::PoseStamped &local_pose,
                     geometry_msgs::PoseStamped &global_pose);
  void calculateTerminalFromDock(const geometry_msgs::PoseStamped& dock,
    geometry_msgs::PoseStamped& terminal);
  void vizDockRefLine(const geometry_msgs::PoseStamped& dock, const double len);
  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  double getDistanceToDockRefLine(const geometry_msgs::PoseStamped& dock);
  double getThetaToDockRefLine(const geometry_msgs::PoseStamped& dock);
  double getDistanceToTarget(const geometry_msgs::PoseStamped& target);
  bool LogOdomFrameInMapFrame(geometry_msgs::PoseStamped& odom_in_map);

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher cmd_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  ros::Publisher terminal_pub_;
  ros::Publisher dock_ref_line_pub_;

  bool odom_update_ = false;
  nav_msgs::Odometry odom_;
  geometry_msgs::Twist cmd_vel_;
  geometry_msgs::PoseStamped target_pose_;

  bool rotate_in_place_;
  // 回充参考终点，坐标基于充电桩坐标系
  double ref_terminal_x_= -0.3;
  double ref_terminal_y_= 0.0;
  double ref_terminal_theta_= 0.0;

  final_dock_as_t final_dock_as_;
  rotate_controller_loader_t rotate_controller_loader_;
  final_dock_controller_loader_t final_dock_controller_loader_;
  rotate_controller_t rotate_controller_;
  final_dock_controller_t final_dock_controller_;

  inline void publishZeroCmdVel()
  {
    cmd_pub_.publish(geometry_msgs::Twist());
  }
};
}

#endif
