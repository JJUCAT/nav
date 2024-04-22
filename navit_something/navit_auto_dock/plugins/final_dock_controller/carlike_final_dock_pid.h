#ifndef PLUGINS_FINAL_DOCK_PID_H
#define PLUGINS_FINAL_DOCK_PID_H

#include <navit_auto_dock/plugins/final_dock_controller.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h> 
#include <pcl/filters/impl/passthrough.hpp> 
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp> 
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

namespace navit_auto_dock {
namespace plugins {
class CarlikePidController : public FinalDockController {
public:
  void initialize(std::string name, std::shared_ptr<tf2_ros::Buffer> &tf);

  void setTargetPose(geometry_msgs::PoseStamped &target);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

private:
  /**
   * @brief 更新机器在 target 坐标系中的位姿
   * @param  target           My Param doc
   * @return true 
   * @return false 
   */
  bool updatePoseInTarget(const geometry_msgs::PoseStamped& target);

  /**
   * @brief 获取机器相对 target 的横向偏移
   * @return double 
   */
  double getDistanceToDockRefLine();

  /**
   * @brief 获取机器相对 target 的航向角偏移
   * @return double 
   */
  double getThetaToDockRefLine();

  /**
   * @brief 获取 target 方向，负值表示 target 在机器后方，暂时用于退桩
   * @return int 
   */
  int getDirectionToDockRefLine();

  /**
   * @brief 更新 final dock 要走的距离
   */
  void updateTargetDistance();

  /**
   * @brief 更新机器已走距离
   */
  void updateJourney();

  geometry_msgs::PoseStamped target_, current_, pose_in_target_;
  nav_msgs::Odometry odom_;
  nav_msgs::Odometry last_odom_;
  ros::Time lidar_callback_time_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  ros::NodeHandle nh_;
  ros::Subscriber odom_sub_;
  ros::Subscriber lidar_sub_;

  struct {
    std::string odom_topic_name = "odom";
    double steering_max = 1.2;
    double kp_y_err = 0.5;
    double kp_theta_err = 1.5;
    double tolerance = 0.01;
    double max_v = 0.3;
    double min_v = 0.1;
    double target_x_offset_to_dock = -1;
    double hook = 0.0;
  } config_;

  struct {
    std::string topic_name = "lidar";
    double valid_x = 1.0;
    int buffer_size = 10;
    double x_min;
    double x_max;
    double y_min;
    double y_max;
    double z_min;
    double z_max;
    double i_min;
    double i_max;
  } lidar_config_;

  double error_ = 999.999;
  double journey_ = 0;
  double journey_history_ = 0;
  double target_distance_ = std::numeric_limits<double>::lowest();
  std::mutex lidar_mutex_;
  double x_in_lidar_ = std::numeric_limits<double>::lowest();
  bool moveback_ = false;
  int y_err_before_moveback_ = 1;
  bool odom_update_ = false;
  std::vector<double> x_min_buffer_;

  void odomCallback(const nav_msgs::OdometryConstPtr &msg);
  void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &msg);

};
}
}

#endif
