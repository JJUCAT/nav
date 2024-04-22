#ifndef APPAROCH_DOCK_H
#define APPAROCH_DOCK_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <navit_auto_dock/plugins/approach_dock_controller.h>
#include <navit_auto_dock/plugins/approach_dock_perception.h>
#include <navit_auto_dock/plugins/approach_dock_filter.h>
#include <navit_msgs/ApproachDockAction.h>
#include <nav_msgs/Odometry.h>
#include <navit_core/abstract_plugin_manager.h>
#include <navit_costmap/costmap_2d_ros.h>

#include <pluginlib/class_loader.hpp>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// deprecated
//#warning navit_collision_checker/collision_checker.h has been deprecated
#include <navit_collision_checker/collision_checker.h>
#include <navit_collision_checker/footprint_collision_checker.h>

#include <opencv2/opencv.hpp>
#include <cmath>

namespace navit_auto_dock {
typedef plugins::ApproachDockController::Ptr ControllerPtr;
typedef plugins::ApproachDockPerception::Ptr PerceptionPtr;
typedef plugins::ApproachDockFilter::Ptr FilterPtr;

class ApproachDock {
  typedef actionlib::SimpleActionServer<navit_msgs::ApproachDockAction> approach_dock_server_t;

  typedef pluginlib::ClassLoader< plugins::ApproachDockController> controller_loader_t;
  typedef pluginlib::ClassLoader< plugins::ApproachDockPerception> perception_loader_t;
  typedef pluginlib::ClassLoader< plugins::ApproachDockFilter> filter_loader_t;

  typedef navit_core::AbstractPluginManager<plugins::ApproachDockController> approach_controller_pm_t;
  typedef navit_core::AbstractPluginManager<plugins::ApproachDockPerception> approach_perception_pm_t;
  typedef navit_core::AbstractPluginManager<plugins::ApproachDockFilter> approach_filter_pm_t;

  // configs
  struct config_t {
    std::string controller_name = "dock_fg100_controller/FgController";
    std::string perception_name = "dock_fake_perception/FakePerception";
    std::string filter_name = "PerceptionEKF";
    double control_frequency = 5.0;
    std::string global_frame = "odom";
    std::string odom_topic_name = "odom";
    std::string cmd_vel_name = "cmd_vel";
    double obstacle_blocking_timeout = 15.0;
    double collision_check_forward_time = 0.5;
    double collision_check_forward_distance = 0.5;
    double lost_perception_timeout = 20.0;
    double pre_in_percep_x_=-0.3;
    double pre_in_percep_y_=0.0;
    double pre_in_percep_theta_=0.0;
  };

  // action feedback and results
  typedef navit_msgs::ApproachDockFeedback Feedback;
  typedef navit_msgs::ApproachDockResult Result;

public:
ROS_DEPRECATED ApproachDock(const std::string& name, std::shared_ptr<tf2_ros::Buffer> &tf_buffer, std::shared_ptr<navit_collision_checker::CollisionChecker>& collision_checker, ros::NodeHandle& nh);

  ApproachDock(const std::string& name,
               std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
               ros::NodeHandle& nh);
  ~ApproachDock()
  {
      approach_controller_pm_.clearPlugins();
      approach_perception_pm_.clearPlugins();
      approach_filter_pm_.clearPlugins();

      controller_.reset();
      perception_.reset();
      filter_.reset();
  }

private:
  void executeCallback(const navit_msgs::ApproachDockGoalConstPtr &goal);

  void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg);

  ControllerPtr loadControlPlugins(const std::string& plugin);
  PerceptionPtr loadPerceptionPlugins(const std::string& plugin);
  FilterPtr loadFilterPlugins(const std::string& plugin);

  bool initControlPlugins(const std::string& plugin, 
                          const ControllerPtr& controller_ptr);
  bool initPerceptionPlugins(const std::string& plugin,
                             const PerceptionPtr& perception_ptr);
  bool initFilterPlugins(const std::string& plugin,
                         const FilterPtr& filter_ptr);
  
  Feedback feedback_;
  Result result_;

  ros::NodeHandle nh_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  approach_dock_server_t approach_dock_as_;

  // plugin manager
  approach_controller_pm_t approach_controller_pm_;
  approach_perception_pm_t approach_perception_pm_;
  approach_filter_pm_t approach_filter_pm_;

  controller_loader_t controller_loader_;
  perception_loader_t perception_loader_;
  filter_loader_t filter_loader_;

  ControllerPtr controller_, default_controller_;
  PerceptionPtr perception_, default_perception_;
  FilterPtr filter_, default_filter_;

  ros::Publisher cmd_vel_pub_;
  geometry_msgs::Twist cmd_vel_, current_vel_;
  geometry_msgs::PoseStamped current_pose_;

  ros::Subscriber odom_sub_;
  config_t approach_dock_config;

  // navit_msgs/ApproachDockGoal接口相关变量
  double offset_distance_;

  void toGlobalFrame(const geometry_msgs::PoseStamped &local_pose,
                     geometry_msgs::PoseStamped &global_pose);

  void getDockToTargetTF(const geometry_msgs::PoseStamped &dock, 
                         const geometry_msgs::PoseStamped &target,
                         tf2::Transform &dock_to_target_tf);

  void fixTargetPose(const geometry_msgs::PoseStamped &dock,
                     const tf2::Transform &dock_to_target_tf,
                     geometry_msgs::PoseStamped &target);

  void calcPreposeFromPercpose(const geometry_msgs::PoseStamped &perception_dock,
                               geometry_msgs::PoseStamped &pre_charging_pose);
  
  bool toChargePile(const geometry_msgs::PoseStamped& dock_pose,
                          geometry_msgs::PoseStamped& charge_port_pose);
  inline void publishZeroCmdVel();

  // collision checker
  std::shared_ptr<navit_collision_checker::FootprintCollisionChecker<navit_costmap::Costmap2D*> > collision_checker_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;

  bool collisionCheckForwardPath(double distance, double forward_time);

  ros::Publisher first_perception_pub_;
  ros::Publisher dock_pose_pub_;
  ros::Publisher ref_dock_in_map_pub_;
  ros::Publisher filtered_dock_pose_pub_;
  ros::Publisher precharging_pose_pub_;
  ros::Publisher collision_check_poses_pub_;

  bool collisionCheckPose(const geometry_msgs::PoseStamped& check_pose)
  {
    if (check_pose.header.frame_id != costmap_->getGlobalFrameID())
    {
      ROS_WARN("please use pose in %s frame", (costmap_->getGlobalFrameID()).c_str());
      return true;
    }
    double yaw = tf2::getYaw(check_pose.pose.orientation);
    double x = check_pose.pose.position.x;
    double y = check_pose.pose.position.y;
    //ROS_DEBUG("check_pose.pose.position.x %lf check_pose.pose.position.y %lf,yaw=%lf ,cost=%lf",x,y,yaw,collision_checker_->footprintCostAtPose(x,y,yaw,costmap_->getRobotFootprint()));
  
    if ( collision_checker_->footprintCostAtPose(x,y,yaw,costmap_->getRobotFootprint()) >= 
                      navit_costmap::LETHAL_OBSTACLE )
    {
      ROS_DEBUG("pose in collision detected,cost=%lf \n",collision_checker_->footprintCostAtPose(x,y,yaw,costmap_->getRobotFootprint()));
      return true;
    }
    
    return false;
  }

  bool LogOdomFrameInMapFrame(geometry_msgs::PoseStamped& odom_in_map);
  geometry_msgs::Twist SlowDown(const geometry_msgs::PoseStamped& precharging_pose,
    const geometry_msgs::Twist& cmd_vel, const double slow_down_rang,
    const double v_max, const double v_min);

  /**
   * @brief 暂停机器后，等待然后感知更新充电桩
   * @param  dock  之前感知的充电桩位姿
   * @param  wait_time  更新时间
   * @return geometry_msgs::PoseStamped  新充电桩位姿
   */
  geometry_msgs::PoseStamped WaitAndUpdateDock(
    const geometry_msgs::PoseStamped dock, const double wait_time);
};
}

#endif
