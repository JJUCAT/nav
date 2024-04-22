#ifndef APPAROCH_DOCK_H
#define APPAROCH_DOCK_H

#include <actionlib/server/simple_action_server.h>
#include <ros/ros.h>

#include <navit_auto_dock/plugins/approach_dock_perception.h>
#include <navit_msgs/ApproachDockAction.h>
#include <nav_msgs/Odometry.h>
#include <navit_core/abstract_plugin_manager.h>
#include <navit_costmap/costmap_2d_ros.h>

#include <pluginlib/class_loader.hpp>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <tf2/utils.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#include <opencv2/opencv.hpp>
#include <cmath>


namespace navit_auto_dock {
typedef plugins::ApproachDockPerception::Ptr PerceptionPtr;

class CalibrateDock {
  typedef actionlib::SimpleActionServer<navit_msgs::ApproachDockAction> approach_dock_server_t;

  typedef pluginlib::ClassLoader< plugins::ApproachDockPerception> perception_loader_t;

  typedef navit_core::AbstractPluginManager<plugins::ApproachDockPerception> approach_perception_pm_t;

  typedef navit_msgs::ApproachDockResult Result;


public:
  CalibrateDock(const std::string& name,
               std::shared_ptr<tf2_ros::Buffer> &tf_buffer,
               ros::NodeHandle& nh);
  ~CalibrateDock()
  {
      approach_perception_pm_.clearPlugins();

      perception_.reset();
  }

private:
  void executeCallback(const navit_msgs::ApproachDockGoalConstPtr &goal);

  
  PerceptionPtr loadPerceptionPlugins(const std::string& plugin);

  bool initPerceptionPlugins(const std::string& plugin,
                             const PerceptionPtr& perception_ptr);

  ros::NodeHandle nh_;
  Result result_;
  
  float yaw_offset;

  approach_dock_server_t approach_dock_as_;

  // plugin manager
  approach_perception_pm_t approach_perception_pm_;

  perception_loader_t perception_loader_;

  PerceptionPtr perception_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
};
}

#endif
