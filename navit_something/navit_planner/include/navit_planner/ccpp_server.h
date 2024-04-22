#ifndef CCPP_SERVER_H
#define CCPP_SERVER_H

#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <navit_core/abstract_plugin_manager.h>
#include <navit_core/base_global_planner.h>
#include <navit_msgs/CoveragePathOnPWHAction.h>
#include <tf/tf.h>

#include <pluginlib/class_loader.hpp>

namespace navit_planner {

using CoveragePlannerPtr = navit_core::CoveragePlanner::Ptr;

class CoveragePlannerServer {
 public:
  explicit CoveragePlannerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer,
                                 ros::NodeHandle& nh,
                                 const std::string& param_ns = "~");

  ~CoveragePlannerServer();

  int getPlan(const geometry_msgs::Polygon& coverage_area,
              const std::vector<geometry_msgs::Polygon>& holes,
              const geometry_msgs::Pose& start, const geometry_msgs::Pose& end,
              const std::string& plugin_name, nav_msgs::Path& planned_path);

 protected:

  using ActionType = navit_msgs::CoveragePathOnPWHAction;
  using ActionGoal = navit_msgs::CoveragePathOnPWHGoalConstPtr;
  using ActionResult = navit_msgs::CoveragePathOnPWHResult;
  using ActionServer = actionlib::SimpleActionServer<ActionType>;
  using PluginManager =
      navit_core::AbstractPluginManager<navit_core::CoveragePlanner>;

  void computePath(const ActionGoal& goal);
  CoveragePlannerPtr loadPlugin(const std::string& plugin_name);
  bool initPlugin(const std::string& plugin_name,
                  const CoveragePlannerPtr& planner_ptr);

  ros::NodeHandle nh_;
  ros::Publisher plan_pub_;

  PluginManager plugin_manager_;
  pluginlib::ClassLoader<navit_core::CoveragePlanner> plugin_loader_;
  std::shared_ptr<ActionServer> action_server_;
  std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
};
}  // namespace navit_planner
#endif
