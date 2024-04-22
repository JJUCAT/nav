#ifndef FCPP_SERVER_H
#define FCPP_SERVER_H

#include <navit_core/base_global_planner.h>
#include <navit_core/abstract_plugin_manager.h>
#include <navit_msgs/CoveragePathOnPWHAction.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pluginlib/class_loader.hpp>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <navit_planner/planner_server_helper.h>

#include <tf/tf.h>
namespace navit_planner
{
using PolygonCoveragePlannerPtr = navit_core::CoveragePlanner::Ptr;
using PolygonCoverageActionType = navit_msgs::CoveragePathOnPWHAction;
using PolygonCoverageActionGoal = navit_msgs::CoveragePathOnPWHGoalConstPtr;
using PolygonCoverageActionResult = navit_msgs::CoveragePathOnPWHResult;
using PolygonCoverageActionServer = actionlib::SimpleActionServer<PolygonCoverageActionType>;

class PolygonCoverageServer
{
public:
    using PolygonCoveragePlanner = navit_core::CoveragePlanner;
    using PlannerLoader = pluginlib::ClassLoader<PolygonCoveragePlanner>;
    using PlannerPluginManager = navit_core::AbstractPluginManager<PolygonCoveragePlanner>;

    PolygonCoverageServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh, const std::string& param_ns = "~");

    ~PolygonCoverageServer() {
        planner_plugin_manager_.clearPlugins();
        planner_.reset();
    }

private:
    ros::NodeHandle nh_;

    void computePath(const PolygonCoverageActionGoal& goal);
    bool checkPath(nav_msgs::Path& path);
    bool getPlan(const geometry_msgs::Polygon& coverage_area,
                const std::vector<geometry_msgs::Polygon>& holes,
                const geometry_msgs::Pose& start,
                const geometry_msgs::Pose& end,
                const std::string& plugin,
                std::vector<nav_msgs::Path>& planned_coverage_path,
                std::vector<nav_msgs::Path>& planned_wall_path);
    void planPathBetweenPoints(nav_msgs::Path& input_path,
                                double max_angle_threshold,
                                int num_points_before_after);
    double calculateAngleBetweenPoses(const geometry_msgs::Pose& pose1,
                                      const geometry_msgs::Pose& pose2,
                                      const geometry_msgs::Pose& pose3);
    PolygonCoveragePlannerPtr planner_;
    PlannerPluginManager planner_plugin_manager_;
    PlannerLoader planner_loader_;
    PolygonCoveragePlannerPtr loadPlannerPlugins(const std::string& plugin);
    bool initPlannerPlugins(const std::string& plugin, const PolygonCoveragePlannerPtr& planner_ptr);
    std::shared_ptr<PolygonCoverageActionServer> as_;
    std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;

    navit_planner::PlannerServerHelper::Ptr planner_server_helper_;
    navit_planner::OmplRos::Ptr ompl_ros_ptr_;

    float expand_boundary_, contract_boundary_, simplification_dis_, sweep_step_;
    bool use_dubins_ = false;
    double max_angle_threshold_ = 150.0;
    int num_points_before_after_ = 15;
    ros::Publisher coverage_plan_pub_, wall_plan_pub_;
    ros::Publisher coverage_area_pub_, holes_pub_, rehanld_path_pub_;
};
}  // namespace navit_planner
#endif
