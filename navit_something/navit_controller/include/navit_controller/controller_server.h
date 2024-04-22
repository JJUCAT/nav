#ifndef CONTROLLER_SERVER_H
#define CONTROLLER_SERVER_H

#include <ros/ros.h>

#include <navit_core/base_controller.h>
#include <navit_core/abstract_plugin_manager.h>

#include <tf2_ros/transform_listener.h>
#include <navit_costmap/costmap_2d_ros.h>

#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <navit_msgs/FollowPathAction.h>
#include <navit_msgs/ToggleAvoidance.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.hpp>

#include <std_srvs/Empty.h>

#include <navit_controller/controller_stats.h>
#include <navit_collision_checker/collision_checker.h>
#include <navit_collision_checker/footprint_collision_checker.h>
#include <tf2/utils.h>
#include <navit_localplanner/local_planner/local_planner.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <navit_common/path_handle.h>
#include "dynamic_reconfigure/server.h"
#include <navit_controller/NavitControllerReconfigureConfig.h>

namespace navit_controller {

using Controller = navit_core::Controller;
using ControllerPtr = navit_core::Controller::Ptr;
using ActionType = navit_msgs::FollowPathAction;
using ActionGoal = navit_msgs::FollowPathGoalConstPtr;
using ActionResult = navit_msgs::FollowPathResult;
using ActionFeedback = navit_msgs::FollowPathFeedback;
using ActionServer = actionlib::SimpleActionServer<ActionType>;

/** @class ControllerServer
 * @brief A class to manage the controller plugins and provide an action server
 * for the FollowPath action.
 */
class ControllerServer
{
    public:
        /** @brief Controller plugin loader
         * @details This class is used to load the controller plugins.
         * It is used to load the plugins and to clear them.
         * @see ControllerServer::loadControllerPlugin
         * @see ControllerServer::controller_loader_
         */
        using ControllerLoader = pluginlib::ClassLoader<Controller>;

        /** @brief Controller plugin manager
         * @details This class is used to manage the controller plugins.
         * It is used to load the plugins and to clear them.
         * @see ControllerServer::loadControllerPlugin
         * @see ControllerServer::controller_plugin_manager_
         */
        using ControllerPluginManager = navit_core::AbstractPluginManager<Controller>;

        /** @brief Constructor
         * @details This constructor is used to initialize the controller server.
         * It loads the controller plugins and initializes the action server.
         * @param tf_buffer The tf buffer used to get the robot pose.
         * @param nh The node handle used to initialize the action server.
         * @see ControllerServer::controller_plugin_manager_
         * @see ControllerServer::controller_loader_
         * @see ControllerServer::as_
         */
        ControllerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh);

        /** @brief Destructor
         * @details This destructor is used to clear the controller plugins.
         */
        ~ControllerServer(){
            controller_plugin_manager_.clearPlugins();
            controller_ptr_.reset();
            default_controller_ptr_.reset();

            costmap_.reset();
            tf_.reset();
        }

    protected:
        ros::NodeHandle nh_;


        /** @brief Load the controller plugin
         * @details This method is used to load the controller plugin.
         * It is used to load the controller plugin and to set the
         * controller_ptr_.
         * @param plugin The name of the plugin to load.
         * @return The pointer to the controller plugin.
         * @see ControllerServer::controller_ptr_
         */
        ControllerPtr loadControllerPlugin(const std::string& plugin);

        /** @brief Initialize the controller plugin
         * @details This method is used to initialize the controller plugin.
         * It is used to initialize the controller plugin and to set the
         * controller_ptr_.
         * @param plugin The name of the plugin to initialize.
         * @param controller_ptr The pointer to the controller plugin.
         * @return True if the plugin is initialized, false otherwise.
         * @see ControllerServer::controller_ptr_
         */
        bool initControllerPlugin(const std::string& plugin,
                                  const ControllerPtr& controller_ptr);

        /** @brief Action server callback
         * @details This callback is called when a new goal is received.
         * It is used to set the goal and to start the controller.
         * @param action_goal The goal received by the action server.
         * @see ControllerServer::setPathToFollow
         * @see ControllerServer::controller_ptr_
         * @see ControllerServer::as_
         */
        void followPath(const ActionGoal& action_goal);

        /** @brief Odometry callback
         * @details This callback is called when a new odometry message is received.
         * It is used to update the current pose and velocity.
         * @param odom_msg The odometry message received.
         * @see ControllerServer::current_pose_
         * @see ControllerServer::current_vel_
         */
        void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

        ControllerPtr controller_ptr_, default_controller_ptr_;
        ControllerLoader controller_loader_;
        ControllerPluginManager controller_plugin_manager_;
        ActionServer as_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;

        ros::Publisher cmd_vel_pub_;
        ros::Publisher plan_pub_;
        ros::Subscriber odom_sub_;

        geometry_msgs::PoseStamped current_pose_;
        geometry_msgs::Twist current_vel_;
        geometry_msgs::PoseStamped end_pose_;
        nav_msgs::Path current_path_;
        nav_msgs::Path original_path_;
        double distance_to_goal_, completion_percentage_;

        /** @brief getPathLength
         * @details This method is used to compute the length of a path.
         * @param path The path to compute the length.
         * @return The length of the path.
         */
        inline double getPathLength(const nav_msgs::Path& path);

        /** @brief progressChecker
         * @details This method is used to check if the robot is making progress.
         * @param feedback The feedback received by the action server.
         * @return True if the robot is making progress, false otherwise.
         */
        inline bool progressChecker(const ActionFeedback& feedback);

        /// The window size used to compute the progress
        std::deque<ActionFeedback> feedback_window_;

        ros::Time last_valid_cmd_time_, blocked_time_;

        /** @brief publishZeroVelocity
         * @details This method is used to publish a zero velocity.
         */
        void publishZeroVelocity();

        /** @brief computeAndPublishVelocity
         * @details This method is used to compute and publish the velocity.
         */
        void computeAndPublishVelocity();

        /** @brief updateGlobalPath
         * @details This method is used to update the global path. And check if the
         * action goal is preempted.
         */
        void updateGlobalPath();

        /**
         * @brief updateLocalPath
         * @details This method is used to update the local path which
         * try to avoid obstacles and follow global path .
         */
        void updateLocalPath();

        /** @brief setPathToFollow
         * @details This method is used to set the path to follow.
         * @param path The path to follow.
         * @return True if the path is set, false otherwise.
         */
        bool setPathToFollow(const nav_msgs::Path& path);

        struct config {
            double control_frequency = 10.0;   // Hz
            double control_wait_timeout = 1.0; // s
            bool enable_local_planning = false;
            double local_plan_frequency = 5.0; // Hz
            double blocked_timeout = 5.0;      // s
            double min_step_distance = 0.05;   // m
            double forward_distance = 2.0;     // m
            double padding_footprint = 0.1;    // m
            double dt = 2.0;                   // s
            int working_scene = 0;
        } config_;

        /// The service server used to clear the costmap
        ros::ServiceServer clear_costmap_srv_, toggle_avoidance_;

        /** @brief clearCostmapService
         * @details This method is used to clear the costmap.
         * @param req The request received by the service server.
         * @param res The response sent by the service server.
         * @return True if the costmap is cleared, false otherwise.
         */
        bool clearCostmapService(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Response& res)
        {
            ROS_INFO("Controller costmap is cleared");

            boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(costmap_->getCostmap()->getMutex()));
            costmap_->resetLayers();
            return true;
        }

        /** @brief getCompletionPercentage
         * @details This method is used to compute the completion percentage.
         * @param tf The tf buffer used to get the robot pose.
         * @param global_pose The robot pose.
         * @param global_plan The global plan.
         * @param completion_percentage The completion percentage.
         * @return True if the completion percentage is computed, false otherwise.
         */
        bool getCompletionPercentage(const tf2_ros::Buffer& tf,
                                     const geometry_msgs::PoseStamped& global_pose,
                                     std::vector<geometry_msgs::PoseStamped>& global_plan,
                                     double& completion_percentage);

        bool goal_canceled_;

        /**
         * @brief  控制器是否检测到碰撞堵塞不走
         * @param  cmd  控制器的速度控制
         * @param  blocking_cmd  控制器堵塞反馈的速度控制，实际上是标志作用
         * @return true
         * @return false
         */
        bool IsControllerBlocking(const double cmd, const double blocking_cmd);

        /**
         * @brief  确认控制器堵塞
         * @param  timeout 超时时间
         * @return true
         * @return false
         */
        bool IsControllerBlocked(const double timeout) const;

        bool toggleAvoidanceService(navit_msgs::ToggleAvoidance::Request& req,
                                     navit_msgs::ToggleAvoidance::Response& res);

        const double kCtrlerBlockingMarker_ = 0.01;

        // -------------------- local planner --------------------
        std::shared_ptr<local_planner::LocalPlanner> lp_;
        std::shared_ptr<std::thread> lp_thread_;
        std::mutex th_mutex_;
        std::mutex set_plan_mutex_;
        std::condition_variable lp_cond_;
        bool suspend_{true};
        bool localplan_get_plan_{false};

        void LocalPlan();
        void ThreadSuspend();
        void ThreadResume();

        // -------------------- collision checker   ----------
        bool predictCollision(const double forward_distance, const double forward_time, const double controller_frequency);
        bool collisionCheckPose(const geometry_msgs::PoseStamped& check_pose);
        ros::Publisher footprint_marker_array_pub_;
        std::shared_ptr<navit_collision_checker::FootprintCollisionChecker<navit_costmap::Costmap2D*>> collision_checker_;

        // -------------------- dynamic reconfigure ----------
        void setupDynamicReconfigure(ros::NodeHandle& nh);
        void reconfigureCB(navit_controller::NavitControllerReconfigureConfig &config, uint32_t level);
        dynamic_reconfigure::Server<navit_controller::NavitControllerReconfigureConfig> *dsrv_;

        // -------------------- working scene --------------------
        enum WorkingScene
        {
          kFollowPath = 0, // 沿线全覆盖或者手绘的沿线轨迹
          kFollowEdge, // 沿墙用于录制的地图边界
          kFollowPathWithoutAvoidance, // 无避障沿线（停障），用于一些窄道、小径等场景
        };

        void UpdateWorkingScene();
};

}
#endif
