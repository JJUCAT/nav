#ifndef CCPP_CONTROLLER_SERVER_H
#define CCPP_CONTROLLER_SERVER_H

#include <ros/ros.h>

#include <navit_core/base_controller.h>
#include <navit_core/abstract_plugin_manager.h>

#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>

#include <navit_costmap/costmap_2d_ros.h>
#include <navit_costmap/cleaned_layer.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <navit_msgs/FollowCoveragePathAction.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <actionlib/server/simple_action_server.h>
#include <pluginlib/class_loader.hpp>

#include <std_srvs/Empty.h>

#include <navit_controller/controller_stats.h>
#include <tf2/utils.h>

#include <navit_controller/controller_server_helper.h>
namespace navit_controller {

using Controller = navit_core::Controller;
using ControllerPtr = navit_core::Controller::Ptr;
using FcpActionType = navit_msgs::FollowCoveragePathAction;
using FcpActionGoal = navit_msgs::FollowCoveragePathGoalConstPtr;
using FcpActionResult = navit_msgs::FollowCoveragePathResult;
using FcpActionFeedback = navit_msgs::FollowCoveragePathFeedback;
using FcpActionServer = actionlib::SimpleActionServer<FcpActionType>;

class CCPPControllerServer
{
    public:
        using ControllerLoader = pluginlib::ClassLoader<Controller>;

        using ControllerPluginManager = navit_core::AbstractPluginManager<Controller>;

        CCPPControllerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh);

        ~CCPPControllerServer(){
            controller_plugin_manager_.clearPlugins();
            controller_ptr_.reset();
            default_controller_ptr_.reset();

            costmap_.reset();
            tf_.reset();
        }

    protected:
        ros::NodeHandle nh_;

        ControllerPtr loadControllerPlugin(const std::string& plugin);

        bool initControllerPlugin(const std::string& plugin,
                                  const ControllerPtr& controller_ptr);

        void followCoveragePath(const FcpActionGoal& action_goal); 

        void odomCallback(const nav_msgs::OdometryConstPtr& odom_msg);

        ControllerPtr controller_ptr_, default_controller_ptr_;
        ControllerLoader controller_loader_;
        ControllerPluginManager controller_plugin_manager_;
        FcpActionServer as_;
        std::shared_ptr<tf2_ros::Buffer> tf_;
        std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;
        std::shared_ptr<navit_costmap::Costmap2DROS> swept_costmap_;
    
        ros::Publisher cmd_vel_pub_, cleaned_pub_;
        ros::Publisher plan_pub_, local_path_pub_, local_origon_path_pub_;
        ros::Subscriber odom_sub_;

        geometry_msgs::PoseStamped current_pose_;
        geometry_msgs::Twist current_vel_;
        nav_msgs::Path current_path_;
        nav_msgs::Path original_path_;
        double distance_to_goal_, completion_percentage_;

        inline double getPathLength(const nav_msgs::Path& path);

        inline bool progressChecker(const FcpActionFeedback& feedback);

        std::deque<FcpActionFeedback> feedback_window_;
        
        ros::Time last_valid_cmd_time_;

        void publishZeroVelocity();

        void computeAndPublishVelocity();

        
        void updateGlobalPath();

        bool setPathToFollow(const nav_msgs::Path& path);

        struct config {
            double control_frequency = 10.0;
            double control_wait_timeout = 1.0;
            double extract_length = 5.0;
            std::string global_frame = "odom";
            std::string robot_frame = "base_link";
            std::string cmd_vel_topic_name = "/controller_cmd_vel";
            std::string clear_costmap_srv_name = "clear_coverage_controller_costmap";
        } config_;

        ros::ServiceServer clear_costmap_srv_;

        bool clearCostmapService(std_srvs::Empty::Request& req,
                                 std_srvs::Empty::Response& res)
        {
            ROS_INFO("Controller costmap is cleared");

            boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(costmap_->getCostmap()->getMutex()));
            costmap_->resetLayers();
            return true;
        }

        bool getCompletionPercentage(const tf2_ros::Buffer& tf, 
                                     const geometry_msgs::PoseStamped& global_pose, 
                                     std::vector<geometry_msgs::PoseStamped>& global_plan,
                                     double& completion_percentage);
        void tfHandleThread();
        void reset() {
            extract_index_ = 0;
            start_index_ = 0;
            current_path_.poses.clear();
            original_path_.poses.clear();
        }
        bool goal_canceled_;

        std::shared_ptr<navit_costmap::CleanedLayer> cleaned_layer_;
        std::shared_ptr<navit_controller::ControllerServerHelper> controller_server_helper_ptr_;
        boost::thread* tf_handle_thread_ptr_;
        geometry_msgs::TransformStamped transform_stamped_;
        geometry_msgs::TransformStamped robot_pose_;
        std::string action_plan_frame_id_ = "map";
        int extract_index_ = 0;
        int start_index_ = 0;
};

}
#endif
