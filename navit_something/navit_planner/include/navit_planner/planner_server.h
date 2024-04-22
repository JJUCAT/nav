#ifndef PLANNER_SERVER_H
#define PLANNER_SERVER_H

#include <navit_core/base_global_planner.h>
#include <navit_core/abstract_plugin_manager.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <navit_msgs/ComputePathAction.h>
#include <navit_msgs/PlanSecurityCheckingAction.h>
#include <nav_msgs/Path.h>

#include <pluginlib/class_loader.hpp>
#include <actionlib/server/simple_action_server.h>

#include <std_srvs/Empty.h>

// for collision checker service
#include <navit_collision_checker/footprint_collision_checker.h>
#include <navit_costmap/cost_values.h>
#include <navit_msgs/IsPoseInCollision.h>
#include <tf2/utils.h>

// for stats
#include <navit_planner/planner_stats.h>

namespace navit_planner {
    using PlannerPtr = navit_core::GlobalPlanner::Ptr;
    using ActionType = navit_msgs::ComputePathAction;
    using ActionGoal = navit_msgs::ComputePathGoalConstPtr;
    using ActionResult = navit_msgs::ComputePathResult;
    using ActionServer = actionlib::SimpleActionServer<ActionType>;

    using PSC_ActionType = navit_msgs::PlanSecurityCheckingAction;
    using PSC_ActionGoal = navit_msgs::PlanSecurityCheckingGoalConstPtr;
    using PSC_ActionResult = navit_msgs::PlanSecurityCheckingResult;
    using PSC_ActionServer = actionlib::SimpleActionServer<PSC_ActionType>;

    class PlannerServer
    {
        public:
            using Planner = navit_core::GlobalPlanner;
            using PlannerLoader = pluginlib::ClassLoader<Planner>;
            using PlannerPluginManager = navit_core::AbstractPluginManager<Planner>;

            PlannerServer(std::shared_ptr<tf2_ros::Buffer>& tf_buffer, ros::NodeHandle& nh);

            ~PlannerServer()
            {
                planner_plugin_manager_.clearPlugins();
                planner_.reset();

                collision_checker_.reset();
                costmap_.reset();
                tf_buffer_.reset();
            }

            nav_msgs::Path getPlan(const geometry_msgs::PoseStamped& start,
                                   const geometry_msgs::PoseStamped& goal,
                                   const std::string& plugin_name);

        protected:
            PlannerStats planner_stats_;
            ros::NodeHandle nh_;

            void computePath(const ActionGoal& goal);

            void PlanSecurityChecking(const PSC_ActionGoal& action_goal);

            void publishPath();

            PlannerPtr planner_, default_planner_;
            PlannerPluginManager planner_plugin_manager_;
            PlannerLoader planner_loader_;

            PlannerPtr loadPlannerPlugins(const std::string& plugin);
            bool initPlannerPlugins(const std::string& plugin,
                                    const PlannerPtr& planner_ptr);

            std::shared_ptr<ActionServer> as_;
            std::shared_ptr<PSC_ActionServer> psc_as_; // plan_security_checking

            std::shared_ptr<navit_costmap::Costmap2DROS> costmap_;
            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

            nav_msgs::Path planned_path_;

            ros::Publisher plan_pub_;

            bool transformPosesToGlobalFrame(geometry_msgs::PoseStamped& start,
                                             geometry_msgs::PoseStamped& goal);

            void waitForCostmap();

            bool getStartPose(const ActionGoal& action_goal,
                              geometry_msgs::PoseStamped& start);

            ros::ServiceServer clear_costmaps_srv_;    

            bool clearCostmapService(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
            {
                ROS_INFO("Planner Costmap is cleared!");
                boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(costmap_->getCostmap()->getMutex()));
                costmap_->resetLayers();
                return true;
            }

            // collision checker
            std::shared_ptr<navit_collision_checker::FootprintCollisionChecker<navit_costmap::Costmap2D*> > collision_checker_;
            ros::ServiceServer collision_checker_srv_;    

            bool collisionCheck(const geometry_msgs::PoseStamped& check_pose,
              const double collised_cost = navit_costmap::LETHAL_OBSTACLE)
            {
                if (check_pose.header.frame_id != costmap_->getGlobalFrameID())
                {
                    ROS_WARN("please use pose in %s frame", (costmap_->getGlobalFrameID()).c_str());
                    return true;
                }
                double yaw = tf2::getYaw(check_pose.pose.orientation);
                double x = check_pose.pose.position.x;
                double y = check_pose.pose.position.y;

                double checked_cost = collision_checker_->footprintCostAtPose(x,y,yaw,costmap_->getRobotFootprint());
                if (checked_cost >= collised_cost)
                {
                    ROS_INFO("pose in collision detected");
                    return true;
                }
               return false;
            }

            bool collisionCheckService(navit_msgs::IsPoseInCollision::Request& req,
                                       navit_msgs::IsPoseInCollision::Response& res)
            {
                res.in_collision = false;
                if (collisionCheck(req.pose))
                {
                    ROS_INFO("pose in collision detected");
                    res.in_collision = true;
                }

               return true;
            }
    };

}
#endif 
