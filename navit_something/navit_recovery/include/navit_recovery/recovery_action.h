#ifndef NAVIT_RECOVERY__RECOVERY_H
#define NAVIT_RECOVERY__RECOVERY_H

#include <ros/ros.h>
#include <navit_core/base_recovery.h>
#include <actionlib/server/simple_action_server.h>
#include <tf2_ros/buffer.h>
#include <navit_collision_checker/collision_checker.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <geometry_msgs/Twist.h>
#include "navit_costmap/cost_values.h"
#include "ros/init.h"

namespace navit_recovery {

    enum class Status : int8_t
    {
        SUCCEEDED = 1,
        FAILED = 2,
        RUNNING = 3,
    };

    template<typename ActionT, typename ActionGoal, typename ActionResult>
    class RecoveryAction : public navit_core::Recovery
    {
        public:
            using ActionServer = actionlib::SimpleActionServer<ActionT>;

            RecoveryAction(): 
                as_(nullptr), 
                enabled_(false), 
                control_frequency_(10.0),
                global_frame_("odom"),
                robot_base_frame_("base_link"),
                transform_tolerance_(0.1)
                {}

            virtual ~RecoveryAction(){}

            virtual Status onRun (const typename ActionGoal::ConstPtr& cmd) = 0;

            virtual Status onCycleUpdate() = 0;

            virtual Status onCancel() { return navit_recovery::Status::SUCCEEDED; }

            void initialize(
                    const std::string& name,
                    const std::shared_ptr<tf2_ros::Buffer>& tf,
                    const std::shared_ptr<navit_collision_checker::CollisionChecker>& collision_checker)
            {
                name_ = name;
                ros::NodeHandle nh, pnh("~/" + name_); 

                pnh.param("control_frequency", control_frequency_, control_frequency_);
                pnh.param("global_frame",global_frame_, global_frame_ );
                pnh.param("robot_base_frame", robot_base_frame_, robot_base_frame_);
                pnh.param("transform_tolerance", transform_tolerance_, transform_tolerance_);

                tf_ = tf;

                as_ = std::make_shared<ActionServer>(nh, name_, boost::bind(&RecoveryAction::execute, this,_1),false);

                collision_checker_ = collision_checker;

                std::string cmd_vel_topic_name = "cmd_vel";
                pnh.param("cmd_vel_topic_name", cmd_vel_topic_name, cmd_vel_topic_name);
                vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1);

                loadParams(name_);

                as_->start();
                enabled_ = true;
                ROS_INFO("Recovery plugin %s is initialized!", name_.c_str());
            }

            virtual void loadParams(const std::string& name) = 0;

            void SetExtraCostmap(const std::shared_ptr<navit_costmap::Costmap2DROS>& map) override
            {
              use_extra_costmap_ = true;
              map_ros_ = map;
            }

        protected:
            ros::NodeHandle nh_;
            std::string name_;
            std::shared_ptr<tf2_ros::Buffer> tf_;
            std::shared_ptr<ActionServer> as_;
            std::shared_ptr<navit_collision_checker::CollisionChecker> collision_checker_;

            ros::Publisher vel_pub_;

            bool enabled_;
            double control_frequency_;
            std::string global_frame_;
            std::string robot_base_frame_;
            double transform_tolerance_;

            bool use_extra_costmap_;
            std::shared_ptr<navit_costmap::Costmap2DROS> map_ros_;

            void execute(const typename ActionGoal::ConstPtr& action_goal)
            {
                ROS_INFO("Attempting %s", name_.c_str());

                if (!enabled_)
                {
                    ROS_WARN("Recovery action not initialized!");
                    return;
                }

                if (onRun(action_goal) != Status::SUCCEEDED)
                {
                    ROS_WARN("Action goal check failed!");
                    return;
                }

                ros::Time start = ros::Time::now();

                ros::Rate r(control_frequency_);

                ActionResult result;

                if (use_extra_costmap_) SetupExtraCostmap();

                while (nh_.ok())
                {
                    if(as_->isPreemptRequested())
                    {
                        // stop
                        onCancel();
                        ROS_WARN("Cancel recovery action %s", name_.c_str());
                        stopRobot();
                        as_->setPreempted(result, "preemtped");
                        if (use_extra_costmap_) ShutDownExtraCostmap();
                        return;
                    }

                    switch (onCycleUpdate())
                    {
                        case Status::SUCCEEDED:
                            ROS_INFO("%s completed successful!", name_.c_str());
                            result.total_time.data = ros::Time::now() - start;
                            as_->setSucceeded(result,"recovery completed!");
                            if (use_extra_costmap_) ShutDownExtraCostmap();
                            return;

                        case Status::FAILED:
                            ROS_WARN("%s failed!", name_.c_str());
                            result.total_time.data = ros::Time::now() - start;
                            as_->setAborted(result,"recovery failed!");
                            if (use_extra_costmap_) ShutDownExtraCostmap();
                            return;

                        case Status::RUNNING:

                        default:
                            r.sleep();
                            break;
                    }
                }
            
            }

            void stopRobot()
            {
                geometry_msgs::Twist stop_vel;
                stop_vel.linear.x = 0.0;
                stop_vel.linear.y = 0.0;
                stop_vel.angular.z = 0.0;
                vel_pub_.publish(stop_vel);
            }

            void SetupExtraCostmap()
            {
              ROS_INFO("[RCVR] setup extra costmap.");
              map_ros_->resume();
              {
                ROS_INFO("[RCVR] clear extra costmap.");
                boost::unique_lock<navit_costmap::Costmap2D::mutex_t> lock(*(map_ros_->getCostmap()->getMutex()));
                map_ros_->resetLayers();                
              }
              ros::Rate r(100);
              while (!map_ros_->isCurrent()) r.sleep();
              collision_checker_->SetExtraCostmap(map_ros_,
                map_ros_->getRobotFootprint(), navit_costmap::LETHAL_OBSTACLE);
            }

            void ShutDownExtraCostmap()
            {
              ROS_INFO("[RCVR] shutdown extra costmap.");
              map_ros_->pause();
            }
    };



}
#endif
