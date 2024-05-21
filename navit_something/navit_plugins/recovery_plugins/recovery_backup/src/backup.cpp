#include <recovery_backup/backup.h>
#include "geometry_msgs/PoseStamped.h"

using namespace navit_recovery;

namespace recovery_backup {
    void RecoveryBackUp::loadParams(const std::string& name)
    {
        ros::NodeHandle pnh("~/" + name);
        pnh.param("Kp", config_.Kp, config_.Kp);
        pnh.param("abs_max_speed", config_.abs_max_speed, config_.abs_max_speed);
        pnh.param("control_frequency", config_.control_frequency, config_.control_frequency);
        pnh.param("cmd_vel_topic", config_.cmd_vel_topic, config_.cmd_vel_topic);

        ROS_INFO("Recovery Action BackUp params loaded!!");
    }

    Status RecoveryBackUp::onRun(const ActionGoalT::ConstPtr& action_goal)
    {
        ROS_INFO("[Rcvr][Backup] receive backup, distance %f, speed %f",
          action_goal->backup_distance, action_goal->speed);
        ros::NodeHandle nh;
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>(config_.cmd_vel_topic, 1);
        
        estimated_time_ = action_goal->backup_distance / action_goal->speed;
        end_ = ros::Time::now() + ros::Duration(estimated_time_);
        ROS_INFO("[Rcvr][Backup] estimated time %f", estimated_time_);

        backup_distance_ = action_goal->backup_distance;

        if (action_goal->speed >= config_.abs_max_speed)
            cmd_vel_.linear.x = -config_.abs_max_speed; //backup speed
        else
            cmd_vel_.linear.x = -action_goal->speed;
        return navit_recovery::Status::SUCCEEDED;
    }

    Status RecoveryBackUp::onCycleUpdate()
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration time_left = end_ - current_time; 

        // ugly
        auto distance_traveled = time_left.toSec() / estimated_time_ * backup_distance_;
        feedback_.distance_traveled = distance_traveled;
        as_->publishFeedback(feedback_);

        if (time_left.toSec() > 0)
        {
            // collision check
            geometry_msgs::PoseStamped pose;
            if (!map_ros_->getRobotPose(pose)) {
              ROS_ERROR("[Rcvr][Backup] get robot pose from map failed !");
              return navit_recovery::Status::FAILED;
            }

            if (collision_checker_->isCollisionFree(pose, cmd_vel_, time_left.toSec()))
            {
                cmd_pub_.publish(cmd_vel_);
                return navit_recovery::Status::RUNNING;
            }
            else
            {
                ROS_WARN("[Rcvr][Backup] check collised.");
                // in collision, we should stop
                cmd_pub_.publish(geometry_msgs::Twist());
                return navit_recovery::Status::FAILED;
            }
        }
        else
        {
            cmd_pub_.publish(geometry_msgs::Twist());
            return navit_recovery::Status::SUCCEEDED;
        }
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(recovery_backup::RecoveryBackUp, navit_core::Recovery)
