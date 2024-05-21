

#include <recovery_spin/spin.h>

using namespace navit_recovery;

namespace recovery_spin {
    void RecoverySpin::loadParams(const std::string& name)
    {
        ros::NodeHandle pnh("~/" + name);
        pnh.param("Kp", config_.Kp, config_.Kp);
        pnh.param("abs_max_speed", config_.abs_max_speed, config_.abs_max_speed);
        pnh.param("control_frequency", config_.control_frequency, config_.control_frequency);
        pnh.param("cmd_vel_topic", config_.cmd_vel_topic, config_.cmd_vel_topic);

        ROS_INFO("Recovery Action BackUp params loaded!!");
    }

    Status RecoverySpin::onRun(const ActionGoalT::ConstPtr& action_goal)
    {
        ros::NodeHandle nh;
        cmd_pub_ = nh.advertise<geometry_msgs::Twist>(config_.cmd_vel_topic, 1);
        
        estimated_time_ = action_goal->target_yaw / action_goal->speed;
        end_ = ros::Time::now() + ros::Duration(estimated_time_);

        spin_distance_ = action_goal->target_yaw;

        if (action_goal->speed >= config_.abs_max_speed)
            cmd_vel_.linear.z = config_.abs_max_speed; //backup speed
        else  if(action_goal->speed <= -config_.abs_max_speed)
        {
            cmd_vel_.linear.z = -config_.abs_max_speed;
        } 
            else{
                 cmd_vel_.linear.z = action_goal->speed;
            }
        return navit_recovery::Status::SUCCEEDED;
    }

    Status RecoverySpin::onCycleUpdate()
    {
        ros::Time current_time = ros::Time::now();
        ros::Duration time_left = end_ - current_time; 

        // ugly
        auto angular_traveled = time_left.toSec() / estimated_time_ * spin_distance_;
        feedback_.angular_traveled = angular_traveled;
        as_->publishFeedback(feedback_);

        if (time_left.toSec() > 0)
        {
            geometry_msgs::PoseStamped pose;
            if (!map_ros_->getRobotPose(pose)) {
              ROS_ERROR("[Rcvr][Backup] get robot pose from map failed !");
              return navit_recovery::Status::FAILED;
            }

            // collision check
            if (collision_checker_->isCollisionFree(pose, cmd_vel_, time_left.toSec()))
            {
                cmd_pub_.publish(cmd_vel_);
                return navit_recovery::Status::RUNNING;
            }
            else
            {
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
PLUGINLIB_EXPORT_CLASS(recovery_spin::RecoverySpin, navit_core::Recovery)
