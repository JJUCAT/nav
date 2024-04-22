#include "average_filter.h"

namespace navit_auto_dock {
namespace plugins {
    void AverageFilter::initialize(const std::string& name,
                                   const std::shared_ptr<tf2_ros::Buffer>& tf)
    {
        ros::NodeHandle pnh("~/" + name);
        pnh.param("window_length", config_.window_lenght, config_.window_lenght);
    }

    void AverageFilter::reset()
    {
       filter_.clear(); 
    }

    void AverageFilter::update(geometry_msgs::PoseStamped& dock_pose,
                               const geometry_msgs::Twist& current_vel)
    {
	geometry_msgs::PoseStamped pose = dock_pose;
        ROS_DEBUG("the goal pose before filter : x %f,y%f,yew %f", pose.pose.position.x,
			pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
	if (filter_.size() < config_.window_lenght) {
		filter_.push_back(pose);
	} else {
		filter_.pop_front();
		filter_.push_back(pose);
	}
	geometry_msgs::PoseStamped temp;
	double yaw = 0;
	std::list<geometry_msgs::PoseStamped>::iterator it;
	for (it = filter_.begin(); it != filter_.end(); ++it) {
		temp.pose.position.x = temp.pose.position.x + it->pose.position.x;
		temp.pose.position.y = temp.pose.position.y + it->pose.position.y;
		yaw = yaw + tf2::getYaw(it->pose.orientation);
	}
	pose.pose.position.x = temp.pose.position.x / (filter_.size());
	pose.pose.position.y = temp.pose.position.y / (filter_.size());
	yaw = angles::normalize_angle(yaw / filter_.size());
	tf2::Quaternion orientation;
	orientation.setRPY(0.0, 0.0, yaw);
	pose.pose.orientation = tf2::toMsg(orientation);
	ROS_DEBUG("the goal pose after filter: x %f,y%f,yew %f", pose.pose.position.x,
			pose.pose.position.y, tf2::getYaw(pose.pose.orientation));
        dock_pose = pose;
    }
}
}
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navit_auto_dock::plugins::AverageFilter, navit_auto_dock::plugins::ApproachDockFilter)

