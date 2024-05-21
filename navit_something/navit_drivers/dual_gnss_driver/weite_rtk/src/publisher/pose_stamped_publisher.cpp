#include "integrated_navigation/publisher/pose_stamped_publisher.h"

namespace integrated_navigation {
    PoseStampedPublisher::PoseStampedPublisher(ros::NodeHandle& nh,
                                             std::string topic_name,
                                             std::string frame_id,
                                             int buff_size)
            : frame_id_(frame_id) {
        publisher_ = nh.advertise<geometry_msgs::PoseStamped>(topic_name, buff_size);
    }

    void PoseStampedPublisher::Publish(const State state) {
        PublishData(state);
    }

    void PoseStampedPublisher::PublishData(const State state) {
        geometry_msgs::PoseStampedPtr pose(new geometry_msgs::PoseStamped());
        ros::Time ros_time(state.timestamp);
        pose->header.stamp = ros_time;
        pose->header.frame_id = frame_id_;

        // set the position.
        pose->pose.position.x = state.position.x();
        pose->pose.position.y = state.position.y();
        pose->pose.position.z = 0.5;

        // set the quaternion.
        Eigen::Quaterniond q(state.G_R_I);
        pose->pose.orientation.x = q.x();
        pose->pose.orientation.y = q.y();
        pose->pose.orientation.z = q.z();
        pose->pose.orientation.w = q.w();

        publisher_.publish(pose);
    }
}