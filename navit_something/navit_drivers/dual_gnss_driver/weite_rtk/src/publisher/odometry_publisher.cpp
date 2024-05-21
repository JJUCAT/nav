#include "integrated_navigation/publisher/odometry_publisher.h"
#include "integrated_navigation/utils/utils.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/LinearMath/Quaternion.h"

namespace integrated_navigation {
    OdometryPublisher::OdometryPublisher(ros::NodeHandle& nh,
                                         std::string topic_name,
                                         std::string base_frame_id,
                                         std::string child_frame_id,
                                         int buff_size)
             : base_frame_id_(base_frame_id),
               child_frame_id_(child_frame_id){
        publisher_ = nh.advertise<nav_msgs::Odometry>(topic_name, buff_size);
    }

    void OdometryPublisher::Publish(const State state) {
        PublishData(state);
    }

    void OdometryPublisher::Publish(State state, const double time) {
        PublishData(state, time);
    }

    void OdometryPublisher::PublishData(const State state) {
        nav_msgs::OdometryPtr odometry(new nav_msgs::Odometry());
        ros::Time ros_time(state.timestamp);
        odometry->header.stamp = ros_time;
        odometry->header.frame_id = base_frame_id_;
        odometry->child_frame_id = child_frame_id_;

        // set the pose
        odometry->pose.pose.position.x = state.position.x();
        odometry->pose.pose.position.y = state.position.y();
        odometry->pose.pose.position.z = state.position.z();

        Eigen::Quaterniond q(state.G_R_I);
        odometry->pose.pose.orientation.x = q.x();
        odometry->pose.pose.orientation.y = q.y();
        odometry->pose.pose.orientation.z = q.z();
        odometry->pose.pose.orientation.w = q.w();

        double roll, pitch, yaw;
        // euler.
        double fused_yaw = atan2(-state.G_R_I(0, 1), state.G_R_I(1, 1)) * kRadianToDegree;
        double fused_pitch = asin(state.G_R_I(2, 1)) * kRadianToDegree;
        double fused_roll = atan2(-state.G_R_I(2, 0), state.G_R_I(2, 2)) * kRadianToDegree;
        if (fused_yaw > 0.) {
            fused_yaw -= 360.;
        }
        fused_yaw = -fused_yaw;
        yaw = fused_yaw;
        pitch = fused_pitch;
        roll = fused_roll;


        // set the twist:
        odometry->twist.twist.linear.x = state.velocity.x();
        odometry->twist.twist.linear.y = state.velocity.y();
        odometry->twist.twist.linear.z = state.velocity.z();

        if(state.imu_data_ptr != NULL){
            odometry->twist.twist.angular.x = state.imu_data_ptr->gyro.x();
            odometry->twist.twist.angular.y = state.imu_data_ptr->gyro.y();
            odometry->twist.twist.angular.z = state.imu_data_ptr->gyro.z();
        }
        odometry->pose.covariance[22] = state.steady_status;
        odometry->pose.covariance[23] = state.lock_heading;

        odometry->pose.covariance[25] = yaw ; 
        odometry->pose.covariance[26] = pitch;
        odometry->pose.covariance[27] = roll;

        publisher_.publish(odometry);
    }

    void OdometryPublisher::PublishData(const State state, const double time) {
        nav_msgs::OdometryPtr odom(new nav_msgs::Odometry());
        ros::Time ros_time(time);
        odom->header.stamp = ros_time;
        odom->header.frame_id = base_frame_id_;
        odom->child_frame_id = child_frame_id_;

        odom->pose.covariance[0] = state.cov(0, 0);
        odom->pose.covariance[1] = state.cov(1, 1);
        odom->pose.covariance[2] = state.cov(2, 2);
        odom->pose.covariance[3] = state.cov(3, 3);
        odom->pose.covariance[4] = state.cov(4, 4);
        odom->pose.covariance[5] = state.cov(5, 5);
        odom->pose.covariance[6] = state.cov(6, 6);
        odom->pose.covariance[7] = state.cov(7, 7);
        odom->pose.covariance[8] = state.cov(8, 8);
        odom->pose.covariance[9] = state.cov(9, 9);
        odom->pose.covariance[10] = state.cov(10, 10);
        odom->pose.covariance[11] = state.cov(11, 11);
        odom->pose.covariance[12] = state.cov(12, 12);
        odom->pose.covariance[13] = state.cov(13, 13);
        odom->pose.covariance[14] = state.cov(14, 14);
        odom->pose.covariance[15] = state.acc_bias.x();
        odom->pose.covariance[16] = state.acc_bias.y();
        odom->pose.covariance[17] = state.acc_bias.z();
        odom->pose.covariance[18] = state.gyro_bias.x();
        odom->pose.covariance[19] = state.gyro_bias.y();
        odom->pose.covariance[20] = state.gyro_bias.z();
        odom->pose.covariance[21] = state.steady_state;
        odom->pose.covariance[22] = state.steady_status;

        publisher_.publish(odom);
    }
}