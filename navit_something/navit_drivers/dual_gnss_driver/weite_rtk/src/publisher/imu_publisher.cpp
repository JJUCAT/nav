#include "integrated_navigation/publisher/imu_publisher.h"

namespace integrated_navigation {
    ImuPublisher::ImuPublisher(ros::NodeHandle& nh, std::string topic_name, int buff_size) {
        publisher_ = nh.advertise<sensor_msgs::Imu>(topic_name, buff_size);
    }

    void ImuPublisher::Publish(const State state) {
        PublishData(state);
    }

    void ImuPublisher::PublishData(const State state) {
        // set timestamps
        sensor_msgs::ImuPtr imu(new sensor_msgs::Imu());

        imu->header.frame_id = "imu_link";
        ros::Time ros_time(state.timestamp);
        imu->header.stamp = ros_time;

        // set orientation:
        Eigen::Quaterniond q(state.G_R_I);
        imu->orientation.w = q.w();
        imu->orientation.x = q.x();
        imu->orientation.y = q.y();
        imu->orientation.z = q.z();
        for (int i = 0; i < 3; ++i) {     //行
            for (int j = 0; j < 3; ++j) { //列
                imu->orientation_covariance[i * 3 + j] = state.cov(i + 6, j + 6);
            }
        }

        // set angular velocity:
        geometry_msgs::Vector3 angular_velocity;
        geometry_msgs::Vector3 linear_acceleration;
        if(state.imu_data_ptr != NULL){
            angular_velocity.x = state.imu_data_ptr->gyro.x();
            angular_velocity.y = state.imu_data_ptr->gyro.y();
            angular_velocity.z = state.imu_data_ptr->gyro.z();

            // set linear acceleration:
            linear_acceleration.x = state.imu_data_ptr->acc.x();
            linear_acceleration.y = state.imu_data_ptr->acc.y();
            linear_acceleration.z = state.imu_data_ptr->acc.z();
        }
        imu->angular_velocity = angular_velocity;
        imu->linear_acceleration = linear_acceleration;

        publisher_.publish(imu);
    }
}