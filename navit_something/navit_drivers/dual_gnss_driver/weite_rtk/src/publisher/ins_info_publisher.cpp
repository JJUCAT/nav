#include "integrated_navigation/publisher/ins_info_publisher.h"

namespace integrated_navigation {
    InsInfoPublisher::InsInfoPublisher(ros::NodeHandle &nh, std::string topic_name,
                                                           std::string frame_id, int buff_size)
        : frame_id_(frame_id) {
        publisher_ = nh.advertise<navit_msgs::InsInfo>(topic_name, buff_size);
    }

    void InsInfoPublisher::Publish(const State state) { PublishData(state); }

    void InsInfoPublisher::PublishData(const State state) {
        navit_msgs::InsInfoPtr pose_vel(new navit_msgs::InsInfo());
        pose_vel->header.frame_id = frame_id_;
        ros::Time ros_time(state.timestamp);
        pose_vel->header.stamp = ros_time;

        // gps time.
        pose_vel->gps_week_num = state.gps_week_num;
        pose_vel->gps_seconds = state.gps_seconds;

        // position, lla.
        pose_vel->latitude = state.lla.x();
        pose_vel->longitude = state.lla.y();
        pose_vel->altitude = state.lla.z();
        for (int i = 0; i < 3; ++i) {     //行
            for (int j = 0; j < 3; ++j) { //列
                pose_vel->position_covariance[i * 3 + j] = state.cov(i, j);
            }
        }

        // velocity, enu.
        pose_vel->velocity_x = state.velocity.x();
        pose_vel->velocity_y = state.velocity.y();
        pose_vel->velocity_z = state.velocity.z();
        auto &velocity_cov = state.cov.block<3, 3>(3, 3);
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                pose_vel->velocity_covariance[i * 3 + j] = velocity_cov(i, j);
            }
        }

        // euler.
        double fused_yaw = atan2(-state.G_R_I(0, 1), state.G_R_I(1, 1)) * kRadianToDegree;
        double fused_pitch = asin(state.G_R_I(2, 1)) * kRadianToDegree;
        double fused_roll = atan2(-state.G_R_I(2, 0), state.G_R_I(2, 2)) * kRadianToDegree;
        if (fused_yaw > 0.) {
            fused_yaw -= 360.;
        }
        fused_yaw = -fused_yaw;
        pose_vel->heading = fused_yaw;
        pose_vel->pitch = fused_pitch;
        pose_vel->roll = fused_roll;

        // geometry_msgs/Quaternion quaternion
        Eigen::Quaterniond fused_quat(state.G_R_I);
        fused_quat.normalize();
        pose_vel->quaternion.w = fused_quat.w();
        pose_vel->quaternion.x = fused_quat.x();
        pose_vel->quaternion.y = fused_quat.y();
        pose_vel->quaternion.z = fused_quat.z();

        // imu sensor data.
        if (state.imu_data_ptr != NULL) {
            pose_vel->gyro_x = state.imu_data_ptr->gyro.x();
            pose_vel->gyro_y = state.imu_data_ptr->gyro.y();
            pose_vel->gyro_z = state.imu_data_ptr->gyro.z();
            pose_vel->acc_x = state.imu_data_ptr->acc.x();
            pose_vel->acc_y = state.imu_data_ptr->acc.y();
            pose_vel->acc_z = state.imu_data_ptr->acc.z();
        }

        // rover status.
        pose_vel->status = state.gnss_quality * 10 + state.filter_status;
        pose_vel->satellites1 = state.num_satellites1;
        pose_vel->satellites2 = state.num_satellites2;
        pose_vel->age = state.age;

        publisher_.publish(pose_vel);
    }
} // namespace integrated_navigation