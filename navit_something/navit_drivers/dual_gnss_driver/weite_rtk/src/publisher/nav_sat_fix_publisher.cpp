#include "integrated_navigation/publisher/nav_sat_fix_publisher.h"

namespace integrated_navigation {
    NavSatFixPublisher::NavSatFixPublisher(ros::NodeHandle& nh, std::string topic_name, std::string frame_id, int buff_size)
        :frame_id_(frame_id) {
        publisher_ = nh.advertise<sensor_msgs::NavSatFix>(topic_name, buff_size);
    }

    void NavSatFixPublisher::Publish(const State state) {
        PublishData(state);
    }

    void NavSatFixPublisher::PublishData(const State state) {
        sensor_msgs::NavSatFixPtr fix(new sensor_msgs::NavSatFix());
        fix->header.frame_id = frame_id_;
        ros::Time ros_time(state.timestamp);
        fix->header.stamp = ros_time;

        if(state.gnss_quality == 4 || state.gnss_quality == 17 || state.gnss_quality == 3 || state.gnss_quality == 16){
            fix->status.status = 2;
        }else{
            fix->status.status = 1;
        }

        if (state.filter_status == 0) {
            fix->status.status = 1;
        }
        fix->status.service = 1;

        fix->latitude = state.lla.x();
        fix->longitude = state.lla.y();
        fix->altitude = state.lla.z();

        double fused_yaw = atan2(-state.G_R_I(0,1),state.G_R_I(1,1))*kRadianToDegree;
        double fused_pitch = asin(state.G_R_I(2,1))*kRadianToDegree;
        double fused_roll = atan2(-state.G_R_I(2,0),state.G_R_I(2,2))*kRadianToDegree;
        if(fused_yaw > 0.){
            fused_yaw -= 360.;
        }
        fused_yaw = -fused_yaw;
        fix->position_covariance[0] = fused_yaw;
        fix->position_covariance[3] = fused_pitch;
        fix->position_covariance[4] = fused_roll;
        fix->position_covariance[5] = state.velocity.x();
        fix->position_covariance[6] = state.velocity.y();
        fix->position_covariance[7] = state.velocity.z();

        publisher_.publish(fix);
    }
}