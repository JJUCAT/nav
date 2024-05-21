#include "integrated_navigation/publisher/gpchc_publisher.h"
#include "std_msgs/Bool.h"

namespace integrated_navigation {
    GpchcPublisher::GpchcPublisher(ros::NodeHandle& nh, std::string topic_name, std::string health_topic_name, std::string frame_id)
        :frame_id_(frame_id) {
        publisher_ = nh.advertise<sensor_msgs::NavSatFix>(topic_name, 10);
        health_publisher_ = nh.advertise<std_msgs::Bool>(health_topic_name,1);
    }

    void GpchcPublisher::Publish(const navit_msgs::GpchcPtr gpchc_msg) {
        PublishData(gpchc_msg);
    }

    void GpchcPublisher::PublishData(const navit_msgs::GpchcPtr gpchc_msg) {
        sensor_msgs::NavSatFixPtr fix(new sensor_msgs::NavSatFix());

        fix->header.stamp = gpchc_msg->header.stamp;
        fix->header.frame_id = gpchc_msg->header.frame_id;

        int dataready1, dataready2;
        dataready1 = gpchc_msg->status / 10;
        dataready2 = gpchc_msg->status % 10;
        
        if(dataready1 == 4 && dataready2 == 2){
            fix->status.status = 2;//信号强
        }
        else if(dataready1 != 4 && dataready2 != 2){
            fix->status.status = 0;//信号无
        }
        else{
            fix->status.status = 1;//信号弱
        }

        fix->status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
        fix->latitude = gpchc_msg->lattitude;
        fix->longitude = gpchc_msg->longitude;
        fix->altitude = gpchc_msg->altitude;
        fix->position_covariance[0] = gpchc_msg->heading;
        fix->position_covariance[3] = gpchc_msg->pitch;//Pitch angle
        fix->position_covariance[4] = gpchc_msg->roll;//Rolling angle
        fix->position_covariance[5] = gpchc_msg->vel_e;//east speed
        fix->position_covariance[6] = gpchc_msg->vel_n;//north speed
        fix->position_covariance[7] = gpchc_msg->vel_u;//up speed
        fix->position_covariance_type = 1;

        publisher_.publish(fix);
    }

    void GpchcPublisher::PublishHealth(const navit_msgs::GpchcPtr gpchc_msg) {
        PublishHealthData(gpchc_msg);
    }

    void GpchcPublisher::PublishHealthData(const navit_msgs::GpchcPtr gpchc_msg) {
        std_msgs::BoolPtr fix(new std_msgs::Bool());

        int dataready1, dataready2;
        dataready1 = gpchc_msg->status / 10;
        dataready2 = gpchc_msg->status % 10;
        if(dataready1 == 4 && dataready2 == 2){
            fix->data = true;
        }else{
            fix->data = false;
        }

        health_publisher_.publish(fix);
    }
}