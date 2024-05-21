#include "integrated_navigation/publisher/system_control_publisher.h"

namespace integrated_navigation {
    SystemControlPublisher::SystemControlPublisher(ros::NodeHandle &nh, std::string topic_name, std::string frame_id,
                                                   int buff_size)
        : frame_id_(frame_id) {
        publisher_ = nh.advertise<navit_msgs::SystemErrorStatus>(topic_name, buff_size);
    }

    void SystemControlPublisher::Publish(const RoverMessagePtr rover_ptr, const BaseMessagePtr base_ptr,
            const bool imu_status, const bool gnss_status) {
        PublishData(rover_ptr, base_ptr, imu_status, gnss_status);
    }

    void SystemControlPublisher::PublishData(const RoverMessagePtr rover_ptr, const BaseMessagePtr base_ptr,
            const bool imu_status, const bool gnss_status) {
        navit_msgs::SystemErrorStatusPtr control(new navit_msgs::SystemErrorStatus());
        control->header.frame_id = frame_id_;
        control->header.stamp = ros::Time::now();

        // sensor status.
        control->rover_info.imu_status = imu_status;
        control->rover_info.gnss_status = gnss_status;

        // base.
        control->base_info.satellites = base_ptr->satellites;
        control->base_info.sol_status = base_ptr->sol_status;
        control->base_info.pos_type = base_ptr->pos_type;
        control->base_info.battery1 = base_ptr->battery1;
        control->base_info.battery2 = base_ptr->battery2;
        control->base_info.base_status = base_ptr->base_status;
        control->base_info.radio_status = base_ptr->radio_status;
        control->base_info.diff_status = base_ptr->diff_status;
        control->base_info.sat_status = base_ptr->sat_status;

        // rover.
        if (rover_ptr == NULL) {
            control->rover_info.board_type = "NONE";
            control->rover_info.sol_status = "NONE";
            control->rover_info.pos_type = "NONE";
            control->rover_info.heading_type = "NONE";
            control->rover_info.diff_age = 0.;
            control->rover_info.satellites1 = 0;
            control->rover_info.satellites2 = 0;
        } else {
            control->rover_info.board_type =  rover_ptr->board_type;
            control->rover_info.sol_status = rover_ptr->sol_status;
            control->rover_info.pos_type = rover_ptr->pos_type;
            control->rover_info.heading_type = rover_ptr->heading_type;
            control->rover_info.diff_age = rover_ptr->diff_age;
            control->rover_info.satellites1 = rover_ptr->satellites1;
            control->rover_info.satellites2 = rover_ptr->satellites2;
        }

        publisher_.publish(control);
    }
} // namespace integrated_navigation