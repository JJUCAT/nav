#ifndef APPROCH_DOCK_FILTER_EKF_H
#define APPROCH_DOCK_FILTER_EKF_H

#include <navit_auto_dock/plugins/approach_dock_filter.h>
#include <opencv2/opencv.hpp>
#include <cmath>
#include <ros/ros.h>
#include <tf2/utils.h>

namespace navit_auto_dock {
namespace plugins {
    class PerceptionEKF : public ApproachDockFilter
    {
        public:
            ~PerceptionEKF(){
                filter_.reset();
            }
            void initialize(const std::string& name,
                            const std::shared_ptr<tf2_ros::Buffer>& tf) override;

            void reset() override;

            void update(geometry_msgs::PoseStamped& dock_pose,
                        const geometry_msgs::Twist& current_vel) override;

        protected:
            
            struct {
                std::vector<double> sigma_w = {1e-3, 1e-3, 1e-3};
                std::vector<double> sigma_u = {1e-2, 1e-2, 1e-2};
                std::vector<double> error_cov = {0.2, 0.2, 0.2};
                double dt = 0.01;
                std::string base_frame_id = "base_link";
                } config_; 

            void initKalmanFilter();
            void predictKalmanFilter(const geometry_msgs::Twist& cmd_vel);
            void updateKalmanFilter(geometry_msgs::PoseStamped& pose);

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
            std::shared_ptr<cv::KalmanFilter> filter_;
            ros::Publisher measurement_pub_;
    };
}
}
#endif
