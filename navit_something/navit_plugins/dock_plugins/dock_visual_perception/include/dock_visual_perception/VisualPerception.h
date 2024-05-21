#ifndef VISUAL_PERCEPTION_H
#define VISUAL_PERCEPTION_H
#include <string>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <navit_auto_dock/plugins/approach_dock_perception.h>
#include <pluginlib/class_loader.hpp>

#include "Camera.h"
#include "MarkerDetector.h"
#include <tf/tf.h>
// #include <iostream>
// #include <opencv2/opencv.hpp>
// #include <math.h>
// #include <sensor_msgs/CameraInfo.h>
// #include <image_transport/image_transport.h>
// #include <geometry_msgs/PoseStamped.h>

namespace dock_visual_perception
{
    class VisualPerception : public navit_auto_dock::plugins::ApproachDockPerception
    {
        public:
            void initialize(const std::string name, const std::shared_ptr<tf2_ros::Buffer> &tf);
            bool getPose(geometry_msgs::PoseStamped &pose);
            bool start(const geometry_msgs::PoseStamped &target_pose);
            bool stop();
            void process(const sensor_msgs::ImageConstPtr& image_msg);
            geometry_msgs::PoseStamped toRobotFrame(geometry_msgs::PoseStamped &pose);
            // virtual void initialize(sensor_msgs::CameraInfo &camera,tf::StampedTransform CamToOutput);
            // virtual bool get_image(sensor_msgs::ImagePtr& image);
            // virtual bool get_pose();
            // virtual std::pair<geometry_msgs::PoseStamped,int> process(const sensor_msgs::ImageConstPtr& image_msg);
            
            virtual ~VisualPerception(){};
            VisualPerception(){};
            
            // Storge the results of detection
            MarkerDetector<MarkerData> marker_detector_;
            // confidence labels the validation
            int confidence_, marker_resolution_, marker_margin_, input_id_;
            double marker_size_, max_new_marker_error_, max_track_error_, max_frequency_;
            std::string cam_image_topic_, cam_info_topic_, output_frame_;
            bool show_msg_;
            Camera *cam_;
            image_transport::Subscriber image_sub_;

            geometry_msgs::TransformStamped CamToOutputMsg_;
            geometry_msgs::PoseStamped markerPoseMsg_,tagPoseMsg_;

            std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

            enum status_{START_,ERROR_,INVALID_,VALID_};

            double roll_, pitch_, yaw_;
            ros::Publisher qr_code_pub_;
    };
};
#endif