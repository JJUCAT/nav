#ifndef CAMERA_PARAM_H
#define CAMERA_PARAM_H

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>
#include <vector>

namespace navit_camera {

    struct CameraInfo {
        std::string camera_name;
        std::string camera_type; // uvc
        std::string camera_path; // e.g. /dev/ttyUSB0
        cv::Mat camera_matrix;
        cv::Mat camera_distortion;
        unsigned int resolution_width;
        unsigned int resolution_height;
        unsigned int width_offset;
        unsigned int height_offset;

        //! camera fps
        unsigned int fps;
        //! flag of auto exposure
        bool auto_exposure;
        //! exposure value
        unsigned int exposure_value;
        //! exposure time
        unsigned int exposure_time;
        //! auto white balance
        bool auto_white_balance;
        //! auto gain
        bool auto_gain;
        //! contrast
        unsigned int contrast;

        //! camera information in form of ROS sensor_msgs
        sensor_msgs::CameraInfoPtr ros_camera_info;
        //! opencv video capture
        cv::VideoCapture cap_handle;
    };

    class CameraParam
    {
        public:
            CameraParam(ros::NodeHandle& nh);
            CameraParam(){}

            void loadCameraParam();

            void getCameraParam(std::vector<CameraInfo>& cameras_param);

            std::vector<CameraInfo>& getCameraParam();
        private:
            std::vector<CameraInfo> cameras_param_;
            ros::NodeHandle nh_;
    };

}
#endif
