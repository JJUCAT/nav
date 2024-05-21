#ifndef NAVIT_CAMERA__CAMERA_INTERFACE_H
#define NAVIT_CAMERA__CAMERA_INTERFACE_H

#include <opencv2/opencv.hpp>
#include <navit_camera/camera_param.h>

namespace navit_camera {

    class CameraInterface 
    {
        public:

            virtual ~CameraInterface(){}

            virtual void initialize(CameraInfo camera_info) = 0;

            virtual void startCamera(cv::Mat& img) = 0;

        protected:
            bool camera_initialized_;
            CameraInfo camera_info_;
    };
}
#endif
