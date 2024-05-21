#ifndef FAKE_CAMERA_H
#define FAKE_CAMERA_H

#include <navit_camera/camera_interface.h>

namespace navit_camera {

    class FakeCamera : public CameraInterface
    {
        public:
            void initialize(CameraInfo camera_info) override
            {
               ROS_INFO_STREAM("Camera name is " << camera_info.camera_name); 
            }

            void startCamera(cv::Mat& img) override
            {
                cv::Mat fake_img = (cv::Mat_<int>(3,3) << 1,2,3,
                                                     4,5,6,
                                                     7,8,9);
                img = fake_img;
                usleep(10000);
            }
    };
}
#endif
