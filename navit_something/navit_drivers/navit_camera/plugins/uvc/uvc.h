#ifndef UVC_CAMERA_H
#define UVC_CAMERA_H

#include <navit_camera/camera_interface.h>

namespace navit_camera {
    class UvcCamera : public CameraInterface
    {
        public:
            void initialize(CameraInfo camera_info) override;

            void startCamera(cv::Mat& img) override;

            void stopCamera();

        private:
            void setCameraExposure(std::string id, int val);

            bool read_camera_initialized_;
    };
}
#endif
