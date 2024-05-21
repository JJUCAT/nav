#include "uvc.h"
#include <ros/ros.h>

#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>    /* For O_RDWR */
#include <unistd.h>   /* For open(), creat() */

namespace navit_camera {

    void UvcCamera::initialize(CameraInfo camera_info)
    {
        camera_info_ = camera_info;
        camera_initialized_ = false;
    }

    void UvcCamera::startCamera(cv::Mat& img)
    {
        if(!camera_initialized_){
            camera_info_.cap_handle.open(camera_info_.camera_path);
            setCameraExposure(camera_info_.camera_path, camera_info_.exposure_value);
            //ROS_ASSERT_MSG(camera_info_.cap_handle.isOpened(), "Cannot open %s .", cameras_[camera_num].video_path.c_str());
            camera_initialized_ = true;
          }
          else {
              camera_info_.cap_handle >> img;
            }
    }

    void UvcCamera::setCameraExposure(std::string id, int val)
    {
  		int cam_fd;
  		if ((cam_fd = open(id.c_str(), O_RDWR)) == -1) {
  		  std::cerr << "Camera open error" << std::endl;
  		}

  		struct v4l2_control control_s;
  		control_s.id = V4L2_CID_AUTO_WHITE_BALANCE;
  		control_s.value = camera_info_.auto_white_balance;
  		ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  		control_s.id = V4L2_CID_EXPOSURE_AUTO;
  		control_s.value = V4L2_EXPOSURE_MANUAL;
  		ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);

  		// Set exposure value
  		control_s.id = V4L2_CID_EXPOSURE_ABSOLUTE;
  		control_s.value = val;
  		ioctl(cam_fd, VIDIOC_S_CTRL, &control_s);
  		close(cam_fd);
    }

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navit_camera::UvcCamera, navit_camera::CameraInterface)
