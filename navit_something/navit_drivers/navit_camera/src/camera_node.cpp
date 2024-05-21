#include <cv_bridge/cv_bridge.h>
#include <navit_camera/camera_node.h>

namespace navit_camera {

    CameraNode::CameraNode(ros::NodeHandle& nh): 
        nh_(nh),
        camera_plugin_loader_("navit_camera", "navit_camera::CameraInterface")
    {
        camera_param_ = CameraParam(nh_);
        camera_num_ = camera_param_.getCameraParam().size();
        camera_threads_.resize(camera_num_);
        nhs_.resize(camera_num_);
        img_pubs_.resize(camera_num_);
        camera_driver_.resize(camera_num_);

        for (unsigned int i=0; i < camera_num_; i++)
        {
            auto camera_info = camera_param_.getCameraParam()[i];
            //nhs_.push_back(ros::NodeHandle(camera_info.camera_name));
            ros::NodeHandle pnh(camera_info.camera_name);
            nhs_.push_back(pnh);
            image_transport::ImageTransport it(pnh);
            img_pubs_[i] = it.advertise("image_raw",1);
            camera_driver_[i] = camera_plugin_loader_.createInstance(camera_info.camera_type);
            camera_driver_[i]->initialize(camera_info);
        }

        startThread();
    }

    void CameraNode::startThread()
    {
        running_ = true;
        for (unsigned int i = 0; i < camera_num_; i++) {
            camera_threads_[i] = std::thread(&CameraNode::update, this, i);
          }
    }

    void CameraNode::update(const unsigned int index)
    {
        cv::Mat img;
        auto camera_info = camera_param_.getCameraParam()[index];
        bool camera_info_send = false;
          while(running_) {
              camera_driver_[index]->startCamera(img);
              if(!img.empty()) {
                    //ROS_INFO("in publish loop");
                    sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
                    img_msg->header.frame_id = camera_info.camera_name;
                    img_msg->header.stamp = ros::Time::now();
              
                    //camera_param_.getCameraParam()[index].ros_camera_info->header.stamp = img_msg->header.stamp;
                    //img_pubs_[index].publish(img_msg, camera_param_.getCameraParam()[index].ros_camera_info);
                    img_pubs_[index].publish(img_msg);
                  }
            }
    }

    void CameraNode::stopThread()
    {
      running_ = false;
    }

    CameraNode::~CameraNode() {
      running_ = false;
      for (auto &iter: camera_threads_) {
          if (iter.joinable())
            iter.join();
        }

      for (auto &plugin : camera_driver_)
      {
            plugin.reset();
      }
    }
}
