#ifndef CAMERA_NODE_H
#define CAMERA_NODE_H

#include <thread>
#include <vector>

#include <navit_camera/camera_interface.h>
#include <pluginlib/class_loader.hpp>
#include <image_transport/image_transport.h>

#include <ros/ros.h>

namespace navit_camera {

    class CameraNode
    {
        public:
            using CameraLoader = pluginlib::ClassLoader<CameraInterface>;

            CameraNode(ros::NodeHandle& nh);

            void startThread();
            void stopThread();
            void update(const unsigned int camera_index);

            ~CameraNode();

        private:
            std::vector<boost::shared_ptr<CameraInterface> > camera_driver_;
            CameraParam camera_param_;
            unsigned int camera_num_;
            bool running_;
            std::vector<std::thread> camera_threads_;
            std::vector<ros::NodeHandle> nhs_;
            std::vector<image_transport::Publisher> img_pubs_;
            ros::NodeHandle nh_;

            CameraLoader camera_plugin_loader_;
    };
}
#endif
