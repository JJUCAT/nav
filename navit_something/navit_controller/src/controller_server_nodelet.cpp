#include <navit_controller/controller_server_nodelet.h>


namespace navit_controller
{
    void ControllerServerNodelet::onInit()
    {
        NODELET_DEBUG("Initializing ControllerServerNodelet...");

        auto tf_buffer = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
        tf2_ros::TransformListener tf_listener(*tf_buffer);

        controller_server_.reset(new ControllerServer(tf_buffer, getPrivateNodeHandle()));
        
        NODELET_DEBUG("Initialized ControllerServerNodelet");
    }
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(navit_controller::ControllerServerNodelet, nodelet::Nodelet)
