#include <navit_controller/controller_server.h>
#include <tf2_ros/transform_listener.h>
// #include <navit_controller/ccpp_controller_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "controller_server");

    ros::NodeHandle nh("~");

    auto tf_buffer1 = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));
    // auto tf_buffer2 = std::make_shared<tf2_ros::Buffer>(ros::Duration(10.0));

    tf2_ros::TransformListener tf_listener1(*tf_buffer1);
    // tf2_ros::TransformListener tf_listener2(*tf_buffer2);

    auto cs = std::make_shared<navit_controller::ControllerServer>(tf_buffer1, nh);
    // auto ccs = std::make_shared<navit_controller::CCPPControllerServer>(tf_buffer2, nh);

    ros::spin();
    return 0;
}
