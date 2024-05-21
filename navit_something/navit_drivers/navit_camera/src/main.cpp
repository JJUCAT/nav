#include <navit_camera/camera_node.h>
#include <signal.h>

void signalHandler(int signal)
{
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    ros::init(argc, argv, "navit_camera_node", ros::init_options::NoSigintHandler);
    ros::NodeHandle nh("~");
    navit_camera::CameraNode camera_node(nh);
    ros::spin();
    return 0;
}
