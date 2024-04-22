#include <navit_routing/routing_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "routing_server");

    ros::NodeHandle nh("~");

    auto cs = std::make_shared<navit_routing::RoutingServer>(nh);

    ros::spin();

    return 0;
}
