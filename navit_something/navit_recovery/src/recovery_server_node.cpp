#include <navit_recovery/recovery_server.h>
#include <tf2_ros/transform_listener.h>

using namespace navit_recovery;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "recovery_server");

    ros::NodeHandle nh("~");

    auto tf = std::make_shared<tf2_ros::Buffer>();

    tf2_ros::TransformListener tf_listener(*tf);

    auto recovery = std::make_shared<RecoveryServer>(tf, nh);

    ros::spin();

    return 0;
}
