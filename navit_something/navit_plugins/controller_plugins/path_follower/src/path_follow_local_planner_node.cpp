//
// Created by yjh on 23-2-24.
//
#include <ros/ros.h>

#include <path_follow_control.h>

#include <tf2_ros/transform_listener.h>

// #include <navit_controller/controller_server.h>

namespace control {
class PathFollowNode
{
public:
    PathFollowNode(){};
    ~PathFollowNode(){};
    void run();
private:

};

void PathFollowNode::run() {

}

} //namespace control

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "path_follower_node");

    auto tf = std::make_shared<tf2_ros::Buffer>();

    tf2_ros::TransformListener tf_listener(*tf);

    // auto path_follower_control = std::make_shared<navit_controller::ControllerServer>(tf);
    auto path_follower_control = std::make_shared<control::ControllerPathFollow>();

    path_follower_control.initialize();

    ros::spin();

    return 0;
}
