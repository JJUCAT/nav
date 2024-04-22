#include <navit_planner/planner_server.h>
#include <navit_planner/ccpp_server.h>
#include <navit_planner/smooth_path_server.h>
#include <tf2_ros/transform_listener.h>
#include <navit_planner/fcpp_server.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "navit_planner");

    ros::NodeHandle planner_nh("~");

    auto tf = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf);
    auto planner_ = std::make_shared<navit_planner::PlannerServer>(tf, planner_nh);
    auto smooth_path_planner = std::make_shared<navit_planner::SmoothPathServer>(planner_nh);
    auto ccpp_planner_ = std::make_shared<navit_planner::PolygonCoverageServer>(tf, planner_nh);
    ros::spin();
    return 0;
}
