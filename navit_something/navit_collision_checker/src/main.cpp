#include <navit_collision_checker/collision_checker.h>
#include <tf2_ros/transform_listener.h>

using namespace navit_collision_checker;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_collision_checker"); 
    ros::NodeHandle nh;
    auto costmap_sub = std::make_shared<CostmapSub>(nh, "/controller_server/controller_costmap/costmap");
    auto footprint_sub = std::make_shared<FootprintSub>(nh, "/controller_server/controller_costmap/footprint");
    auto tf = std::make_shared<tf2_ros::Buffer>();
    tf2_ros::TransformListener tf_listener(*tf);

    CollisionChecker collision_checker(
            costmap_sub,
            footprint_sub,
            tf,
            "collision_checker",
            "odom",
            "base_link",
            0.8);

    geometry_msgs::Pose2D test_pose2d;
    test_pose2d.x = 5.0;
    test_pose2d.y = 0.0;
    test_pose2d.theta = 0.0;

    geometry_msgs::PoseStamped test_pose;
    test_pose.header.frame_id = "odom";
    test_pose.pose.position.x = 5.0;
    test_pose.pose.orientation.w = 1.0;

    nav_msgs::Path test_path;
    test_path.header.frame_id = "odom";
    test_path.poses.resize(30);
    double d = 0.2;

    for(int i = 0; i < 30; i++)
    {
        test_path.poses[i].pose.position.x = i*d; 
        test_path.poses[i].pose.position.y = 0.1 * i * d; 
        test_path.poses[i].header.frame_id = "odom";
    }

    auto pose_pub = nh.advertise<geometry_msgs::PoseStamped>("collision_test_pose",1);
    auto path_pub = nh.advertise<nav_msgs::Path>("collision_test_path",1);

    bool collision = false;

    ros::Rate r(10.0);
    while (nh.ok())
    {
       collision = collision_checker.isCollisionFree(test_pose);
       ROS_INFO("current pose score is %s", collision == true? "Collision Free" : "in Collision!" );

       collision = collision_checker.isCollisionFree(test_path);
       ROS_INFO("current path is %s", collision == true? "Collision Free" : "in Collision!" );

       test_path.header.stamp = ros::Time::now();
       pose_pub.publish(test_pose);
       path_pub.publish(test_path);
       ros::spinOnce();
       r.sleep(); 
    }


    return 0;
}
