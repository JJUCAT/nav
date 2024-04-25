/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-25
 * @brief 
 */

#include <ros/ros.h>
#include <tf/tf.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_listener.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "voronoi_field_node");
  ros::NodeHandle n("~");

  auto tf = std::make_shared<tf2_ros::Buffer>(); 
  tf2_ros::TransformListener tf_listener(*tf);
  auto costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>("test_costmap", *tf);

  ros::Rate r(1);
  while(ros::ok()) {
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
