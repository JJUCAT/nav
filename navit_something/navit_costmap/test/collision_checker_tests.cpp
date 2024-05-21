#include <gtest/gtest.h>
#include <ros/ros.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <navit_costmap/collision_checker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/Pose2D.h>
#include <navit_costmap/testing_helper.h>

using namespace navit_costmap;

tf2_ros::TransformListener* tfl_;
tf2_ros::Buffer* tf_;
std::shared_ptr<Costmap2DROS> costmap_ptr_;
std::shared_ptr<CollisionChecker> collision_checker_;
/*
 * For reference, the static map looks like this:
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0 254 254 254   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0 254   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0 254 254 254
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   0   0   0   0   0   0   0   0   0   0
 *
 *   upper left is 0,0, lower right is 9,9
 */


TEST( CollisionChecker, testFootprint)
{
  //footprint: [[-0.1, -0.1], [-0.1, 0.1], [0.1, 0.1], [0.16, 0.0], [0.1, -0.1]]
  std::vector<geometry_msgs::Point> footprint = costmap_ptr_->getRobotFootprint();
  EXPECT_EQ( 5, footprint.size() );

  EXPECT_EQ( -0.1f, footprint[ 0 ].x );
  EXPECT_EQ( -0.1f, footprint[ 0 ].y );
  EXPECT_EQ( 0.0f, footprint[ 0 ].z );

  EXPECT_EQ( -0.1f, footprint[ 1 ].x );
  EXPECT_EQ( 0.1f, footprint[ 1 ].y );
  EXPECT_EQ( 0.0f, footprint[ 1 ].z );

  EXPECT_EQ( 0.1f, footprint[ 2 ].x );
  EXPECT_EQ( 0.1f, footprint[ 2 ].y );
  EXPECT_EQ( 0.0f, footprint[ 2 ].z );
  
  EXPECT_EQ( 0.16f, footprint[ 3 ].x );
  EXPECT_EQ( 0.0f, footprint[ 3 ].y );
  EXPECT_EQ( 0.0f, footprint[ 3 ].z );

  EXPECT_EQ( 0.1f, footprint[ 4 ].x );
  EXPECT_EQ( -0.1f, footprint[ 4 ].y );
  EXPECT_EQ( 0.0f, footprint[ 4 ].z );
}

TEST( CollisionChecker, testInCollisionPose)
{
  geometry_msgs::Pose2D origin_pose;

  EXPECT_EQ(false, collision_checker_->isCollisionFree(origin_pose));
}

TEST( CollisionChecker, testCollisionFreePose)
{
  geometry_msgs::Pose2D pose;
  pose.x = 2.0;
  pose.y = 7.3;
  pose.theta = 1.5;

  EXPECT_EQ(true, collision_checker_->isCollisionFree(pose));
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "collision_checker_tests_node");

  tf_ = new tf2_ros::Buffer( ros::Duration( 10 ));
  tfl_ = new tf2_ros::TransformListener(*tf_);

  // This empty transform is added to satisfy the constructor of
  // Costmap2DROS, which waits for the transform from map to base_link
  // to become available.
  geometry_msgs::TransformStamped base_rel_map;
  base_rel_map.transform = tf2::toMsg(tf2::Transform::getIdentity());
  base_rel_map.child_frame_id = "base_link";
  base_rel_map.header.frame_id = "map";
  base_rel_map.header.stamp = ros::Time::now();
  tf_->setTransform( base_rel_map, "collision_checker_tests_node" );
  
  costmap_ptr_ = std::make_shared<Costmap2DROS>("test_costmap",*tf_);

  collision_checker_ = std::make_shared<CollisionChecker>(costmap_ptr_);
  
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
