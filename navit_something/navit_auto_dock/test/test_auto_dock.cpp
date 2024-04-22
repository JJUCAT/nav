#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>

TEST(AutoDockNodelet, LoadUnload)
{
  ros::NodeHandle nh;
  nodelet::Loader manager(false);
  nodelet::M_string remappings;
  nodelet::V_string nargv;

  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::StaticTransformBroadcaster tf_broadcaster;
  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(0.0, 0.0, 0.0));
  tf2::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.stamp = ros::Time::now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform = tf2::toMsg(transform);
  tf_broadcaster.sendTransform(transform_stamped);


  bool res;
  std::string name = "navit_auto_dock/AutoDockNodelet";
  std::string type = "navit_auto_dock/AutoDockNodelet";
  for (int i = 0; i < 10; ++i)
  {
    res = manager.load(name, type, remappings, nargv);
    EXPECT_TRUE(res);

    res = manager.unload(name);
    EXPECT_TRUE(res);
  }

  ROS_INFO("test_auto_dock finished");
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_auto_dock");
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

 
