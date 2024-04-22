#include <gtest/gtest.h>
#include <ros/ros.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <navit_controller/controller_server_nodelet.h>
#include <tf2_ros/static_transform_broadcaster.h>

TEST(ControllerServerNodelet, LoadUnload)
{
  ros::NodeHandle nh;
  nodelet::Loader manager(nh);
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  std::string name = "controller_server_nodelet";
  std::string type = "navit_controller/ControllerServerNodelet";

  // boardcast fake static tf map to base_link
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  geometry_msgs::TransformStamped static_transformStamped;
  static_transformStamped.header.stamp = ros::Time::now();
  static_transformStamped.header.frame_id = "map";
  static_transformStamped.child_frame_id = "base_link";
  static_transformStamped.transform.translation.x = 0.0;
  static_transformStamped.transform.translation.y = 0.0;
  static_transformStamped.transform.translation.z = 0.0;
  static_transformStamped.transform.rotation.x = 0.0;
  static_transformStamped.transform.rotation.y = 0.0;
  static_transformStamped.transform.rotation.z = 0.0;
  static_transformStamped.transform.rotation.w = 1.0;
  static_broadcaster.sendTransform(static_transformStamped);


  bool ret;
  for (int i = 0; i < 50; i++)
  {
	  ret= manager.load(name, type, remappings, my_argv);
	  EXPECT_TRUE(ret);

	  ret = manager.unload(name);
	  EXPECT_TRUE(ret);
	  
	  ret = manager.load(name, type, remappings, my_argv);
	  EXPECT_TRUE(ret);

	  ret = manager.unload(name);
	  EXPECT_TRUE(ret);
  }
}

//TODO: test action call and result


//TODO: test service call 

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller_server_nodelet_test");
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

