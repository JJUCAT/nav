#include <navit_localization/ros_filter_types.h>
#include <navit_localization/LocalizationSwitch.h>
#include <ros/ros.h>


// 提供localization的服务
// 2个ekf分别用于RTK定位和激光雷达定位，ekf_se_map_rtk用于RTK定位，ekf_se_map_lidar用于激光雷达定位
// 根据ros service的请求，切换启动对应的ekf，并销毁另一个ekf
//
//
using LocalizationFilter = RobotLocalization::RosEkf;

std::unique_ptr<LocalizationFilter> ekf_se_map_rtk_ptr;
std::unique_ptr<LocalizationFilter> ekf_se_map_lidar_ptr;
ros::NodeHandle nh;
ros::NodeHandle nh_private("ekf_se_map_rtk");
ros::NodeHandle nh_private_lidar("ekf_se_map_lidar");

bool localization_switch(navit_localization::LocalizationSwitch::Request &req,
						 navit_localization::LocalizationSwitch::Response &res) {
	ROS_INFO("localization switch to lidar request is %d", req.switch_to_lidar);
	
	if (req.switch_to_lidar) {
		ROS_INFO("localization switch to lidar");
		if (ekf_se_map_rtk_ptr) {
			ekf_se_map_rtk_ptr.reset();
		}

		if (!ekf_se_map_lidar_ptr) {
			ekf_se_map_lidar_ptr.reset(new LocalizationFilter(nh, nh_private_lidar));
			ekf_se_map_lidar_ptr->initialize();
		}

		ekf_se_map_rtk_ptr.reset();
		ekf_se_map_rtk_ptr = nullptr;
		return true;
	} else {
		// 切换到RTK定位
		if (ekf_se_map_rtk_ptr == nullptr) {
			ekf_se_map_rtk_ptr = std::make_unique<LocalizationFilter>(nh, nh_private);
			ekf_se_map_rtk_ptr->initialize();
		}
		ekf_se_map_rtk_ptr->reset();
		ekf_se_map_lidar_ptr.reset();
		ekf_se_map_lidar_ptr = nullptr;
		return true;
	}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "navit_localization_node");

  ekf_se_map_rtk_ptr = std::make_unique<LocalizationFilter>(nh, nh_private);
  ekf_se_map_rtk_ptr->initialize();

  ros::ServiceServer service = nh.advertiseService("localization_switch", localization_switch);
  ros::spin();
  return 0;
}
