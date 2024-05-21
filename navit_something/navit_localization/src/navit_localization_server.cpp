#include <navit_localization/navit_localization_server.h>

namespace navit_localization
{

	LocalizationServer::LocalizationServer(ros::NodeHandle& nh) : nh_(nh)
	{
		// Initialize the server
		switch_localization_server_ = nh_.advertiseService("switch_localization", &LocalizationServer::switchLocalization, this);

		// defualt use rtk localization
		private_nh_rtk_ = ros::NodeHandle("~ekf_map_rtk");
		private_nh_lidar_ = ros::NodeHandle("~ekf_map_lidar");

		ros::NodeHandle private_nh(nh_, "ekf_map_rtk");
		// ekf_rtk_ptr_ = std::make_unique<RobotLocalization::RosEkf>(nh_, private_nh);
		// ekf_rtk_ptr_->initialize();

		reset_navsat_client_ = nh_.serviceClient<navit_localization::ResetNavsat>("/reset_navsat");

		ROS_INFO("navit_localization_server: Please switch localization type");
	}

	bool LocalizationServer::switchLocalization(navit_msgs::SwitchLocalization::Request& req,
												navit_msgs::SwitchLocalization::Response& res)
	{
		/*
		 *  navit_msgs::SwitchLocalization的定义
			# use_lidar use_rtk   result
			#    true      true   FUSION_LOCALIZATION
			#    false     true   RTK_LOCALIZATION 
			#    false     false  NO_LOCALIZATION
			#    true      false  LIDAR_LOCALIZATION
			
			bool use_lidar
			bool use_rtk
			
			---
			uint8 NO_LOCALIZATION      = 0
			uint8 RTK_LOCALIZATION     = 1
			uint8 LIDAR_LOCALIZATION   = 2
			uint8 FUSION_LOCALIZATION  = 3
			uint8 REQUEST_ERROR        = 255
			
			uint8 result
		 */

		// 1. check the request
		if (req.use_lidar && req.use_rtk)
		{
			res.result = navit_msgs::SwitchLocalization::Response::FUSION_LOCALIZATION;
			ROS_WARN("navit_localization_server: FUSE_LOCALIZATION is not supported yet");
			return true;
		}
		else if (!req.use_lidar && req.use_rtk)
		{
			res.result = navit_msgs::SwitchLocalization::Response::RTK_LOCALIZATION;
			ROS_INFO("navit_localization_server: Switch to RTK localization");
			
			// reset other filters
			ekf_lidar_ptr_.reset();
			ekf_fusion_ptr_.reset();

			if (ekf_rtk_ptr_ == nullptr)
			{
				ros::NodeHandle private_nh(nh_, "ekf_map_rtk");
				ekf_rtk_ptr_ = std::make_unique<RobotLocalization::RosEkf>(nh_, private_nh);
				ekf_rtk_ptr_->initialize();
				navit_localization::ResetNavsat request;
				request.request.req = true;
				reset_navsat_client_.call(request);
				
				return true;
			}

			ROS_WARN("navit_localization_server: RTK localization is already running");
		}
		else if (!req.use_lidar && !req.use_rtk)
		{
			res.result = navit_msgs::SwitchLocalization::Response::NO_LOCALIZATION;
			ROS_INFO("navit_localization_server: Switch to NO localization");
			
			// reset all filters
			ekf_lidar_ptr_.reset();
			ekf_rtk_ptr_.reset();
			ekf_fusion_ptr_.reset();
		}
		else if (req.use_lidar && !req.use_rtk)
		{
			res.result = navit_msgs::SwitchLocalization::Response::LIDAR_LOCALIZATION;
			ROS_INFO("navit_localization_server: Switch to LIDAR localization");

			// reset other filters
			ekf_rtk_ptr_.reset();
			ekf_fusion_ptr_.reset();

			if (ekf_lidar_ptr_ == nullptr)
			{
				ros::NodeHandle private_nh(nh_, "ekf_map_lidar");
				ekf_lidar_ptr_ = std::make_unique<RobotLocalization::RosEkf>(nh_, private_nh);
				ekf_lidar_ptr_->initialize();
				return true;
			}

			ROS_WARN("navit_localization_server: LIDAR localization is already running");
		}
		else
		{
			ROS_ERROR("navit_localization_server: switchLocalization request error");
			res.result = navit_msgs::SwitchLocalization::Response::REQUEST_ERROR;
			return false;
		}

		return true;
	}

} // namespace navit_localization

