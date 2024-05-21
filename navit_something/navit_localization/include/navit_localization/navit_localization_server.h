#include <ros/ros.h>
#include <navit_msgs/SwitchLocalization.h>
#include <navit_localization/ros_filter_types.h>
#include <navit_localization/ResetNavsat.h>

namespace navit_localization
{
	class LocalizationServer
	{
		public:
			LocalizationServer(ros::NodeHandle& nh);
			~LocalizationServer()
			{
				ekf_rtk_ptr_.reset();
				ekf_lidar_ptr_.reset();
				ekf_fusion_ptr_.reset();
			}

			void run() { ros::spin(); }
		private:
			ros::NodeHandle nh_;
			ros::ServiceServer switch_localization_server_;
			ros::ServiceClient reset_navsat_client_;
			bool switchLocalization(navit_msgs::SwitchLocalization::Request &req, navit_msgs::SwitchLocalization::Response &res);

			ros::NodeHandle private_nh_rtk_;
			ros::NodeHandle private_nh_lidar_;
			ros::NodeHandle private_nh_fusion_;

			// Filter
			std::unique_ptr<RobotLocalization::RosEkf> ekf_rtk_ptr_;
			std::unique_ptr<RobotLocalization::RosEkf> ekf_lidar_ptr_;
			std::unique_ptr<RobotLocalization::RosEkf> ekf_fusion_ptr_;
	};

}
