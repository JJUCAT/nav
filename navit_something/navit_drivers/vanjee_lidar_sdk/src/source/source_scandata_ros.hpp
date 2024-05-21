/*
 * @Author: 
 * @Date: 2023-10-31
 * @LastEditors: 
 * @LastEditTime: 2023-10-31 09:48:20
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/source/source_scandata_ros.hpp
 */
#pragma once
#include "source/source.hpp"
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
namespace vanjee
{
namespace lidar
{
        inline sensor_msgs::LaserScan toRosMsg(const ScanData &vanjee_msg, const std::string &frame_id)
        {
            sensor_msgs::LaserScan ros_msg;

            ros_msg.angle_min = vanjee_msg.angle_min / 180 * 3.1415926;
            ros_msg.angle_max = vanjee_msg.angle_max / 180 * 3.1415926;
            ros_msg.angle_increment = vanjee_msg.angle_increment / 180 * 3.1415926;
            ros_msg.time_increment = vanjee_msg.time_increment;
            ros_msg.scan_time = vanjee_msg.scan_time;
            ros_msg.range_min = vanjee_msg.range_min;
            ros_msg.range_max = vanjee_msg.range_max;

            int pointNum = (vanjee_msg.angle_max - vanjee_msg.angle_min) / vanjee_msg.angle_increment;
            for(int i = 0; i < pointNum; i++)
            {
                ros_msg.ranges.emplace_back(vanjee_msg.ranges[i]);
                ros_msg.intensities.emplace_back(vanjee_msg.intensities[i]);
            }


            ros_msg.header.seq = vanjee_msg.seq;
            ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
            ros_msg.header.frame_id = frame_id;

            return ros_msg;
        }
        /// @brief DestinationScanDataRos在ROS主题`/vanjee_lidar_scan`发布点云
        class DestinationScanDataRos : public DestinationScanData
        {
        private:
            std::shared_ptr<ros::NodeHandle> nh_;
            ros::Publisher scan_data_pub_;   
            std::string frame_id_; 

        public:
            /// @brief 初始化DestinationPacketRos实例
            virtual void init(const YAML::Node &config);
            /// @brief 在ROS主题`/vanjee_lidar_scan`发布点云
            virtual void sendScanData(const ScanData &msg);
            virtual ~DestinationScanDataRos() = default;
        };

        inline void DestinationScanDataRos::init(const YAML::Node &config)
        {
            yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
            std::string ros_send_topic;
            yamlRead<std::string>(config["ros"], "ros_send_laser_scan_topic", ros_send_topic, "vanjee_scan");

            nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
            scan_data_pub_ = nh_->advertise<sensor_msgs::LaserScan>(ros_send_topic, 10);
        }
        inline void DestinationScanDataRos::sendScanData(const ScanData &msg)
        {
          scan_data_pub_.publish(toRosMsg(msg, frame_id_));
        }
} 

} 

#endif
#ifdef ROS2_FOUND
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
namespace vanjee
{
namespace lidar
{
        inline sensor_msgs::msg::LaserScan toRosMsg(const ScanData &vanjee_msg, const std::string &frame_id)
        {
	        sensor_msgs::msg::LaserScan ros_msg;

            ros_msg.angle_min = vanjee_msg.angle_min / 180 * 3.1415926;
            ros_msg.angle_max = vanjee_msg.angle_max / 180 * 3.1415926;
            ros_msg.angle_increment = vanjee_msg.angle_increment / 180 * 3.1415926;
            ros_msg.time_increment = vanjee_msg.time_increment;
            ros_msg.scan_time = vanjee_msg.scan_time;
            ros_msg.range_min = vanjee_msg.range_min;
            ros_msg.range_max = vanjee_msg.range_max;
		
	        int pointNum = (vanjee_msg.angle_max - vanjee_msg.angle_min) / vanjee_msg.angle_increment;
            for(int i = 0; i < pointNum; i++)
            {
                ros_msg.ranges.emplace_back(vanjee_msg.ranges[i]);
                ros_msg.intensities.emplace_back(vanjee_msg.intensities[i]);
            }

            //ros_msg.header.seq = vanjee_msg.seq;
            ros_msg.header.stamp.sec = (long)vanjee_msg.timestamp;
            ros_msg.header.stamp.nanosec = (long)((vanjee_msg.timestamp - (long)vanjee_msg.timestamp) * 1000000000);
            ros_msg.header.frame_id = frame_id;

            return ros_msg;
        }
        /// @brief DestinationScanDataRos在ROS主题`/vanjee_lidar_scan`发布点云
        class DestinationScanDataRos : virtual public DestinationScanData
        {
        private:
            std::shared_ptr<rclcpp::Node> node_ptr_;
            rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_data_pub_;   
            std::string frame_id_; 

        public:
            /// @brief 初始化DestinationPacketRos实例
            virtual void init(const YAML::Node &config);
            /// @brief 在ROS主题`/vanjee_lidar_scan`发布点云
            virtual void sendScanData(const ScanData &msg);
            virtual ~DestinationScanDataRos() = default;
        };

        inline void DestinationScanDataRos::init(const YAML::Node &config)
        {
            yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
            std::string ros_send_topic;
            yamlRead<std::string>(config["ros"], "ros_send_laser_scan_topic", ros_send_topic, "vanjee_scan");
            static int node_index = 0;
            std::stringstream node_name;
            node_name << "vanjee_lidar_points_destination_" << node_index++;
            node_ptr_.reset(new rclcpp::Node(node_name.str()));
            scan_data_pub_ = node_ptr_->create_publisher<sensor_msgs::msg::LaserScan>(ros_send_topic, 100);
            
        }
        inline void DestinationScanDataRos::sendScanData(const ScanData &msg)
        {
            scan_data_pub_->publish(toRosMsg(msg, frame_id_));
        }
} 

} 

#endif