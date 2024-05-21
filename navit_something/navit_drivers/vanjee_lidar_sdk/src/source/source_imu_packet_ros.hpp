#pragma once
#include <source/source.hpp>

#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
namespace vanjee
{
  namespace lidar
  {
      inline sensor_msgs::Imu toRosMsg(const ImuPacket &msg, const std::string &frame_id)
      {
        sensor_msgs::Imu ros_msg;

        ros_msg.orientation.x = msg.orientation[0];
        ros_msg.orientation.y = msg.orientation[1];
        ros_msg.orientation.z = msg.orientation[2];
        ros_msg.orientation.w = msg.orientation[3];

        ros_msg.angular_velocity.x = msg.angular_voc[0];
        ros_msg.angular_velocity.y = msg.angular_voc[1];
        ros_msg.angular_velocity.z = msg.angular_voc[2];

        ros_msg.linear_acceleration.x = msg.linear_acce[0];
        ros_msg.linear_acceleration.y = msg.linear_acce[1];
        ros_msg.linear_acceleration.z = msg.linear_acce[2];

        for(int i = 0; i < 9; i++)
        {
          ros_msg.orientation_covariance[i] = msg.orientation_covariance[i];
          ros_msg.angular_velocity_covariance[i] = msg.angular_voc_covariance[i];
          ros_msg.linear_acceleration_covariance[i] = msg.linear_acce_covariance[i];
        }

        ros_msg.header.seq = msg.seq;
        ros_msg.header.stamp = ros_msg.header.stamp.fromSec(msg.timestamp);
        ros_msg.header.frame_id = frame_id;

        return ros_msg;
      }

      class DestinationImuPacketRos:public DestinationImuPacket
      {
        private:
            std::unique_ptr<ros::NodeHandle> nh_;
            ros::Publisher imu_pkt_pub_; 
            std::string frame_id_;   

        public:
            virtual void init(const YAML::Node &config);
            virtual void sendImuPacket(const ImuPacket &msg);
            virtual ~DestinationImuPacketRos() = default;

      };
      inline void DestinationImuPacketRos::init(const YAML::Node &config)
      {
          yamlRead<std::string>(config["ros"],
                                "ros_frame_id", frame_id_, "vanjee_lidar_imu");

          std::string ros_send_topic;
          yamlRead<std::string>(config["ros"], "ros_send_imu_packet_topic",
                                ros_send_topic, "vanjee_lidar_imu_packets");

          nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
          imu_pkt_pub_ = nh_->advertise<sensor_msgs::Imu>(ros_send_topic, 10);
      }

      inline void DestinationImuPacketRos::sendImuPacket(const ImuPacket &msg)
      {
           imu_pkt_pub_.publish(toRosMsg(msg, frame_id_));
      }

  }
}

#endif

#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
namespace vanjee
{
  namespace lidar
  {

    inline sensor_msgs::msg::Imu toRosMsg(const ImuPacket &msg, const std::string &frame_id)
    {
      sensor_msgs::msg::Imu ros_msg;

        ros_msg.orientation.x = msg.orientation[0];
        ros_msg.orientation.y = msg.orientation[1];
        ros_msg.orientation.z = msg.orientation[2];
        ros_msg.orientation.w = msg.orientation[3];

        ros_msg.angular_velocity.x = msg.angular_voc[0];
        ros_msg.angular_velocity.y = msg.angular_voc[1];
        ros_msg.angular_velocity.z = msg.angular_voc[2];

        ros_msg.linear_acceleration.x = msg.linear_acce[0];
        ros_msg.linear_acceleration.y = msg.linear_acce[1];
        ros_msg.linear_acceleration.z = msg.linear_acce[2];

        for(int i = 0; i < 9; i++)
        {
          ros_msg.orientation_covariance[i] = msg.orientation_covariance[i];
          ros_msg.angular_velocity_covariance[i] = msg.angular_voc_covariance[i];
          ros_msg.linear_acceleration_covariance[i] = msg.linear_acce_covariance[i];
        }

        //ros_msg.header.seq = msg.seq;
        ros_msg.header.stamp.sec = (uint32_t)floor(msg.timestamp);;
        ros_msg.header.stamp.nanosec = (uint32_t)round((msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
        ros_msg.header.frame_id = frame_id;

        return ros_msg;
    }

      class DestinationImuPacketRos:public DestinationImuPacket
      {
        private:
            std::shared_ptr<rclcpp::Node> node_ptr_;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;  
            std::string frame_id_;   

        public:
            virtual void init(const YAML::Node &config);
            virtual void sendImuPacket(const ImuPacket &msg);
            virtual ~DestinationImuPacketRos() = default;
      };

       inline void DestinationImuPacketRos::init(const YAML::Node &config)
      {
          yamlRead<std::string>(config["ros"],
                                "ros_frame_id", frame_id_, "vanjee_lidar_imu");

          std::string ros_send_topic;
          yamlRead<std::string>(config["ros"], "ros_send_imu_packet_topic",
                                ros_send_topic, "vanjee_lidar_imu_packets");

          static int node_index = 0;
          std::stringstream node_name;
          node_name << "vanjee_lidar_points_destination_" << node_index++;
          node_ptr_.reset(new rclcpp::Node(node_name.str()));
          pub_ = node_ptr_->create_publisher<sensor_msgs::msg::Imu>(ros_send_topic, 100);
      }

      inline void DestinationImuPacketRos::sendImuPacket(const ImuPacket &msg)
      {
           pub_->publish(toRosMsg(msg, frame_id_));
      }
  }
}
#endif