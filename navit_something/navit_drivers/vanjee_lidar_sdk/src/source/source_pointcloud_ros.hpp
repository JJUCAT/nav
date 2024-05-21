/*
 * @Author: guo
 * @Date: 2023-02-07 13:26:58
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-22 09:48:20
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/source/source_pointcloud_ros.hpp
 */
#pragma once
#include "source/source.hpp"
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
namespace vanjee
{
namespace lidar
{
        inline sensor_msgs::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id,
                                                 bool send_by_rows)
        {
            sensor_msgs::PointCloud2 ros_msg;

            int fields = 4;
#ifdef POINT_TYPE_XYZIRT
            fields = 6;
#endif
#ifdef POINT_TYPE_XYZHSV
            fields = 6;
#endif
            ros_msg.fields.clear();
            ros_msg.fields.reserve(fields);

            if (send_by_rows)
            {
                ros_msg.width = vanjee_msg.width;
                ros_msg.height = vanjee_msg.height;
            }
            else
            {
                ros_msg.width = vanjee_msg.height; 
                ros_msg.height = vanjee_msg.width;
            }

            int offset = 0;
            offset = addPointField(ros_msg, "x", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "y", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "z", 1, sensor_msgs::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
            offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
            offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::PointField::FLOAT32, offset);          
            offset = addPointField(ros_msg, "ring", 1, sensor_msgs::PointField::UINT16, offset);
            offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::PointField::FLOAT64, offset);
#endif
#ifdef POINT_TYPE_XYZHSV
            offset = addPointField(ros_msg, "h", 1, sensor_msgs::PointField::FLOAT32, offset);          
            offset = addPointField(ros_msg, "s", 1, sensor_msgs::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "v", 1, sensor_msgs::PointField::FLOAT32, offset);
#endif

            ros_msg.point_step = offset;
            ros_msg.row_step = ros_msg.width * ros_msg.point_step;
            ros_msg.is_dense = vanjee_msg.is_dense;
            ros_msg.data.resize(ros_msg.point_step * vanjee_msg.points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#endif            
#ifdef POINT_TYPE_XYZIRT
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
            sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
            sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif
#ifdef POINT_TYPE_XYZHSV
            sensor_msgs::PointCloud2Iterator<float> iter_h_(ros_msg, "h");
            sensor_msgs::PointCloud2Iterator<float> iter_s_(ros_msg, "s");
            sensor_msgs::PointCloud2Iterator<float> iter_v_(ros_msg, "v");
#endif

            if (send_by_rows)
            {
                for (size_t i = 0; i < vanjee_msg.height; i++)
                {
                    for (size_t j = 0; j < vanjee_msg.width; j++)
                    {
                        const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i + j * vanjee_msg.height];

                        *iter_x_ = point.x;
                        *iter_y_ = point.y;
                        *iter_z_ = point.z;                     

                        ++iter_x_;
                        ++iter_y_;
                        ++iter_z_;
#ifdef POINT_TYPE_XYZI                       
                        *iter_intensity_ = point.intensity;
                        ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
                        *iter_intensity_ = point.intensity;
                        *iter_ring_ = point.ring;
                        *iter_timestamp_ = point.timestamp;

                        ++iter_intensity_;
                        ++iter_ring_;
                        ++iter_timestamp_;
#endif
#ifdef POINT_TYPE_XYZHSV
                        *iter_h_ = point.h;
                        *iter_s_ = point.s;
                        *iter_h_= point.v;

                        ++iter_h_;
                        ++iter_s_;
                        ++iter_v_;
#endif
                    }
                }
            }
            else
            {
                for (size_t i = 0; i < vanjee_msg.points.size(); i++)
                {
                    const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i];

                    *iter_x_ = point.x;
                    *iter_y_ = point.y;
                    *iter_z_ = point.z;

                    ++iter_x_;
                    ++iter_y_;    
                    ++iter_z_;
#ifdef POINT_TYPE_XYZI                   
                    *iter_intensity_ = point.intensity;
                    ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
                    *iter_ring_ = point.ring;
                    *iter_intensity_ = point.intensity;
                    *iter_timestamp_ = point.timestamp;
                    ++iter_intensity_;
                    ++iter_ring_;
                    ++iter_timestamp_;
#endif
#ifdef POINT_TYPE_XYZHSV
                        *iter_h_= point.h;
                        *iter_s_ = point.s;
                        *iter_v_ = point.v;

                        ++iter_h_;
                        ++iter_s_;
                        ++iter_v_;
#endif
                }
            }

            ros_msg.header.seq = vanjee_msg.seq;
            ros_msg.header.stamp = ros_msg.header.stamp.fromSec(vanjee_msg.timestamp);
            ros_msg.header.frame_id = frame_id;

            return ros_msg;
        }
        /// @brief DestinationPointCloudRos在ROS主题`/vanjee_lidar_points`发布点云
        class DestinationPointCloudRos : public DestinationPointCloud
        {
        private:
            std::shared_ptr<ros::NodeHandle> nh_;
            ros::Publisher pub_;   
            std::string frame_id_; 
            bool send_by_rows_;

        public:
            /// @brief 初始化DestinationPacketRos实例
            virtual void init(const YAML::Node &config);
            /// @brief 在ROS主题`/vanjee_lidar_points`发布点云
            virtual void sendPointCloud(const LidarPointCloudMsg &msg);
            virtual ~DestinationPointCloudRos() = default;
        };

        inline void DestinationPointCloudRos::init(const YAML::Node &config)
        {
            yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, true);

            bool dense_points;

            yamlRead<bool>(config["driver"], "dense_points", dense_points, false);

            // if (dense_points)
            //     send_by_rows_ = false;
            // else
            //     send_by_rows_ = true;
            yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
            std::string ros_send_topic;
            yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

            nh_ = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle());
            pub_ = nh_->advertise<sensor_msgs::PointCloud2>(ros_send_topic, 10);
        }
        inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg)
        {
          pub_.publish(toRosMsg(msg, frame_id_, send_by_rows_));
        }
} 

} 

#endif
#ifdef ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
namespace vanjee
{
namespace lidar
{
        inline sensor_msgs::msg::PointCloud2 toRosMsg(const LidarPointCloudMsg &vanjee_msg, const std::string &frame_id,
                                                 bool send_by_rows)
        {
            sensor_msgs::msg::PointCloud2 ros_msg;

            int fields = 4;
#ifdef POINT_TYPE_XYZIRT
            fields = 6;
#endif
#ifdef POINT_TYPE_XYZHSV
            fields = 6;
#endif
            ros_msg.fields.clear();
            ros_msg.fields.reserve(fields);

            if (send_by_rows)
            {
                ros_msg.width = vanjee_msg.width;
                ros_msg.height = vanjee_msg.height;
            }
            else
            {
                ros_msg.width = vanjee_msg.height; 
                ros_msg.height = vanjee_msg.width;
            }

            int offset = 0;
            offset = addPointField(ros_msg, "x", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "y", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "z", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#ifdef POINT_TYPE_XYZI
            offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#endif
#ifdef POINT_TYPE_XYZIRT
            offset = addPointField(ros_msg, "intensity", 1, sensor_msgs::msg::PointField::FLOAT32, offset);          
            offset = addPointField(ros_msg, "ring", 1, sensor_msgs::msg::PointField::UINT16, offset);
            offset = addPointField(ros_msg, "timestamp", 1, sensor_msgs::msg::PointField::FLOAT64, offset);
#endif
#ifdef POINT_TYPE_XYZHSV
            offset = addPointField(ros_msg, "h", 1, sensor_msgs::msg::PointField::FLOAT32, offset);          
            offset = addPointField(ros_msg, "s", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
            offset = addPointField(ros_msg, "v", 1, sensor_msgs::msg::PointField::FLOAT32, offset);
#endif

#if 0
  std::cout << "off:" << offset << std::endl;
#endif

            ros_msg.point_step = offset;
            ros_msg.row_step = ros_msg.width * ros_msg.point_step;
            ros_msg.is_dense = vanjee_msg.is_dense;
            ros_msg.data.resize(ros_msg.point_step * vanjee_msg.points.size());

            sensor_msgs::PointCloud2Iterator<float> iter_x_(ros_msg, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y_(ros_msg, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z_(ros_msg, "z");
#ifdef POINT_TYPE_XYZI
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
#endif            
#ifdef POINT_TYPE_XYZIRT
            sensor_msgs::PointCloud2Iterator<float> iter_intensity_(ros_msg, "intensity");
            sensor_msgs::PointCloud2Iterator<uint16_t> iter_ring_(ros_msg, "ring");
            sensor_msgs::PointCloud2Iterator<double> iter_timestamp_(ros_msg, "timestamp");
#endif
#ifdef POINT_TYPE_XYZHSV
            sensor_msgs::PointCloud2Iterator<float> iter_h_(ros_msg, "h");
            sensor_msgs::PointCloud2Iterator<float> iter_s_(ros_msg, "s");
            sensor_msgs::PointCloud2Iterator<float> iter_v_(ros_msg, "v");
#endif

            if (send_by_rows)
            {
                for (size_t i = 0; i < vanjee_msg.height; i++)
                {
                    for (size_t j = 0; j < vanjee_msg.width; j++)
                    {
                        const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i + j * vanjee_msg.height];

                        *iter_x_ = point.x;
                        *iter_y_ = point.y;
                        *iter_z_ = point.z;                     

                        ++iter_x_;
                        ++iter_y_;
                        ++iter_z_;
#ifdef POINT_TYPE_XYZI                       
                        *iter_intensity_ = point.intensity;
                        ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
                        *iter_intensity_ = point.intensity;
                        *iter_ring_ = point.ring;
                        *iter_timestamp_ = point.timestamp;

                        ++iter_intensity_;
                        ++iter_ring_;
                        ++iter_timestamp_;
#endif
#ifdef POINT_TYPE_XYZHSV
                        *iter_h_ = point.h;
                        *iter_s_ = point.s;
                        *iter_h_= point.v;

                        ++iter_h_;
                        ++iter_s_;
                        ++iter_v_;
#endif
                    }
                }
            }
            else
            {
                for (size_t i = 0; i < vanjee_msg.points.size(); i++)
                {
                    const LidarPointCloudMsg::PointT &point = vanjee_msg.points[i];

                    *iter_x_ = point.x;
                    *iter_y_ = point.y;
                    *iter_z_ = point.z;


                    ++iter_x_;
                    ++iter_y_;    
                    ++iter_z_;
#ifdef POINT_TYPE_XYZI                   
                    *iter_intensity_ = point.intensity;
                    ++iter_intensity_;
#endif

#ifdef POINT_TYPE_XYZIRT
                    *iter_ring_ = point.ring;
                    *iter_intensity_ = point.intensity;
                    *iter_timestamp_ = point.timestamp;
                    ++iter_intensity_;
                    ++iter_ring_;
                    ++iter_timestamp_;
#endif
#ifdef POINT_TYPE_XYZHSV
                    *iter_h_= point.h;
                    *iter_s_ = point.s;
                    *iter_v_ = point.v;

                    ++iter_h_;
                    ++iter_s_;
                    ++iter_v_;
#endif
                }
            }

            ros_msg.header.stamp.sec = (uint32_t)floor(vanjee_msg.timestamp);
            ros_msg.header.stamp.nanosec = (uint32_t)round((vanjee_msg.timestamp - ros_msg.header.stamp.sec) * 1e9);
            ros_msg.header.frame_id = frame_id;

            return ros_msg;
        }
        /// @brief DestinationPointCloudRos在ROS主题`/vanjee_lidar_points`发布点云
        class DestinationPointCloudRos : virtual public DestinationPointCloud
        {
        private:
            std::shared_ptr<rclcpp::Node> node_ptr_;
            rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;   
            std::string frame_id_; 
            bool send_by_rows_;

        public:
            /// @brief 初始化DestinationPacketRos实例
            virtual void init(const YAML::Node &config);
            /// @brief 在ROS主题`/vanjee_lidar_points`发布点云
            virtual void sendPointCloud(const LidarPointCloudMsg &msg);
            virtual ~DestinationPointCloudRos() = default;
        };

        inline void DestinationPointCloudRos::init(const YAML::Node &config)
        {
            yamlRead<bool>(config["ros"], "ros_send_by_rows", send_by_rows_, true);

            bool dense_points;

            yamlRead<bool>(config["driver"], "dense_points", dense_points, false);
            
            // if (dense_points)
            //     send_by_rows_ = false;
            // else
            //     send_by_rows_ = true;
            yamlRead<std::string>(config["ros"], "ros_frame_id", frame_id_, "vanjee_lidar");
            std::string ros_send_topic;
            yamlRead<std::string>(config["ros"], "ros_send_point_cloud_topic", ros_send_topic, "vanjee_lidar_points");

            static int node_index = 0;
            std::stringstream node_name;
            node_name << "vanjee_lidar_points_destination_" << node_index++;
            node_ptr_.reset(new rclcpp::Node(node_name.str()));
            pub_ = node_ptr_->create_publisher<sensor_msgs::msg::PointCloud2>(ros_send_topic, 100);
            
        }
        inline void DestinationPointCloudRos::sendPointCloud(const LidarPointCloudMsg &msg)
        {
            pub_->publish(toRosMsg(msg, frame_id_, send_by_rows_));
        }
} 

} 

#endif