/*
 * @Author: guo
 * @Date: 2023-02-07 13:19:45
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-09 09:20:55
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/manager/node_manager.cpp
 */

#include "manager/node_manager.hpp"
#include "source/source_driver.hpp"
#include "source/source_pointcloud_ros.hpp"
#include "source/source_imu_packet_ros.hpp"
#include "source/source_scandata_ros.hpp"

namespace vanjee
{
namespace lidar
{
        
        void NodeManager::init(const YAML::Node &config)
        {
            
            YAML::Node common_config = yamlSubNodeAbort(config, "common"); 
            int msg_source = 0;
            yamlRead<int>(common_config, "msg_source", msg_source, 0);
            bool send_point_cloud_ros;
            yamlRead<bool>(common_config, "send_point_cloud_ros", send_point_cloud_ros, false); 
            bool send_imu_packet_ros;
            yamlRead<bool>(common_config, "send_imu_packet_ros", send_imu_packet_ros, false);
            bool send_laser_scan_ros;
            yamlRead<bool>(common_config, "send_laser_scan_ros", send_laser_scan_ros, false);
            
            YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
            for (uint8_t i = 0; i < lidar_config.size(); i++)
            {
                std::shared_ptr<Source> source;
                switch (msg_source)
                {
                case SourceType::MSG_FROM_LIDAR: 

                    WJ_INFO << "------------------------------------------------------" << WJ_REND;
                    WJ_INFO << "Receive Packets From : Online LiDAR" << WJ_REND;
                    WJ_INFO << "Msop Port: " << lidar_config[i]["driver"]["host_msop_port"].as<uint16_t>() << WJ_REND;
                    WJ_INFO << "------------------------------------------------------" << WJ_REND;

                    source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_LIDAR);
                    source->init(lidar_config[i]);
                    break;

                case SourceType::MSG_FROM_PCAP: 

                    WJ_INFO << "------------------------------------------------------" << WJ_REND;
                    WJ_INFO << "Receive Packets From : Pcap" << WJ_REND;
                    WJ_INFO << "Msop Port: " << lidar_config[i]["driver"]["host_msop_port"].as<uint16_t>() << WJ_REND;
                    WJ_INFO << "------------------------------------------------------" << WJ_REND;

                    source = std::make_shared<SourceDriver>(SourceType::MSG_FROM_PCAP);
                    source->init(lidar_config[i]);
                    break;

                default:
                    WJ_ERROR << "Unsupported LiDAR message source:" << msg_source << "." << WJ_REND;
                    exit(-1);
                }

                if (send_point_cloud_ros)
                {
                    WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
                    WJ_DEBUG << "Send PointCloud To : ROS" << WJ_REND;
                    WJ_DEBUG << "PointCloud Topic: " << lidar_config[i]["ros"]["ros_send_point_cloud_topic"].as<std::string>()
                             << WJ_REND;
                    WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

                    std::shared_ptr<DestinationPointCloud> dst = std::make_shared<DestinationPointCloudRos>();
                    dst->init(lidar_config[i]);
                    source->regPointCloudCallback(dst);
                }

                if(send_imu_packet_ros)
                {
                    try
                    {
                        string ros_topic_str = lidar_config[i]["ros"]["ros_send_imu_packet_topic"].as<std::string>();

                        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
                        WJ_DEBUG << "Send ImuPackets To : ROS" << WJ_REND;
                        WJ_DEBUG << "ImuPacket Topic: " << ros_topic_str << WJ_REND;
                        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

                        std::shared_ptr<DestinationImuPacket> dst = std::make_shared<DestinationImuPacketRos>();
                        dst->init(lidar_config[i]);
                        source->regImuPacketCallback(dst);
                    }
                    catch(...)
                    {
                        WJ_WARNING << "ros_send_imu_packet_topic is null" << WJ_REND;
                    }
                }

                if(send_laser_scan_ros)
                {
                    try
                    {
                        string ros_topic_str = lidar_config[i]["ros"]["ros_send_laser_scan_topic"].as<std::string>();

                        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;
                        WJ_DEBUG << "Send LaserScan To : ROS" << WJ_REND;
                        WJ_DEBUG << "LaserScan Topic: " << ros_topic_str << WJ_REND;
                        WJ_DEBUG << "------------------------------------------------------" << WJ_REND;

                        std::shared_ptr<DestinationScanDataRos> dst = std::make_shared<DestinationScanDataRos>();
                        dst->init(lidar_config[i]);
                        source->regScanDataCallback(dst);
                    }
                    catch(...)
                    {
                        WJ_WARNING << "ros_send_laser_scan_topic is null" << WJ_REND;
                    }
                }
                sources_.emplace_back(source);
            }
        }
        void NodeManager::start()
        {
            for (auto &iter : sources_)
            {
                if (iter != nullptr)
                {
                    iter->start();
                }
            }
        }

        void NodeManager::stop()
        {
            for (auto &iter : sources_)
            {
                if (iter != nullptr)
                {
                    iter->stop();
                }
            }
        }

        NodeManager::~NodeManager()
        {
            stop();
        }

}  

}  
