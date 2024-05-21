/*
 * @Author: guo
 * @Date: 2023-02-01 18:57:34
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-09 16:43:28
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/source/source.hpp
 */
#pragma once

#include "msg/vanjee_msg/lidar_point_cloud_msg.hpp"
#include "utility/yaml_reader.hpp"
#include <vanjee_driver/msg/packet.hpp>
#include <vanjee_driver/msg/imu_packet.hpp>
#include <vanjee_driver/msg/scan_data_msg.hpp>

namespace vanjee
{
namespace lidar
{
        class DestinationPointCloud
        {
        public:
            typedef std::shared_ptr<DestinationPointCloud> Ptr;
            virtual void init(const YAML::Node &config) {}
            virtual void start() {}
            virtual void stop() {}
            virtual void sendPointCloud(const LidarPointCloudMsg &msg) = 0;
            virtual ~DestinationPointCloud() = default;
        };
        class DestinationImuPacket
        {
        public:
            typedef std::shared_ptr<DestinationImuPacket> Ptr;
            virtual void init(const YAML::Node &config) {}
            virtual void start() {}
            virtual void stop() {}
            virtual void sendImuPacket(const ImuPacket &msg) = 0;
            virtual ~DestinationImuPacket() = default;
        };
        class DestinationScanData
        {
        public:
            typedef std::shared_ptr<DestinationScanData> Ptr;
            virtual void init(const YAML::Node &config) {}
            virtual void start() {}
            virtual void stop() {}
            virtual void sendScanData(const ScanData &msg) = 0;
            virtual ~DestinationScanData() = default;
        };
        enum SourceType
        {
            MSG_FROM_LIDAR = 1,
            MSG_FROM_PCAP = 2
        };
        class Source
        {
        protected:
            /// @brief 点云发送函数
            void sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
            void sendImuPacket(const ImuPacket &msg);
            void sendScanData(const ScanData &msg);

            SourceType src_type_;

            std::vector<DestinationPointCloud::Ptr> pc_cb_vec_;
            std::vector<DestinationImuPacket::Ptr> imu_pkt_cb_vec_;
            std::vector<DestinationScanData::Ptr> scan_data_cb_vec_;

        public:
            typedef std::shared_ptr<Source> Ptr;
            virtual void init(const YAML::Node &config) {}
            virtual void start() {}
            virtual void stop() {}
            /// @brief 注册点云回调函数
            virtual void regPointCloudCallback(DestinationPointCloud::Ptr dst);
            /// @brief 注册IMU数据包回调函数
            virtual void regImuPacketCallback(DestinationImuPacket::Ptr dst);
            /// @brief 注册ScanData数据包回调函数
            virtual void regScanDataCallback(DestinationScanData::Ptr dst);
            virtual ~Source() = default;
            Source(SourceType src_tyoe);
        };

        inline Source::Source(SourceType src_type)
            : src_type_(src_type)
        {
        }
        inline void Source::regPointCloudCallback(DestinationPointCloud::Ptr dst)
        {
            pc_cb_vec_.emplace_back(dst);
        }
        inline void Source::regImuPacketCallback(DestinationImuPacket::Ptr dst)
        {
          imu_pkt_cb_vec_.emplace_back(dst);
        }
        inline void Source::regScanDataCallback(DestinationScanData::Ptr dst)
        {
          scan_data_cb_vec_.emplace_back(dst);
        }
        inline void Source::sendPointCloud(std::shared_ptr<LidarPointCloudMsg> msg) 
        {
            for (auto iter : pc_cb_vec_)
            {

                iter->sendPointCloud(*msg);
            }
        }
        inline void Source::sendImuPacket(const ImuPacket &msg)
        {
            for(auto iter : imu_pkt_cb_vec_)
            {
                iter->sendImuPacket(msg);
                
            }
        }
        inline void Source::sendScanData(const ScanData &msg)
        {
            for(auto iter : scan_data_cb_vec_)
            {
                iter->sendScanData(msg);
                
            }
        }

} 

} 
