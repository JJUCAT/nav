/*
 * @Author: guo
 * @Date: 2023-02-02 16:47:57
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-29 09:35:08
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/source/source_driver.hpp
 */
#pragma once

#include "source/source.hpp"
#include <vanjee_driver/api/lidar_driver.hpp>
#include <vanjee_driver/utility/sync_queue.hpp>

namespace vanjee
{
namespace lidar
{
        class SourceDriver : public Source
        {
        public:
            virtual void init(const YAML::Node &config);
            virtual void start();
            virtual void stop();
            virtual ~SourceDriver();

            SourceDriver(SourceType src_type);

        protected:
            std::shared_ptr<LidarPointCloudMsg> getPointCloud(void);
            void putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg);
            std::shared_ptr<ImuPacket> getImuPacket(void);
            void putImuPacket(std::shared_ptr<ImuPacket> msg);
            void putException(const lidar::Error &msg);
            std::shared_ptr<ScanData> getScanData(void);
            void putScanData(std::shared_ptr<ScanData> msg);
            void processPointCloud();
            void processImuPacket();
            void processScanData();

            std::shared_ptr<lidar::LidarDriver<LidarPointCloudMsg>> driver_ptr_;
            SyncQueue<std::shared_ptr<LidarPointCloudMsg>> free_point_cloud_queue_; 
            SyncQueue<std::shared_ptr<LidarPointCloudMsg>> point_cloud_queue_;

            SyncQueue<std::shared_ptr<ImuPacket>> free_imu_packet_queue_;
            SyncQueue<std::shared_ptr<ImuPacket>> imu_packet_queue_;

            SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
            SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;

            std::thread point_cloud_process_thread_; 
            std::thread imu_packet_process_thread_;
            std::thread scan_data_process_thread_;
            bool to_exit_process_;
        };
        SourceDriver::SourceDriver(SourceType src_type)
            : Source(src_type), to_exit_process_(false)
        {
        }

        inline void SourceDriver::init(const YAML::Node &config)
        {
            YAML::Node driver_config = yamlSubNodeAbort(config, "driver");
            vanjee::lidar::WJDriverParam driver_param;
            yamlRead<uint16_t>(driver_config, "connect_type", driver_param.input_param.connect_type, 1);
            yamlRead<uint16_t>(driver_config, "host_msop_port", driver_param.input_param.host_msop_port, 3001);
            yamlRead<uint16_t>(driver_config,"lidar_msop_port",driver_param.input_param.lidar_msop_port,3333);
            yamlRead<std::string>(driver_config, "host_address", driver_param.input_param.host_address, "0.0.0.0");
            yamlRead<std::string>(driver_config, "group_address", driver_param.input_param.group_address, "0.0.0.0");
            yamlRead<std::string>(driver_config, "lidar_address", driver_param.input_param.lidar_address, "0.0.0.0");
            yamlRead<bool>(driver_config, "use_vlan", driver_param.input_param.use_vlan, false);
            yamlRead<std::string>(driver_config, "pcap_path", driver_param.input_param.pcap_path, "");
            yamlRead<bool>(driver_config, "pcap_repeat", driver_param.input_param.pcap_repeat, true);
            yamlRead<float>(driver_config, "pcap_rate", driver_param.input_param.pcap_rate, 1);
            yamlRead<uint16_t>(driver_config, "use_layer_bytes", driver_param.input_param.user_layer_bytes, 0);
            yamlRead<uint16_t>(driver_config, "tail_layer_bytes", driver_param.input_param.tail_layer_bytes, 0);

            std::string lidar_type;
            yamlReadAbort<std::string>(driver_config, "lidar_type", lidar_type);
            driver_param.lidar_type = strToLidarType(lidar_type);
            yamlRead<bool>(driver_config, "wait_for_difop", driver_param.decoder_param.wait_for_difop, false);
            yamlRead<bool>(driver_config, "use_lidar_clock", driver_param.decoder_param.use_lidar_clock, false);
            yamlRead<float>(driver_config, "min_distance", driver_param.decoder_param.min_distance, 0.2);
            yamlRead<float>(driver_config, "max_distance", driver_param.decoder_param.max_distance, 200);
            yamlRead<float>(driver_config, "start_angle", driver_param.decoder_param.start_angle, 0);
            yamlRead<float>(driver_config, "end_angle", driver_param.decoder_param.end_angle, 360);
            yamlRead<bool>(driver_config, "dense_points", driver_param.decoder_param.dense_points, false);
            yamlRead<bool>(driver_config, "ts_first_point", driver_param.decoder_param.ts_first_point, false);
            yamlRead<bool>(driver_config, "config_from_file", driver_param.decoder_param.config_from_file, true);
            yamlRead<uint16_t>(driver_config, "rpm", driver_param.decoder_param.rpm, 1200);
            yamlRead<std::string>(driver_config, "angle_path_ver", driver_param.decoder_param.angle_path_ver, "");
            yamlRead<std::string>(driver_config, "angle_path_hor", driver_param.decoder_param.angle_path_hor, "");
            yamlRead<std::string>(driver_config, "imu_param_path", driver_param.decoder_param.imu_param_path, "");
            yamlRead<uint16_t>(driver_config, "publish_mode", driver_param.decoder_param.publish_mode, 0);

            yamlRead<float>(driver_config, "x", driver_param.decoder_param.transform_param.x, 0);
            yamlRead<float>(driver_config, "y", driver_param.decoder_param.transform_param.y, 0);
            yamlRead<float>(driver_config, "z", driver_param.decoder_param.transform_param.z, 0);
            yamlRead<float>(driver_config, "roll", driver_param.decoder_param.transform_param.roll, 0);
            yamlRead<float>(driver_config, "pitch", driver_param.decoder_param.transform_param.pitch, 0);
            yamlRead<float>(driver_config, "yaw", driver_param.decoder_param.transform_param.yaw, 0);
            switch (src_type_)
            {
            case SourceType::MSG_FROM_LIDAR:
                driver_param.input_type = InputType::ONLINE_LIDAR;
                break;
            case SourceType::MSG_FROM_PCAP:
                driver_param.input_type = InputType::PCAP_FILE;
                break;
            default:
                break;
            }

            driver_param.print();

            driver_ptr_.reset(new lidar::LidarDriver<LidarPointCloudMsg>());
            driver_ptr_->regPointCloudCallback(std::bind(&SourceDriver::getPointCloud, this),
                                               std::bind(&SourceDriver::putPointCloud, this, std::placeholders::_1));
            driver_ptr_->regExceptionCallback(
                std::bind(&SourceDriver::putException, this, std::placeholders::_1));
            point_cloud_process_thread_ = std::thread(std::bind(&SourceDriver::processPointCloud, this));


            driver_ptr_->regImuPacketCallback(std::bind(&SourceDriver::getImuPacket,this),
                                              std::bind(&SourceDriver::putImuPacket,this,std::placeholders::_1));
            imu_packet_process_thread_ = std::thread(std::bind(&SourceDriver::processImuPacket,this));
            
            driver_ptr_->regScanDataCallback(std::bind(&SourceDriver::getScanData,this),
                                              std::bind(&SourceDriver::putScanData,this,std::placeholders::_1));
            scan_data_process_thread_ = std::thread(std::bind(&SourceDriver::processScanData,this));

            if (!driver_ptr_->init(driver_param))
            {
                WJ_ERROR << "Driver Initialize Error...." << WJ_REND;
                exit(-1);
            }
        }

        inline void SourceDriver::start()
        {
            driver_ptr_->start();
        }

        inline SourceDriver::~SourceDriver()
        {
            stop();
        }

        inline void SourceDriver::stop()
        {
            driver_ptr_->stop();

            to_exit_process_ = true;
            point_cloud_process_thread_.join();
        }

        /// @brief 给成员`driver_ptr_`提供空闲的点云实例
        inline std::shared_ptr<LidarPointCloudMsg> SourceDriver::getPointCloud(void)
        {
            std::shared_ptr<LidarPointCloudMsg> point_cloud = free_point_cloud_queue_.pop();
            if (point_cloud.get() != NULL)
            {
                return point_cloud;
            }

            return std::make_shared<LidarPointCloudMsg>();
        }

        inline std::shared_ptr<ImuPacket> SourceDriver::getImuPacket()
        {
          std::shared_ptr<ImuPacket> pkt = free_imu_packet_queue_.pop();
          if(pkt.get() != NULL)
          {
            return pkt;
          }

          return std::make_shared<ImuPacket>();
        }

        inline std::shared_ptr<ScanData> SourceDriver::getScanData()
        {
          std::shared_ptr<ScanData> pkt = free_scan_data_queue_.pop();
          if(pkt.get() != NULL)
          {
            return pkt;
          }

          return std::make_shared<ScanData>();
        }

        /// @brief 给从成员`driver_ptr_`得到填充好的点云
        void SourceDriver::putPointCloud(std::shared_ptr<LidarPointCloudMsg> msg)
        {
            point_cloud_queue_.push(msg);
        }

        void SourceDriver::putImuPacket(std::shared_ptr<ImuPacket> msg)
        {
            imu_packet_queue_.push(msg);
        }

        /// @brief 给从成员`driver_ptr_`得到填充好的ScanData
        void SourceDriver::putScanData(std::shared_ptr<ScanData> msg)
        {
            scan_data_queue_.push(msg);
        }

        void SourceDriver::processImuPacket()
        {
          while(!to_exit_process_)
          {
            std::shared_ptr<ImuPacket> msg = imu_packet_queue_.popWait(1000);
            if(msg.get() == NULL)
            {
              continue;
            } 

            sendImuPacket(*msg);

            free_imu_packet_queue_.push(msg);
          }
        }

        void SourceDriver::processPointCloud()
        {
            while (!to_exit_process_)
            {
                /// @brief 从待处理点云队列`point_cloud_queue`，取出一个点云实例
                std::shared_ptr<LidarPointCloudMsg> msg = point_cloud_queue_.popWait(1000);
                if (msg.get() == NULL)
                {
                    continue;
                }

                /// @brief 调用sendPointCloud()，其中调用成员`pc_cb_vec_[]`中的DestinationPointCloud实例，发送点云
                sendPointCloud(msg);
                 
                /// @brief 处理后，将它放回空闲队列`free_point_cloud_queue，待下次使用
                free_point_cloud_queue_.push(msg);
            }
        }

        void SourceDriver::processScanData()
        {
            while (!to_exit_process_)
            {
                /// @brief 从待处理点云队列`sacn_data_queue`，取出一个点云实例
                std::shared_ptr<ScanData> msg = scan_data_queue_.popWait(1000);
                if (msg.get() == NULL)
                {
                    continue;
                }
                /// @brief 调用sendScanData()，其中调用成员`pc_cb_vec_[]`中的DestinationScanData实例，发送点云
                sendScanData(*msg);
                
                /// @brief 处理后，将它放回空闲队列`free_scan_data_queue，待下次使用
                free_scan_data_queue_.push(msg);
            }
        }
        
        /// @brief 提示
        inline void SourceDriver::putException(const lidar::Error &msg)
        {
            switch (msg.error_code_type)
            {
            case lidar::ErrCodeType::INFO_CODE:
                WJ_INFO << msg.toString() << WJ_REND;
                break;
            case lidar::ErrCodeType::WARNING_CODE:
                WJ_WARNING << msg.toString() << WJ_REND;
                break;
            case lidar::ErrCodeType::ERROR_CODE:
                WJ_ERROR << msg.toString() << WJ_REND;
                break;
            }
        }
} 

} 
