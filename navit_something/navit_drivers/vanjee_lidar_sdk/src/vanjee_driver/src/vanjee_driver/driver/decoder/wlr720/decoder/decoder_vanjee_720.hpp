/*
 * @Author: Guo Xuemei 1242143575@qq.com
 * @Date: 2023-02-08 15:32:17
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-04-06 11:08:23
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_vanjee720.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEnam
 */
#pragma once

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/decoder/imu_calibration_param.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/cmd_repository_720.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_ldangle_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imuaddpa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imulinepa_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/frames/protocol_imu_temp_get.hpp>
#include <vanjee_driver/driver/decoder/wlr720/protocol/imu/imuParamGet.hpp>

namespace vanjee
{
    namespace lidar
    {
        #pragma pack(push, 1)
        typedef struct
        {
            uint8_t distance[2];  ///< 距离 缩小4倍
            uint8_t intensity;    ///< 强度值
            uint8_t reflectivity; ///< 反射
        } Vanjee720Channel;

        typedef struct
        {
            uint8_t rotation[2];          ///< 方位角
            Vanjee720Channel channel[16]; ///< 通道数据
        } Vanjee720Block;

        typedef struct
        {
            uint8_t rotation[2];          ///< 方位角
            Vanjee720Channel channel[19]; ///< 通道数据
        } Vanjee720Block2;

        typedef struct
        {
            uint8_t header[2];
            uint8_t rotation[2];          ///< 方位角
            Vanjee720Channel channel[19]; ///< 通道数据
        } Vanjee720BlockFFEE;

        typedef struct
        {
            uint8_t MAC_id[2];      ///<
            uint16_t circle_num; ///<
            uint8_t Datatime[6];   ///<
            uint8_t timesatmp[4];  ///<
            int16_t X_linespend;   ///<
            int16_t Y_linespend;
            int16_t Z_linespend;
            int16_t X_ADDspend;
            int16_t Y_ADDspend;
            int16_t Z_ADDspend;
            uint8_t info[34];
        } Vanjee720Difop;

        typedef struct
        {
            uint8_t head[2];
            uint8_t channelNum;
            uint8_t returnWaveNum;
            uint8_t BlockNum;
            Vanjee720Block blocks[18]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt_16;

        typedef struct
        {
            Vanjee720Block2 blocks[15]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt_19;

        typedef struct
        {
            uint8_t head[2];
            uint8_t channelNum;
            uint8_t returnWaveNum;
            uint8_t BlockNum;
            Vanjee720Block2 blocks[12]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt_19_double;

        typedef struct
        {
            Vanjee720BlockFFEE blocks[15]; ///< 数据块
            Vanjee720Difop difop;
        } Vanjee720MsopPkt_19FFEE;

        #pragma pack(pop)
        template <typename T_PointCloud>
        class DecoderVanjee720 : public DecoderMech<T_PointCloud>
        {
        private:
            double light_vec1[57600];   // 缓存16通道一圈点云时间差
            double light_vec2[68400];   // 缓存19通道一圈点云时间差

            double lightgroupVal = 0.00005555;  // 同角度下时间间隔
		    double lightlineVal = 0.00000234;   // 相邻通道时间间隔

            double time_start;			// 索引为0点时间
            double time_pre;			// 索引为max点时间
            double time_step = 0.1;     // 相邻两圈索引为0点时间差

            
            int32_t azimuth_cur = -1.0; // 当前角度
            int32_t azimuth_pre = -1.0; // 前一个点角度

            int32_t preCirclesNo = -1;

            bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
            double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）

            uint8_t publish_mode = 0;

            std::shared_ptr<SplitStrategy> split_strategy_; 
            static WJDecoderMechConstParam &getConstParam(uint8_t mode);
            static WJEchoMode getEchoMode(uint8_t mode);

        public:
            constexpr static double FRAME_DURATION = 0.1;
            constexpr static uint32_t SINGLE_PKT_NUM = 360;
            /// 从设备性息中获取回波模式，角度修正表
            // virtual void decodeDifopPkt(const uint8_t* pkt, size_t size);
            void setLightAngle(int32_t azimuth, int32_t resolution);
            void reloadLightAngle(int32_t resolution, int32_t azimuth_cur, int32_t azimuth_pre, double time_cur);

            virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
            virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
            virtual ~DecoderVanjee720() = default;
            explicit DecoderVanjee720(const WJDecoderParam &param);

            bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt_2(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt_3(const uint8_t *pkt, size_t size);
            bool decodeMsopPkt_4(const uint8_t *pkt, size_t size);

            void SendImuData(Vanjee720Difop difop , double temperature , int angimuth,double timestamp);

            uint16_t chanToline(uint16_t chan);

        public:
            std::shared_ptr<ImuParamGet720> m_imuPaGet;// = std::make_shared<NodeManager>();
            double imu_temperature;
        };
    
        template <typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::setLightAngle(int32_t azimuth, int32_t resolution)
        {
            double light_angle1[16];
            double light_angle2[19];
            //fprintf(stderr,"point  %d\n",point);

            for(int i=0;i<16;i++)
            {
                if(i<4)
                {
                    light_angle1[i] = lightlineVal * i;
                }
                else if(i<7)
                {
                    light_angle1[i] = lightlineVal * (i-4) + lightgroupVal / 4;
                }
                else if(i<12)
                {
                    light_angle1[i] = lightlineVal * (i-7) + lightgroupVal / 4 * 2;
                }
                else
                {
                    light_angle1[i] = lightlineVal * (i-12) + lightgroupVal / 4 * 3;
                }
                //fprintf(stderr,"%.12f******\n",light_angle1[i]);
            }

            for(int i=0;i<3600;i++)
            {
                for(int j=0;j<16;j++)
                {
                    light_vec1[j+16*i]= i * lightgroupVal + light_angle1[j];
                }
            }

            for(int i = 0;i<19;i++)
            {
                if(i<5)
                {
                    light_angle2[i] = lightlineVal * i;
                }
                else if(i<9)
                {
                    light_angle2[i] = lightlineVal * (i-5) + lightgroupVal / 4;
                }
                else if(i<15)
                {
                    light_angle2[i] = lightlineVal * (i-9) + lightgroupVal / 4 * 2;
                }
                else
                {
                    light_angle2[i] = lightlineVal * (i-15) + lightgroupVal / 4 * 3;
                }
                //fprintf(stderr,"%f&&&\n",light_angle2[i]);
            }

            for(int i=0;i<3600;i++)
            {
                for(int j=0;j<19;j++)
                {
                    light_vec2[j+19*i]= i * lightgroupVal + light_angle2[j];
                }
            }

            if(azimuth != 0)
            {
                int blk = azimuth / resolution;
                double time_gap1 = light_vec1[blk];
                double time_gap2 = light_vec2[blk];
                for(int i=0;i<3600;i++)
                {
                    for(int j=0;j<16;j++)
                    {
                        light_vec1[j+16*i]= i * lightgroupVal + light_angle1[j] - time_gap1;
                    }
                }

                for(int i=0;i<3600;i++)
                {
                    for(int j=0;j<19;j++)
                    {
                        light_vec2[j+19*i]= i * lightgroupVal + light_angle2[j] - time_gap2;
                    }
                }
            }
        }

        template <typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::reloadLightAngle(int32_t resolution, int32_t azimuth_cur, int32_t azimuth_pre, double time_cur)
        {
            if(azimuth_cur < 0 || azimuth_pre < 0 || azimuth_cur < azimuth_pre)
            {
                // if(time_cur - time_pre < -0.0003d)
                //     WJ_TITLE << "***********wlr720 gapTimeTurn************" << (time_cur - time_pre) << "   gapTimeCircle" << (time_cur - time_start) << WJ_REND;
                if(azimuth_cur < azimuth_pre && time_cur - time_pre > 0 && time_cur - time_pre <= 0.11)
                {
                    time_step = time_cur - time_start;
                }
                else
                {
                    time_step = 0.1;
                }
                time_start = time_cur;
                lightgroupVal = 0.00005555 * (time_step / 0.1);
                lightlineVal = 0.00000234 * (time_step / 0.1);
                setLightAngle(azimuth_cur, resolution);
            }
        }
        
        template <typename T_PointCloud>
        inline WJDecoderMechConstParam &DecoderVanjee720<T_PointCloud>::getConstParam(uint8_t mode)
        {
            //WJ_INFOL << "publish_mode ============mode=================" << mode << WJ_REND;
            uint16_t msop_len = 1253;
            uint16_t laser_num = 16;
            uint16_t block_num = 18;
            uint16_t chan_num = 16;
            float distance_min = 0.5f;
            float distance_max = 120.0f;
            float distance_resolution = 0.004f;
            float init_temperature = 80.0f;

            // if(lidar_type_id == 1 && mode == 1)
            // {
            //     msop_len = 1260;
            //     laser_num = 16;
            //     block_num = 15;
            //     chan_num = 19;
            // }
            // else if(lidar_type_id == 1 && mode == 1)
            // {
            //     msop_len = 1253;
            //     laser_num = 16;
            //     block_num = 18;
            //     chan_num= 16;
            // }
            // else if(lidar_type_id == 2 && mode == 1)
            // {
            //     msop_len = 1235;
            //     laser_num = 16;
            //     block_num = 15;
            //     chan_num = 16;
            // }
            // else if(lidar_type_id == 2 && mode == 2)
            // {
            //     msop_len = 1001;
            //     laser_num = 16;
            //     block_num = 12;
            //     chan_num = 19;
            // }

            static WJDecoderMechConstParam param =
                {
                    msop_len /// msop len
                    ,
                    laser_num /// laser number
                    ,
                    block_num /// blocks per packet
                    ,
                    chan_num /// channels per block
                    ,
                    distance_min /// distance min
                    ,
                    distance_max /// distance max
                    ,
                    distance_resolution /// distance resolution
                    ,
                    init_temperature /// initial value of temperature
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
        }
        
        template <typename T_PointCloud>
        inline WJEchoMode DecoderVanjee720<T_PointCloud>::getEchoMode(uint8_t mode)
        {
            switch (mode)
            {
            case 0x10: ///
                return WJEchoMode::ECHO_DUAL;
            case 0x20:
            case 0x30:

            default:
                return WJEchoMode::ECHO_SINGLE;
            }
        }
        
        template <typename T_PointCloud>
        inline DecoderVanjee720<T_PointCloud>::DecoderVanjee720(const WJDecoderParam &param)
            : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
        {
            publish_mode = param.publish_mode;
            this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
            split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

            m_imuPaGet = std::make_shared<ImuParamGet720>(0);
            imu_temperature = 25.0;
            if(this->param_.config_from_file)
            {
              int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
              this->angles_ready_ = (ret_angle == 0);

              int ret_imu = this->imu_calibration_param_.loadFromFile(param.imu_param_path);  
              
              m_imuPaGet->setImuTempCalibrationParams(
                                    this->imu_calibration_param_.x_axis_temp_k,this->imu_calibration_param_.x_axis_temp_b,
                                    this->imu_calibration_param_.y_axis_temp_k,this->imu_calibration_param_.y_axis_temp_b,
                                    this->imu_calibration_param_.z_axis_temp_k,this->imu_calibration_param_.z_axis_temp_b);
                                   
              m_imuPaGet->setImuAcceCalibrationParams(
                                    this->imu_calibration_param_.x_axis_acc_k,this->imu_calibration_param_.x_axis_acc_b,
                                    this->imu_calibration_param_.y_axis_acc_k,this->imu_calibration_param_.y_axis_acc_b,
                                    this->imu_calibration_param_.z_axis_acc_k,this->imu_calibration_param_.z_axis_acc_b);
            }
        }

        template <typename T_PointCloud>
        inline bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
        {
            bool ret = false;
            switch (size)
            {
            case 1260:
            {
                ret = decodeMsopPkt_1(pkt, size);
            }
            break;

            case 1253:
            {
                ret = decodeMsopPkt_2(pkt, size);
            }
            break;

            case 1235:
            {
                ret = decodeMsopPkt_3(pkt, size);
            }
            break;

            case 1001:
            {
                ret = decodeMsopPkt_4(pkt, size);
            }
            break;
            }
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt_1(const uint8_t *pkt, size_t size)
        {
            const Vanjee720MsopPkt_19FFEE &packet = *(Vanjee720MsopPkt_19FFEE *)pkt;

            int ring = packet.difop.info[13] << 8 | packet.difop.info[12];

            bool ret = false;
            double pkt_ts = 0;

            uint16_t circle_num = ((packet.difop.circle_num & 0xff00) >> 8) | ((packet.difop.circle_num & 0xff) << 8);
            uint32_t loss_circles_num = (circle_num + 65536 - preCirclesNo) % 65536;
            if(loss_circles_num > 1 && preCirclesNo >= 0)
                WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
            preCirclesNo = circle_num;

            std::tm stm;
            memset(&stm, 0, sizeof(stm));
            stm.tm_year = packet.difop.Datatime[5] + 100;
            stm.tm_mon = packet.difop.Datatime[4] - 1;
            stm.tm_mday = packet.difop.Datatime[3];
            stm.tm_hour = packet.difop.Datatime[2];
            stm.tm_min = packet.difop.Datatime[1];
            stm.tm_sec = packet.difop.Datatime[0];
            double nsec = (packet.difop.timesatmp[0] + (packet.difop.timesatmp[1] << 8) + (packet.difop.timesatmp[2] << 16) + ((packet.difop.timesatmp[3] & 0x0F) << 24)) / 100000000.0;
            pkt_ts = std::mktime(&stm) + nsec;

            if (fabs(getTimeHost() * 1e-6 - this->getPacketDuration() - pkt_ts - local2lidar_time_gap) > 0.3)
            {
              azimuth_pre = -1;
              count_local_time_flag = false;
              std::cout << " ******** reset count_local_time_flag" << std::endl;
            }
            
            if(!this->param_.use_lidar_clock)
            {
                if(!count_local_time_flag)
                {
                    //uint64_t ts = getTimeHost();
                    double ts = getTimeHost() * 1e-6 - this->getPacketDuration();
                    local2lidar_time_gap = ts - pkt_ts;
                    pkt_ts = ts;
                    count_local_time_flag = true;
                }
                else
                {
                    pkt_ts += local2lidar_time_gap;
                }
            }
 
            int rmp = (packet.difop.info[13] << 8) + packet.difop.info[12];
            int32_t resolution = rmp / 30;
            azimuth_cur = (packet.blocks[0].rotation[1] << 8 | packet.blocks[0].rotation[0]) % 36000;
            
            reloadLightAngle(resolution, azimuth_cur, azimuth_pre, pkt_ts);
            
            SendImuData(packet.difop , imu_temperature , packet.blocks[0].rotation[1]<<8 | packet.blocks[0].rotation[0] , pkt_ts);

            for (uint16_t blk = 0; blk < 15; blk++)
            {
                //double point_time = pkt_ts + (this->packet_duration_) * (blk - 1) * 1e-6;
                const Vanjee720BlockFFEE &block = packet.blocks[blk];
                int32_t azimuth = (block.rotation[1] << 8 | block.rotation[0]) % 36000;
                // std::cout <<"split_strategy_" << azimuth<< std::endl;

                if (this->split_strategy_->newBlock(azimuth))
                {
                    //std::cout << "PointNum:::" << this->point_cloud_->height * this->point_cloud_->width << std::endl;
                    //std::cout << "split_strategy_" << azimuth << std::endl;
                    this->cb_split_frame_(19, this->cloudTs());
                    this->first_point_ts_ = pkt_ts;
                    ret = true;
                }
                if ((block.header[0] == 255) && (block.header[1] == 238))
                {
                    for (uint16_t chan = 0; chan < 19; chan++)
                    {
                        float x, y, z, xy;

                        const Vanjee720Channel &channel = block.channel[chan];
                        float tem_dsitance = channel.distance[1] << 8 | channel.distance[0];
                        float distance = tem_dsitance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final;// = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }

                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.intensity);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * 19 + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            
                            setIntensity(point, 0);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * 19 + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                    this->prev_point_ts_ = time_start + light_vec2[(azimuth / resolution + 1) * 19 - 1];
                    azimuth_pre = azimuth;
                    time_pre = this->prev_point_ts_;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt_2(const uint8_t *pkt, size_t size)
        {
            const Vanjee720MsopPkt_16 &packet = *(Vanjee720MsopPkt_16 *)pkt;
            
            bool ret = false;
            double pkt_ts = 0;

            uint16_t circle_num = ((packet.difop.circle_num & 0xff00) >> 8) | ((packet.difop.circle_num & 0xff) << 8);
            uint32_t loss_circles_num = (circle_num + 65536 - preCirclesNo) % 65536;
            if(loss_circles_num > 1 && preCirclesNo >= 0)
                WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
            preCirclesNo = circle_num;

            std::tm stm;
            memset(&stm, 0, sizeof(stm));
            stm.tm_year = packet.difop.Datatime[5] + 100;
            stm.tm_mon = packet.difop.Datatime[4] - 1;
            stm.tm_mday = packet.difop.Datatime[3];
            stm.tm_hour = packet.difop.Datatime[2];
            stm.tm_min = packet.difop.Datatime[1];
            stm.tm_sec = packet.difop.Datatime[0];
            double nsec = (packet.difop.timesatmp[0] + (packet.difop.timesatmp[1] << 8) + (packet.difop.timesatmp[2] << 16) + ((packet.difop.timesatmp[3] & 0x0F) << 24)) / 100000000.0;
            pkt_ts = std::mktime(&stm) + nsec;

            if (fabs(getTimeHost() * 1e-6 - this->getPacketDuration() - pkt_ts - local2lidar_time_gap) > 0.3)
            {
              azimuth_pre = -1;
              count_local_time_flag = false;
              std::cout << " ******** reset count_local_time_flag" << std::endl;
            }
            
            if(!this->param_.use_lidar_clock)
            {
                if(!count_local_time_flag)
                {
                    //uint64_t ts = getTimeHost();
                    double ts = getTimeHost() * 1e-6 - this->getPacketDuration();
                    local2lidar_time_gap = ts - pkt_ts;
                    pkt_ts = ts;
                    count_local_time_flag = true;
                }
                else
                {
                    pkt_ts += local2lidar_time_gap;
                }
            }

            int rmp = (packet.difop.info[13] << 8) + packet.difop.info[12];
            int32_t resolution = rmp / 30;
            azimuth_cur = (packet.blocks[0].rotation[1] << 8 | packet.blocks[0].rotation[0]) % 36000;
            
            reloadLightAngle(resolution, azimuth_cur, azimuth_pre, pkt_ts);
                        
            SendImuData(packet.difop , imu_temperature , packet.blocks[0].rotation[1]<<8 | packet.blocks[0].rotation[0] , pkt_ts);

            int blacknum = packet.returnWaveNum * packet.BlockNum;
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                if(packet.returnWaveNum == 2 && publish_mode == 0 && (blk % 2 == 1))
                {
                    continue;
                }
                else if(packet.returnWaveNum == 2 && publish_mode == 1 && (blk % 2 == 0))
                {
                    continue;
                }
                //double point_time = pkt_ts + (this->packet_duration_) * (blk - 1) * 1e-6;
                const Vanjee720Block &block = packet.blocks[blk];
                int32_t azimuth = (block.rotation[1] << 8 | block.rotation[0]) % 36000;
                // std::cout <<"split_strategy_" << azimuth<< std::endl;
                if (this->split_strategy_->newBlock(azimuth))
                {
                    this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
                    this->first_point_ts_ = pkt_ts;
                    ret = true;
                }
                {
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        const Vanjee720Channel &channel = block.channel[chan];
                        float tem_dsitance = channel.distance[1] << 8 | channel.distance[0];
                        float distance = tem_dsitance * this->const_param_.DISTANCE_RES;
                        int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                        int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth * 10) % 360000;
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }


                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.intensity);
                            setTimestamp(point, time_start + light_vec1[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                            
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            setIntensity(point, 0);
                            setTimestamp(point, time_start + light_vec1[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                    this->prev_point_ts_ = time_start + light_vec1[(azimuth / resolution + 1) * pkt[2] - 1];//point_time;
                    azimuth_pre = azimuth;
                    time_pre = this->prev_point_ts_;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }

        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt_3(const uint8_t *pkt, size_t size)
        {
            auto &packet = *(Vanjee720MsopPkt_19 *)&pkt[5];
            int ring = packet.difop.info[12] << 8 | packet.difop.info[11];

            bool ret = false;
            double pkt_ts = 0;

            uint16_t circle_num = ((packet.difop.circle_num & 0xff00) >> 8) | ((packet.difop.circle_num & 0xff) << 8);
            uint32_t loss_circles_num = (circle_num + 65536 - preCirclesNo) % 65536;
            if(loss_circles_num > 1 && preCirclesNo >= 0)
                WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
            preCirclesNo = circle_num;

            std::tm stm;
            memset(&stm, 0, sizeof(stm));
            stm.tm_year = packet.difop.Datatime[5] + 100;
            stm.tm_mon = packet.difop.Datatime[4] - 1;
            stm.tm_mday = packet.difop.Datatime[3];
            stm.tm_hour = packet.difop.Datatime[2];
            stm.tm_min = packet.difop.Datatime[1];
            stm.tm_sec = packet.difop.Datatime[0];
            double nsec = (packet.difop.timesatmp[0] + (packet.difop.timesatmp[1] << 8) + (packet.difop.timesatmp[2] << 16) + ((packet.difop.timesatmp[3] & 0x0F) << 24)) / 100000000.0;
            pkt_ts = std::mktime(&stm) + nsec;

            if (fabs(getTimeHost() * 1e-6 - this->getPacketDuration() - pkt_ts - local2lidar_time_gap) > 0.3)
            {
              azimuth_pre = -1;
              count_local_time_flag = false;
              std::cout << " ******** reset count_local_time_flag" << std::endl;
            }
            
            if(!this->param_.use_lidar_clock)
            {
                if(!count_local_time_flag)
                {
                    //uint64_t ts = getTimeHost();
                    double ts = getTimeHost() * 1e-6 - this->getPacketDuration();
                    local2lidar_time_gap = ts - pkt_ts;
                    pkt_ts = ts;
                    count_local_time_flag = true;
                }
                else
                {
                    pkt_ts += local2lidar_time_gap;
                }
            }

            int rmp = (packet.difop.info[13] << 8) + packet.difop.info[12];
            int32_t resolution = rmp / 30;
            azimuth_cur = (packet.blocks[0].rotation[1] << 8 | packet.blocks[0].rotation[0]) % 36000;
            
            reloadLightAngle(resolution, azimuth_cur, azimuth_pre, pkt_ts);

            SendImuData(packet.difop , imu_temperature , packet.blocks[0].rotation[1]<<8 | packet.blocks[0].rotation[0] , pkt_ts);

            int blacknum = pkt[3] * pkt[4];
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                // double point_time = pkt_ts + (this->packet_duration_) * (blk - 1) * 1e-6;
                const Vanjee720Block2 &block = packet.blocks[blk];
                int32_t azimuth = (block.rotation[1] << 8 | block.rotation[0]) % 36000;
                // std::cout <<"split_strategy_" << azimuth<< std::endl;

                if (this->split_strategy_->newBlock(azimuth))
                {
                    //std::cout << "PointNum:::" << this->point_cloud_->height * this->point_cloud_->width << std::endl;
                    //std::cout << "split_strategy_" << azimuth << std::endl;
                    this->cb_split_frame_(19, this->cloudTs());
                    this->first_point_ts_ = pkt_ts;
                    ret = true;
                }
                
                {
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        const Vanjee720Channel &channel = block.channel[chan];
                        float tem_dsitance = channel.distance[1] << 8 | channel.distance[0];
                        float distance = tem_dsitance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final;// = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }

                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.intensity);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            setIntensity(point, 0);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                    this->prev_point_ts_ = time_start + light_vec2[(azimuth / resolution + 1) * pkt[2] - 1];
                    azimuth_pre = azimuth;
                    time_pre = this->prev_point_ts_;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }


        template <typename T_PointCloud>
        bool DecoderVanjee720<T_PointCloud>::decodeMsopPkt_4(const uint8_t *pkt, size_t size)
        {
            auto &packet = *(Vanjee720MsopPkt_19_double *)pkt;
            int ring = packet.difop.info[12] << 8 | packet.difop.info[11];

            bool ret = false;
            double pkt_ts = 0;

            uint16_t circle_num = ((packet.difop.circle_num & 0xff00) >> 8) | ((packet.difop.circle_num & 0xff) << 8);
            uint32_t loss_circles_num = (circle_num + 65536 - preCirclesNo) % 65536;
            if(loss_circles_num > 1 && preCirclesNo >= 0)
                WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
            preCirclesNo = circle_num;

            std::tm stm;
            memset(&stm, 0, sizeof(stm));
            stm.tm_year = packet.difop.Datatime[5] + 100;
            stm.tm_mon = packet.difop.Datatime[4] - 1;
            stm.tm_mday = packet.difop.Datatime[3];
            stm.tm_hour = packet.difop.Datatime[2];
            stm.tm_min = packet.difop.Datatime[1];
            stm.tm_sec = packet.difop.Datatime[0];
            double nsec = (packet.difop.timesatmp[0] + (packet.difop.timesatmp[1] << 8) + (packet.difop.timesatmp[2] << 16) + ((packet.difop.timesatmp[3] & 0x0F) << 24)) / 100000000.0;
            pkt_ts = std::mktime(&stm) + nsec;

            if (fabs(getTimeHost() * 1e-6 - this->getPacketDuration() - pkt_ts - local2lidar_time_gap) > 0.3)
            {
              azimuth_pre = -1;
              count_local_time_flag = false;
              std::cout << " ******** reset count_local_time_flag" << std::endl;
            }

            if(!this->param_.use_lidar_clock)
            {
                if(!count_local_time_flag)
                {
                    //uint64_t ts = getTimeHost();
                    double ts = getTimeHost() * 1e-6 - this->getPacketDuration();
                    local2lidar_time_gap = ts - pkt_ts;
                    pkt_ts = ts;
                    count_local_time_flag = true;
                }
                else
                {
                    pkt_ts += local2lidar_time_gap;
                }
            }

            int rmp = (packet.difop.info[13] << 8) + packet.difop.info[12];
            int32_t resolution = rmp / 30;
            azimuth_cur = (packet.blocks[0].rotation[1] << 8 | packet.blocks[0].rotation[0]) % 36000;
            reloadLightAngle(resolution, azimuth_cur, azimuth_pre, pkt_ts);
            
            SendImuData(packet.difop , imu_temperature , packet.blocks[0].rotation[1]<<8 | packet.blocks[0].rotation[0] , pkt_ts);

            int blacknum = packet.returnWaveNum * packet.BlockNum;
            for (uint16_t blk = 0; blk < blacknum; blk++)
            {
                if(packet.returnWaveNum == 2 && publish_mode == 0 && (blk % 2 == 1))
                {
                    continue;
                }
                else if(packet.returnWaveNum == 2 && publish_mode == 1 && (blk % 2 == 0))
                {
                    continue;
                }
                // double point_time = pkt_ts + (this->packet_duration_) * (blk - 1) * 1e-6;
                const Vanjee720Block2 &block = packet.blocks[blk];
                int32_t azimuth = (block.rotation[1] << 8 | block.rotation[0]) % 36000;
                // std::cout <<"split_strategy_" << azimuth<< std::endl;

                if (this->split_strategy_->newBlock(azimuth))
                {
                    //std::cout << "PointNum:::" << this->point_cloud_->height * this->point_cloud_->width << std::endl;

                    //std::cout << "split_strategy_" << azimuth << std::endl;
                    this->cb_split_frame_(19, this->cloudTs());
                    this->first_point_ts_ = pkt_ts;
                    ret = true;
                }
                
                {
                    for (uint16_t chan = 0; chan < pkt[2]; chan++)
                    {
                        float x, y, z, xy;

                        const Vanjee720Channel &channel = block.channel[chan];
                        float tem_dsitance = channel.distance[1] << 8 | channel.distance[0];
                        float distance = tem_dsitance * this->const_param_.DISTANCE_RES;

                        int l_line = chanToline(chan+1);
                        int32_t angle_vert = this->chan_angles_.vertAdjust(l_line);
                        int32_t angle_horiz_final;// = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                        switch (chan)
                        {
                        case 0:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;

                        case 5:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 1) % 360000;
                            break;

                        case 9:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 2) % 360000;
                            break;

                        case 14:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10 + ring / 12 * 3) % 360000;
                            break;

                        default:
                            angle_horiz_final = this->chan_angles_.horizAdjust(l_line, azimuth * 10) % 360000;
                            break;
                        }
                        if (angle_horiz_final < 0)
                        {
                            angle_horiz_final += 360000;
                        }

                        if (this->param_.start_angle < this->param_.end_angle)
                        {
                            if (angle_horiz_final < this->param_.start_angle * 1000 || angle_horiz_final > this->param_.end_angle * 1000)
                            {
                                distance = 0;
                            }
                        }
                        else
                        {
                            if (angle_horiz_final > this->param_.end_angle * 1000 && angle_horiz_final < this->param_.start_angle * 1000)
                            {
                                distance = 0;
                            }
                        }

                        int32_t azimuth_index = angle_horiz_final;
                        int32_t verticalVal_720 = angle_vert;

                        if (this->distance_section_.in(distance))
                        {
                            xy = distance * COS(verticalVal_720);
                            x = xy * SIN(azimuth_index);
                            y = xy * (COS(azimuth_index));
                            z = distance * SIN(verticalVal_720);
                            this->transformPoint(x, y, z);

                            typename T_PointCloud::PointT point;
                            setX(point, x);
                            setY(point, y);
                            setZ(point, z);
                            setIntensity(point, channel.intensity);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                        else 
                        {
                            typename T_PointCloud::PointT point;
                            if (!this->param_.dense_points)
                            {
                                setX(point, NAN);
                                setY(point, NAN);
                                setZ(point, NAN);
                            }
                            else
                            {
                                setX(point, 0);
                                setY(point, 0);
                                setZ(point, 0);
                            }
                            
                            setIntensity(point, 0);
                            setTimestamp(point, time_start + light_vec2[(azimuth / resolution) * pkt[2] + chan]);
                            setRing(point, chan);

                            this->point_cloud_->points.emplace_back(point);
                        }
                    }
                    this->prev_point_ts_ = time_start + light_vec2[(azimuth / resolution + 1) * pkt[2] - 1];
                    azimuth_pre = azimuth;
                    time_pre = this->prev_point_ts_;
                }
            }

            this->prev_pkt_ts_ = pkt_ts;
            return ret;
        }


        template <typename T_PointCloud>
        inline uint16_t DecoderVanjee720<T_PointCloud>::chanToline(uint16_t chan)
        {
            uint16_t line;
            switch (chan)
            {
            case 1:
                line = 8;
                break;

            case 2:
                line = 1;
                break;

            case 3:
                line = 2;
                break;

            case 4:
                line = 3;
                break;

            case 5:
                line = 4;
                break;

            case 6:
                line = 8;
                break;

            case 7:
                line = 5;
                break;

            case 8:
                line = 6;
                break;

            case 9:
                line = 7;
                break;

            case 10:
                line = 8;
                break;

            case 11:
                line = 9;
                break;

            case 12:
                line = 10;
                break;

            case 13:
                line = 11;
                break;

            case 14:
                line = 12;
                break;

            case 15:
                line = 8;
                break;

            case 16:
                line = 13;
                break;

            case 17:
                line = 14;
                break;

            case 18:
                line = 15;
                break;

            case 19:
                line = 16;
                break;
            }

            return line-1;
        }


        template <typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::SendImuData(Vanjee720Difop difop , double temperature , int angimuth , double timestamp)
        {
            if ((angimuth % 3600) == 0)
            {
                int32 time = (difop.timesatmp[0] + (difop.timesatmp[1] << 8) + (difop.timesatmp[2] << 16) + ((difop.timesatmp[3] & 0x0F) << 24));
            
                bool l_Getflag = m_imuPaGet->imuGet(difop.X_linespend , difop.Y_linespend, difop.Z_linespend, difop.X_ADDspend, difop.Y_ADDspend, difop.Z_ADDspend, temperature, time);
                if (l_Getflag)
                {
                    this->imu_packet_->timestamp = timestamp;
                    this->imu_packet_->angular_voc[0] = m_imuPaGet->imuResultStu.xang;
                    this->imu_packet_->angular_voc[1] = m_imuPaGet->imuResultStu.yang;
                    this->imu_packet_->angular_voc[2] = m_imuPaGet->imuResultStu.zang;

                    this->imu_packet_->linear_acce[0] = m_imuPaGet->imuResultStu.xadd;
                    this->imu_packet_->linear_acce[1] = m_imuPaGet->imuResultStu.yadd;
                    this->imu_packet_->linear_acce[2] = m_imuPaGet->imuResultStu.zadd;

                    this->imu_packet_->orientation[0] = m_imuPaGet->imuResultStu.q1;
                    this->imu_packet_->orientation[1] = m_imuPaGet->imuResultStu.q2;
                    this->imu_packet_->orientation[2] = m_imuPaGet->imuResultStu.q3;
                    this->imu_packet_->orientation[3] = m_imuPaGet->imuResultStu.q0;

                    this->cb_imu_pkt_();
                }
                
            }
            
        }

        template<typename T_PointCloud>
        void DecoderVanjee720<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
        {
          std::shared_ptr<ProtocolAbstract720> p;
          std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

          if(*sp_cmd == *(CmdRepository720::CreateInstance()->Sp_LDAngleGet))
          {
            p = std::make_shared<Protocol_LDAngleGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->Sp_ImuLineParamGet))
          {
            p = std::make_shared<Protocol_ImuLineGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->Sp_ImuAddParamGet))
          {
            p = std::make_shared<Protocol_ImuAddGet720>();
          }
          else if(*sp_cmd == *(CmdRepository720::CreateInstance()->Sp_TemperatureParamGet))
          {
            p = std::make_shared<Protocol_ImuTempGet>();
          }
          else
          {
            return;
          }
          p->Load(*protocol);

          std::shared_ptr<ParamsAbstract> params = p->Params;
          if(typeid(*params) == typeid(Params_LDAngle720))
          {
            std::shared_ptr<Params_LDAngle720> param = std::dynamic_pointer_cast<Params_LDAngle720>(params);

            std::vector<double> vert_angles;
            std::vector<double> horiz_angles;

            for (int NumOfLines = 0; NumOfLines < param->NumOfLines; NumOfLines++)
            {
              vert_angles.push_back((double)(param->VerAngle[NumOfLines] / 1000.0));
              horiz_angles.push_back((double)(0));
            }

            this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->NumOfLines , vert_angles , horiz_angles);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
              Decoder<T_PointCloud>::angles_ready_ = true;
            }
          }
          else if(typeid(*params) == typeid(Params_IMULine720))
          {
            std::shared_ptr<Params_IMULine720> param = std::dynamic_pointer_cast<Params_IMULine720>(params);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<IMU> angular_vel data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            }

            this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path,this->imu_calibration_param_.TEMP_PARAM,
                                                        param->X_K,param->X_B,param->Y_K , param->Y_B,param->Z_K , param->Z_B );

            fprintf(stderr, "\n%f  %f  %f ; %f  %f  %f\n\n" , param->X_K , param->X_B , param->Y_K , param->Y_B , param->Z_K , param->Z_B);

            m_imuPaGet->setImuTempCalibrationParams(param->X_K , param->X_B , param->Y_K , param->Y_B , param->Z_K , param->Z_B);

          }
          else if(typeid(*params) == typeid(Params_IMUAdd720))
          {
            std::shared_ptr<Params_IMUAdd720> param = std::dynamic_pointer_cast<Params_IMUAdd720>(params);

            if (Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
            {
              WJ_INFOL << "Get LiDAR<IMU> linear_acc data..." << WJ_REND;
              (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            }

            this->imu_calibration_param_.loadFromLiDAR(this->param_.imu_param_path,this->imu_calibration_param_.ACC_PARAM,
                                                            param->X_K,param->X_B,param->Y_K , param->Y_B,param->Z_K , param->Z_B );

            fprintf(stderr, "\n%f  %f  %f ; %f  %f  %f\n\n" , param->X_K , param->X_B , param->Y_K , param->Y_B , param->Z_K , param->Z_B);

            m_imuPaGet->setImuAcceCalibrationParams(param->X_K , param->X_B , param->Y_K , param->Y_B , param->Z_K , param->Z_B);
          }
          else if(typeid(*params) == typeid(Params_LiDARRunStatus))
          {
            std::shared_ptr<Params_LiDARRunStatus> param = std::dynamic_pointer_cast<Params_LiDARRunStatus>(params);

            WJ_INFOL << "imu_temp:" << param->imu_temp << WJ_REND;

            imu_temperature = param->imu_temp / 100.0f;            
          }
          else
          {
            WJ_WARNING << "Unknown Params Type..." << WJ_REND;
          }

        }

    } // namespace lidar

} // namespace vanjee
