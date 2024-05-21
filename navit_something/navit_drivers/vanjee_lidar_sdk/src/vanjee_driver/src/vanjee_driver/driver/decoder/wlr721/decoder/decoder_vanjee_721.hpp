/*
 * @Author: Guo Xuemei 1242143575@qq.com
 * @Date: 2023-02-08 15:32:17
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-04-06 11:08:23
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_vanjee721.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEnam
 */
#pragma once 

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/cmd_repository_721.hpp>
#include <vanjee_driver/driver/decoder/wlr721/protocol/frames/protocol_ldangle_get_721.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    typedef struct
    {
        uint8_t distance[2];      
        uint8_t intensity;      
        uint8_t reflectivity;   
    }Vanjee721Channel;
        
    typedef struct
    {
        uint8_t header[2];   
        uint8_t rotation[2]; 
        Vanjee721Channel channel[64]; 
    }Vanjee721Block;
    typedef struct
    {
        uint8_t id[4];      
        uint16_t circle_num; 
        uint16_t frame_num;  
        uint8_t channel_num;  
        uint8_t echo_mode;   
        uint16_t real_rpm;   
        uint32_t second;
        uint32_t microsecond;
        // WJTimestampUTC timestamp;
        uint8_t reserved[4];       
    }Vanjee721Difop;

    typedef struct
    {
        Vanjee721Block blocks[5];   
        Vanjee721Difop difop;
    }Vanjee721MsopPkt_1;

    typedef struct
    {
        Vanjee721Block blocks[4];   
        Vanjee721Difop difop;
    }Vanjee721MsopPkt_2;
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee721 :public DecoderMech<T_PointCloud>
    {
    private:
        int32_t preCirclesNo = -1;
        uint8_t publish_mode = 0;
        bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
        double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam& getConstParam(uint8_t mode);
    public:
        constexpr static double FRAME_DURATION = 0.1;
        constexpr static uint32_t SINGLE_PKT_NUM = 360;

        virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee721() = default;
        explicit DecoderVanjee721(const WJDecoderParam& param);

        bool decodeMsopPkt_1(const uint8_t* pkt, size_t size);
        bool decodeMsopPkt_2(const uint8_t* pkt, size_t size);
    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam& DecoderVanjee721<T_PointCloud>::getConstParam(uint8_t mode)
    {
        switch (mode)
        {
            case 1:
            {
                static WJDecoderMechConstParam param =
                    {
                        1324 
                        ,64 
                        ,5 
                        ,64 
                        ,0.3f 
                        ,45.0f 
                        ,0.004f 
                        ,80.0f 
                    };
                param.BLOCK_DURATION = 0.1 / 360;
                return param;
            }
            break;

            case 2:
            default:
            {
                static WJDecoderMechConstParam param =
                    {
                        1064 
                        ,64 
                        ,4 
                        ,64 
                        ,0.3f 
                        ,45.0f 
                        ,0.004f 
                        ,80.0f 
                    };
                param.BLOCK_DURATION = 0.1 / 360;
                return param;
            }
            break;
        }
    }

    template <typename T_PointCloud>
    inline DecoderVanjee721<T_PointCloud>::DecoderVanjee721(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
    {
        publish_mode = param.publish_mode;
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);
        if(this->param_.config_from_file)
        {
            int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver);
            this->angles_ready_ = (ret_angle == 0);
        }
    }
    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
    {
        bool ret = false;
        uint16_t l_pktheader = pkt[0]<<8 | pkt[1];
        switch(l_pktheader)
        {
            case 0xFFEE:
            {
                ret = decodeMsopPkt_1(pkt , size);
            }
            break;

            case 0xFFDD:
            {
                ret = decodeMsopPkt_2(pkt , size);
            }
            break;
        }   
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt_1(const uint8_t* pkt, size_t size)
    {
        const Vanjee721MsopPkt_1 &packet = *(Vanjee721MsopPkt_1 *) pkt;
        bool ret = false; 
        double pkt_ts = 0;
        
        int32_t loss_circles_num = (packet.difop.circle_num + 65536 - preCirclesNo) % 65536;
        if(loss_circles_num > 1 && preCirclesNo >= 0)
            WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
        preCirclesNo = packet.difop.circle_num;

        double sec = packet.difop.second;
        double usec = (double)(packet.difop.microsecond & 0x0fffffff);
        pkt_ts = sec + usec * 1e-6;
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

        for (uint16_t  blk = 0; blk < 5; blk++)
        {
            double point_time = pkt_ts + (this->packet_duration_)*(blk-1)* 1e-6;
            const Vanjee721Block& block = packet.blocks[blk];
            int32_t azimuth = block.rotation[0] << 8 | block.rotation[1];
            
            if (this->split_strategy_->newBlock(azimuth))
            {
                this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
                this->first_point_ts_ = pkt_ts;
                ret = true;       
            }

            if((block.header[0] == 255) && (block.header[1] == 238))
            {
                for (uint16_t  chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
                {
                    float x, y, z, xy;
                    
                    const Vanjee721Channel& channel = block.channel[chan];
                    float tem_dsitance = channel.distance[0] << 8 | channel.distance[1];
                    float distance = tem_dsitance * this->const_param_.DISTANCE_RES;
                    int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                    int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth*10) % 360000;
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
                    int32_t verticalVal_721 = angle_vert;

                    if (this->distance_section_.in(distance))
                    {
                        xy = distance * COS(verticalVal_721);
                        x = xy * SIN(azimuth_index);
                        y = xy * (COS(azimuth_index));
                        z = distance * SIN(verticalVal_721);
                        this->transformPoint(x, y, z);

                        typename T_PointCloud::PointT point;
                        setX(point, x);
                        setY(point, y);
                        setZ(point, z);
                        setIntensity(point, channel.intensity);
                        setTimestamp(point, point_time);
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
                        setTimestamp(point, point_time);
                        setRing(point, chan);

                        this->point_cloud_->points.emplace_back(point);
                    }
                }
                this->prev_point_ts_ =  point_time;
            }
            
        }

        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee721<T_PointCloud>::decodeMsopPkt_2(const uint8_t* pkt, size_t size)
    {
        const Vanjee721MsopPkt_2 &packet = *(Vanjee721MsopPkt_2 *) pkt;
        bool ret = false; 
        double pkt_ts = 0;

        int32_t loss_circles_num = (packet.difop.circle_num + 65536 - preCirclesNo) % 65536;
        if(loss_circles_num > 1 && preCirclesNo >= 0)
            WJ_WARNING << "loss " << loss_circles_num << " circle" << WJ_REND;
        preCirclesNo = packet.difop.circle_num;

        double sec = packet.difop.second;
        double usec = (double)(packet.difop.microsecond & 0x0fffffff);
        pkt_ts = sec + usec * 1e-6;
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

        for (uint16_t  blk = 0; blk < 4; blk++)
        {
            if((packet.difop.echo_mode & 0xf0) != 0 && publish_mode == 0 && (blk % 2 == 1))
            {
                continue;
            }
            else if((packet.difop.echo_mode & 0xf0) != 0 && publish_mode == 1 && (blk % 2 == 0))
            {
                continue;
            }

            double point_time = pkt_ts + (this->packet_duration_)*(blk-1)* 1e-6;
            const Vanjee721Block& block = packet.blocks[blk];
            int32_t azimuth = block.rotation[0] << 8 | block.rotation[1];

            if (this->split_strategy_->newBlock(azimuth))
            {
                this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
                this->first_point_ts_ = pkt_ts;
                ret = true;
                
            }
            {
                for (uint16_t  chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
                {
                    float x, y, z, xy;
                    
                    const Vanjee721Channel& channel = block.channel[chan];
                    float tem_dsitance = channel.distance[0] << 8 | channel.distance[1];
                    float distance = tem_dsitance * this->const_param_.DISTANCE_RES;
                    int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                    int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth*10) % 360000;
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
                    int32_t verticalVal_721 = angle_vert;

                    if (this->distance_section_.in(distance))
                    {
                        xy = distance * COS(verticalVal_721);
                        x = xy * SIN(azimuth_index);
                        y = xy * (COS(azimuth_index));
                        z = distance * SIN(verticalVal_721);
                        this->transformPoint(x, y, z);

                        typename T_PointCloud::PointT point;
                        setX(point, x);
                        setY(point, y);
                        setZ(point, z);
                        setIntensity(point, channel.intensity);
                        setTimestamp(point, point_time);
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
                        setTimestamp(point, point_time);
                        setRing(point, chan);

                        this->point_cloud_->points.emplace_back(point);
                    }
                }
                this->prev_point_ts_ =  point_time;
            }
            
        }

        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    void DecoderVanjee721<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
      std::shared_ptr<ProtocolAbstract> p;
      std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd,protocol->SubCmd);

      if(*sp_cmd == *(CmdRepository721::CreateInstance()->Sp_LDAngleGet))
      {
        p =std::make_shared<Protocol_LDAngleGet721>();
      }
      else
      {

      }

      p->Load(*protocol);

      std::shared_ptr<ParamsAbstract> params = p->Params;
      if(typeid(*params) == typeid(Params_LDAngle721))
      {
        std::shared_ptr<Params_LDAngle721> param = std::dynamic_pointer_cast<Params_LDAngle721>(params);

        std::vector<double> vert_angles;
        std::vector<double> horiz_angles;

        for (int NumOfLines = 0;NumOfLines < param->NumOfLines; NumOfLines ++)
        {
          vert_angles.push_back((double)(param->VerAngle[NumOfLines]/1000.0));
          horiz_angles.push_back((double)(param->HorAngle[NumOfLines]/1000.0));
        }

        this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->NumOfLines , vert_angles , horiz_angles); 

        if(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
        {
          WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
          (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
          Decoder<T_PointCloud>::angles_ready_ = true;
        }
      }
      else
      {

      }
    }

} 
   
} 
