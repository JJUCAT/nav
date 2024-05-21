/*
 * @Author: guo
 * @Date: 2023-02-01 15:47:29
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-28 02:32:48
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_vanjee733.hpp
 */
#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr733/protocol/frames/cmd_repository_733.hpp>
#include <vanjee_driver/driver/decoder/wlr733/protocol/frames/protocol_ldangle_get_733.hpp>
#include <vanjee_driver/driver/decoder/wlr733/protocol/params/params_ldangle_733.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    typedef struct 
    {
        uint16_t distance;      ///< 距离 缩小4倍
        uint8_t intensity;      ///< 强度值
        uint8_t reflectivity;   ///< 反射率

    }Vanjee733Channel;
        
    typedef struct 
    {
        uint16_t header;   ///< 数据块头FFEE
        uint16_t rotation; ///< 方位角
        Vanjee733Channel channel[64]; ///< 通道数据
    }Vanjee733Block;

    typedef struct 
    {
        uint8_t time_syn_state;     ///< 时间同步状态
        uint8_t time_syn_source;    ///< 时间同步来源
        uint8_t time_syn_error;     ///< 时间同步误差
        uint8_t protocol_ver;       ///< 协议版本号
        uint8_t echo_mode;          ///< 回波模式
        uint8_t pc_msg;             ///< 点云附加信息
        uint8_t reserved[18];       ///< 预留
    }Vanjee733DifopPkt;

    typedef struct 
    {
        Vanjee733Block blocks[4];   ///< 数据块
        uint8_t status[16];         ///< 设备id
        uint16_t circle_num;        ///< 扫描圈序号
        uint16_t frame_num;         ///< 圈内包序号
        uint32_t second;            ///< UNIX时间戳 s
        uint32_t microsecond;       ///< UNIX时间戳 us
        Vanjee733DifopPkt difop;    ///< 设备信息           
    }Vanjee733MsopPkt;
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee733 : public DecoderMech<T_PointCloud>
    {
    private:
        int32_t preCirclesNo = -1;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment
        int32_t preAzimuth = -1;
        uint8_t publish_mode = 0;
        bool first_time_come_flag = true;

        bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
        double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）
    public:
        constexpr static double FRAME_DURATION = 0.05;
        constexpr static uint32_t SINGLE_PKT_NUM = 450;
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee733() = default;
        explicit DecoderVanjee733(const WJDecoderParam &param);

        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);
    };
    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee733<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1092 // msop len
                , 64 // laser number
                , 4 // blocks per packet
                , 64 // channels per block
                , 0.3f // distance min
                , 200.0f // distance max
                , 0.004f // distance resolution
                , 80.0f // initial value of temperature
            };
        param.BLOCK_DURATION = 0.1 / 360;
        return param;
    }

    template <typename T_PointCloud>
    inline DecoderVanjee733<T_PointCloud>::DecoderVanjee733(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
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
    inline bool DecoderVanjee733<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
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
            default:
                break;
        }  
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee733<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        const Vanjee733MsopPkt &pkt = *(Vanjee733MsopPkt *)packet;
        bool ret = false;
        double pkt_ts = 0;

        int16_t near_packe_no_gap = 1;
        if((pkt.difop.echo_mode & 0x80) == 0x80)
            near_packe_no_gap = 2;
        int32_t loss_circles_num = (pkt.circle_num + 65536 - preCirclesNo) % 65536;
        if(loss_circles_num > near_packe_no_gap && preCirclesNo >= 0)
            WJ_WARNING << "loss " << (loss_circles_num / near_packe_no_gap) << " circle" << WJ_REND;
        preCirclesNo = pkt.circle_num;

        double sec = pkt.second;
        double usec = pkt.microsecond;
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

        uint16_t pkt_seq = pkt.frame_num;
        int32_t azimuth_cur = pkt.blocks[0].rotation;
        if (this->split_strategy_->newBlock(azimuth_cur))
        {
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;
        }
        
        for (uint16_t blk = 0; blk < this->const_param_.BLOCKS_PER_PKT; blk++)
        {
            if(publish_mode == 0 && (pkt.difop.echo_mode & 0x80) == 0x80 && (blk % 2) == 1)
            {
                continue;
            }
            else if(publish_mode == 1 && (pkt.difop.echo_mode & 0x80) == 0x80 && (blk %2) == 0)
            {
                continue;
            }

            const  Vanjee733Block& block = pkt.blocks[blk];

            double point_time = pkt_ts + (this->packet_duration_)*(blk-1)* 1e-6;

            for (uint16_t chan = 0; chan < this->const_param_.CHANNELS_PER_BLOCK; chan++)
            {
                float xy, x, y, z;
                const  Vanjee733Channel& channel = block.channel[chan];
                
                float distance =channel.distance * this->const_param_.DISTANCE_RES;
                int32_t angle_vert = this->chan_angles_.vertAdjust(chan);
                int32_t azimuth = block.rotation;
                int32_t angle_horiz_final = this->chan_angles_.horizAdjust(chan, azimuth*10);
                int32_t azimuth_index = ((angle_horiz_final) + 360000) % 360000;
                int32_t verticalVal_733 = angle_vert;
                verticalVal_733 = ((verticalVal_733) + 360000) % 360000;

                if (this->param_.start_angle < this->param_.end_angle)
                {
                    if (azimuth_index < (this->param_.start_angle * 1000) || azimuth_index > (this->param_.end_angle * 1000))
                    {
                        distance = 0;
                    }
                }
                else
                {
                    if (azimuth_index < (this->param_.start_angle * 1000) && azimuth_index > (this->param_.end_angle * 1000))
                    {
                        distance = 0;
                    }
                }

                if (this->distance_section_.in(distance))
                {
                    xy = distance * COS(verticalVal_733);
                    x = xy * SIN(azimuth_index);
                    y = -xy * COS(azimuth_index);
                    z = distance * SIN(verticalVal_733);
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
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    void DecoderVanjee733<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
      std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd,protocol->SubCmd);
      std::shared_ptr<ProtocolAbstract> p;

      if(*sp_cmd == *(CmdRepository733::CreateInstance()->Sp_LDAngleGet))
      {
        p =std::make_shared<Protocol_LDAngleGet733>();
      }
      else
      {
        return;
      }

      p->Load(*protocol);

      std::shared_ptr<ParamsAbstract> params = p->Params;
      
      if(typeid(*params) == typeid(Params_LDAngle733))
      {
        std::shared_ptr<Params_LDAngle733> param = std::dynamic_pointer_cast<Params_LDAngle733>(params);

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
      // if(*sp_cmd == *(CmdRepository733::CreateInstance()->Sp_LDAngleGet))
      // {
      //   Params_LDAngle733 params_LDAngle733;
      //   params_LDAngle733.Load(*protocol);
      //   std::vector<int32_t> vert_angles;
      //   std::vector<int32_t> horiz_angles;
      //   for(int i=0;i<64;i++)
      //   {
      //     vert_angles.emplace_back(params_LDAngle733.VerAngle[i]);
      //     horiz_angles.emplace_back(params_LDAngle733.HorAngle[i]);
      //   }
      //   this->chan_angles_.loadFromLiDAR(this->param_.angle_path_ver , param->NumOfLines , vert_angles,horiz_angles);

      //   if(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_ != nullptr)
      //   {
      //     WJ_INFOL << "Get LiDAR<LD> angle data..." << WJ_REND;
      //     (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
      //     Decoder<T_PointCloud>::angles_ready_ = true;
      //   }
      //}
    }
  } // namespace lidar

} // namespace vanjee
