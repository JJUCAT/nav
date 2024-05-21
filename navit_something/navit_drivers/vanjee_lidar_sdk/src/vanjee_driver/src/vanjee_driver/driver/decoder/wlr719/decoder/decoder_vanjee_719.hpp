#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>

#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/cmd_repository_719.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_heart_bit.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    struct Vanjee719Block
    {
        uint32_t point_data;
    };

    struct Vanjee719MsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_no;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint16_t loss_circle_num;
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t bank_no;
        uint16_t motor_speed;
        Vanjee719Block blocks[300];
        uint32_t second;
        uint32_t subsecond;
        uint8_t remain;
        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_no = ntohs(frame_no);
            device_type = ntohs(device_type);
            loss_circle_num = ntohs(loss_circle_num);
            motor_speed = ntohs(motor_speed);
            for(int i = 0; i < 300; i++)
                blocks[i].point_data = ntohl(blocks[i].point_data);
            second = ntohl(second);
            subsecond = ntohl(subsecond);
            check = ntohs(check);
        }
    };
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee719 : public DecoderMech<T_PointCloud>
    {
    private:
        double light_vec1[10800];   // 缓存一圈点云时间差
        double lightangleVal = 0.033333333;   // 圈间隔时间

        double time_start;			// 索引为0点时间
        double time_pre;			// 索引为max点时间
        double time_step = 0.1;     // 相邻两圈索引为0点时间差

        int32_t azimuth_cur = -1.0; // 当前角度
        int32_t azimuth_pre = -1.0; // 前一个点角度
        int32_t preFramsNo = -1;

        bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
        double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment

        std::vector<uint8_t> bufCache;

        

    public:
        void setLightAngle(int32_t azimuth, uint16_t pointNum);
        void reloadLightAngle(uint16_t pointNum, int32_t azimuth_cur, int32_t azimuth_pre, double time_cur);

        constexpr static double FRAME_DURATION = 0.033333333;
        constexpr static uint32_t SINGLE_PKT_NUM = 28;
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee719() = default;
        explicit DecoderVanjee719(const WJDecoderParam &param);
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);

    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee719<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1234 // msop len
                , 1 // laser number
                , 300 // blocks per packet
                , 1 // channels per block
                , 0.2f // distance min
                , 50.0f // distance max
                , 0.001f // distance resolution
                , 80.0f // initial value of temperature
            };
            param.BLOCK_DURATION = 0.1 / 360;
        return param;
    }

    template <typename T_PointCloud>
    inline DecoderVanjee719<T_PointCloud>::DecoderVanjee719(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
    }

    template <typename T_PointCloud>
    void DecoderVanjee719<T_PointCloud>::setLightAngle(int32_t azimuth, uint16_t pointNum)
    {
        double angle_time_gap;
        angle_time_gap = lightangleVal / pointNum;
        for(int i=0;i<pointNum;i++)
        {
            light_vec1[i] = i * angle_time_gap;
        }

        if(azimuth != 0)
        {
            int pointNo = pointNum * (azimuth / 360000);
            double time_gap1 = light_vec1[pointNo];
            for(int i=0;i<pointNum;i++)
            {
                light_vec1[i] -= time_gap1;
            }
        }
    }

    template <typename T_PointCloud>
    void DecoderVanjee719<T_PointCloud>::reloadLightAngle(uint16_t pointNum, int32_t azimuth_cur, int32_t azimuth_pre, double time_cur)
    {
        if(azimuth_cur < 0 || azimuth_pre < 0 || azimuth_cur < azimuth_pre)
        {
            // if(time_cur - time_pre < -0.0003d)
            //     WJ_TITLE << "***********wlr719 gapTimeTurn************" << (time_cur - time_pre) << "   gapTimeCircle" << (time_cur - time_start) << WJ_REND;
            if(azimuth_cur < azimuth_pre && time_cur - time_pre > 0 && time_cur - time_pre < (pointNum == 10800 ? 0.05 : (pointNum == 5400 ? 0.1 : 0.034)))
            {
                time_step = time_cur - time_start;
            }
            else
            {
                if(pointNum == 10800)
                    time_step = 0.05;
                else if(pointNum == 5400)
                    time_step = 0.1;
                else if(pointNum == 3600)
                    time_step = 0.033333333;
            }
            time_start = time_cur;
            lightangleVal = time_step;
            setLightAngle(azimuth_cur, pointNum);
        }
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
    {
        bool ret = false;
        std::vector<uint8_t> data;
        if(bufCache.size() > 0)
        {
          std::copy(bufCache.begin(),bufCache.end(),std::back_inserter(data));
          std::copy(pkt, pkt+size, std::back_inserter(data));
        }
        else
        {
          std::copy(pkt, pkt+size, std::back_inserter(data));
        }
        
        bufCache.clear();
        bufCache.shrink_to_fit();

        uint32 indexLast = 0;
        for (size_t i = 0; i < data.size(); i++)
        {
            if(data.size() - i < 4)
                break;
            if(!(data[i] == 0xff && data[i+1] == 0xaa))
            {
                indexLast = i + 1;
                continue;
            }

            uint16_t frameLen = ((data[i + 2] << 8) | data[i + 3]) + 4;
            
            if(i + frameLen > data.size())
                break;

            if(!(data[i + frameLen - 2] == 0xee && data[i + frameLen - 1] == 0xee))
            {
                indexLast = i + 1;
                continue;
            }
            
            if(frameLen == 1234)
            {
                uint8_t pkt[1234];
                memcpy(pkt, &data[i], 1234);
                if(decodeMsopPkt_1(pkt, 1234))
                {
                    ret = true;
                    indexLast = i + frameLen;
                    break;
                }
            }

            i += frameLen - 1;
            indexLast = i + 1;
        }

        if(indexLast < data.size())
        {
            bufCache.assign(data.begin()+indexLast,data.end());
        }
        
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        Vanjee719MsopPkt& pkt = (*(Vanjee719MsopPkt *)packet);
        pkt.ToLittleEndian();

        bool ret = false;
        double pkt_ts = 0;
        double gapTime1900_1970 = (25567LL * 24 * 3600);

        int32_t loss_packets_num = (pkt.frame_no + 65536 - preFramsNo) % 65536;
        if(loss_packets_num > 20 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = pkt.frame_no;

        pkt_ts = (double)pkt.second - gapTime1900_1970 + ((double)pkt.subsecond * 0.23283 / 1000000000.0);

        if(pkt_ts <= 0)
        {
            pkt_ts = 0;                
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
        uint32_t resolution = 100;
        uint16_t point_num = 3600;
        uint8_t frequency = 30;
        uint16_t motor_speed = 23437500 / pkt.motor_speed;
        if(motor_speed < 600+150)
        {
            frequency = 10;
            point_num = 10800;
        }
        else if(motor_speed < 1200+150)
        {
            frequency = 20;
            point_num = 5400;
        }
        else if (motor_speed < 1800+150)
        {
            frequency = 30;
            point_num = 3600;
        }
        else
        {
            WJ_ERROR << "lidar speed error!" << WJ_REND;
            return ret;
        }
        resolution = 360000 / point_num;
        azimuth_cur = (pkt.bank_no - 1) * (360000 *300 / point_num);       
        
        reloadLightAngle(point_num, azimuth_cur, azimuth_pre, pkt_ts);
        
        if (this->split_strategy_->newBlock((int64_t)pkt.bank_no))
        {
            if(this->scan_data_->ranges.size() == point_num)
            {
                //scandata struct
                this->scan_data_->angle_min = -180;
                this->scan_data_->angle_max = 180;
                this->scan_data_->angle_increment = 360.0 / point_num;
                this->scan_data_->time_increment = lightangleVal / point_num; //1 / 36 / (float)resolution;
                this->scan_data_->scan_time = 1.0 / (float)frequency;
                this->scan_data_->range_min = this->param_.min_distance;//0.1;
                this->scan_data_->range_max = this->param_.max_distance;//200;

                this->cb_scan_data_(this->cloudTs());
            }
            this->scan_data_->ranges.clear();
            this->scan_data_->intensities.clear();
            
            //pointcould
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;

        }
        for (uint16_t point_index = 0; point_index < 300; point_index++)
        {
            int32_t azimuth = (azimuth_cur + point_index * 360000 / point_num) % 360000;

            float x, y, z, xy;
            uint32_t point_value = pkt.blocks[point_index].point_data;
            float distance = ((point_value & 0x7fff800) >> 11) / 1000.0;
            float intensity = point_value & 0x7ff;
            
            int32_t angle_vert = 0;
            int32_t angle_horiz_final = azimuth;

            if (angle_horiz_final < 0)
            {
                angle_horiz_final += 360000;
            }

            int32_t angle_horiz_final_count = (angle_horiz_final + 270000) % 360000;
            if (this->param_.start_angle < this->param_.end_angle)
            {
                if (angle_horiz_final_count < this->param_.start_angle * 1000 || angle_horiz_final_count > this->param_.end_angle * 1000)
                {
                    distance = 0;
                }
            }
            else
            {
                if (angle_horiz_final_count > this->param_.end_angle * 1000 && angle_horiz_final_count < this->param_.start_angle * 1000)
                {
                    distance = 0;
                }
            }

            int32_t azimuth_index = (angle_horiz_final + 180000) % 360000;
            int32_t verticalVal_719 = angle_vert;
            double cur_point_real_time = time_start + light_vec1[(pkt.bank_no - 1) * 300 + point_index];

            if (this->distance_section_.in(distance))
            {
                xy = distance * COS(verticalVal_719);
                x = xy * COS(azimuth_index);
                y = xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_719);
                this->transformPoint(x, y, z);

                typename T_PointCloud::PointT point;
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                setIntensity(point, intensity);
                setTimestamp(point, cur_point_real_time);
                setRing(point, 0);
                this->point_cloud_->points.emplace_back(point);

                this->scan_data_->ranges.emplace_back(xy);
                this->scan_data_->intensities.emplace_back(intensity);
            }
            else 
            {
                typename T_PointCloud::PointT point;
                if (!this->param_.dense_points)
                {
                    setX(point, NAN);
                    setY(point, NAN);
                    setZ(point, NAN);

                    this->scan_data_->ranges.emplace_back(NAN);
                }
                else
                {
                    setX(point, 0);
                    setY(point, 0);
                    setZ(point, 0);
                    
                    this->scan_data_->ranges.emplace_back(0);
                }
                setIntensity(point, 0.0);
                setTimestamp(point, cur_point_real_time);
                setRing(point, 0);

                this->scan_data_->intensities.emplace_back(0);
                this->point_cloud_->points.emplace_back(point);
            }
            
            this->prev_point_ts_ = cur_point_real_time;
            azimuth_pre = azimuth;
            time_pre = this->prev_point_ts_;
        }

        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template<typename T_PointCloud>
    void DecoderVanjee719<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
        std::shared_ptr<ProtocolAbstract719> p;
        std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

        if(*sp_cmd == *(CmdRepository719::CreateInstance()->Sp_ScanDataGet))
        {
        p = std::make_shared<Protocol_ScanDataGet719>();
        }
        else if(*sp_cmd == *(CmdRepository719::CreateInstance()->Sp_HeartBit))
        {
        p = std::make_shared<Protocol_HeartBit719>();
        }
        else
        {
        return;
        }
        p->Load(*protocol);

        std::shared_ptr<ParamsAbstract> params = p->Params;
        if(typeid(*params) == typeid(Params_ScanData719))
        {
        std::shared_ptr<Params_ScanData719> param = std::dynamic_pointer_cast<Params_ScanData719>(params);
        // if (param->data_get_flag)
        // {
        //   WJ_INFOL << "get wlr719 scan data succ" << WJ_REND;
        //   (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
        // }
        // else
        // {
        //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
        // }
        }
        else if(typeid(*params) == typeid(Params_HeartBit719))
        {
        std::shared_ptr<Params_HeartBit719> param = std::dynamic_pointer_cast<Params_HeartBit719>(params);
        // if (param->heart_bit_flag)
        // {
        //     WJ_INFOL << "get wlr719 heart bit succ" << WJ_REND;
        // }
        // else
        // {
        //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
        // }
        }
        else
        {
        WJ_WARNING << "Unknown Params Type..." << WJ_REND;
        }
    }

}   // namespace lidar
}   // namespace vanjee