#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>

#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/cmd_repository_719c.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heart_bit_tcp.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heart_bit_udp.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
    struct Vanjee719CBlock
    {
        uint8_t intensity;
        uint16_t distance;
    };

    struct Vanjee719C20MsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_no;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint16_t remain1;
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t bank_no;
        uint16_t motor_speed;
        Vanjee719CBlock blocks[300];
        uint32_t second;
        uint32_t subsecond;
        uint8_t remain2;
        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_no = ntohs(frame_no);
            device_type = ntohs(device_type);
            remain1 = ntohs(remain1);
            motor_speed = ntohs(motor_speed);
            for(int i = 0; i < 300; i++)
                blocks[i].distance = ntohs(blocks[i].distance);
            second = ntohl(second);
            subsecond = ntohl(subsecond);
            check = ntohs(check);
        }
    };

    struct Vanjee719C10And20MsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_no;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint16_t remain1;
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t bank_no;
        uint16_t motor_speed;
        Vanjee719CBlock blocks[450];
        uint32_t second;
        uint32_t subsecond;
        uint8_t remain2;
        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_no = ntohs(frame_no);
            device_type = ntohs(device_type);
            remain1 = ntohs(remain1);
            motor_speed = ntohs(motor_speed);
            for(int i = 0; i < 450; i++)
                blocks[i].distance = ntohs(blocks[i].distance);
            second = ntohl(second);
            subsecond = ntohl(subsecond);
            check = ntohs(check);
        }
    };
    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee719C : public DecoderMech<T_PointCloud>
    {
    private:
        double light_vec1[7200];   // 缓存一圈点云时间差
        double lightangleVal = 0.05;         // 圈间隔时间
        double lightlineVal = 0.00001389;    // 相邻通道时间间隔

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

        constexpr static double FRAME_DURATION = 0.05;
        constexpr static uint32_t SINGLE_PKT_NUM = 28;
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee719C() = default;
        explicit DecoderVanjee719C(const WJDecoderParam &param);
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_2(const uint8_t *pkt, size_t size);

    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee719C<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1384        // msop len
                , 4         // laser number
                , 450       // blocks per packet
                , 4         // channels per block
                , 0.2f      // distance min
                , 50.0f     // distance max
                , 0.001f    // distance resolution
                , 80.0f     // initial value of temperature
            };
            param.BLOCK_DURATION = 0.2 / 360;
        return param;
    }

    template <typename T_PointCloud>
    inline DecoderVanjee719C<T_PointCloud>::DecoderVanjee719C(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
    }

    template <typename T_PointCloud>
    void DecoderVanjee719C<T_PointCloud>::setLightAngle(int32_t azimuth, uint16_t pointNum)
    {
        double light_angle[4];
        for(int i = 0; i < 4; i++)
        {
            light_angle[i] = lightlineVal * i;
        }

        for(int i = 0; i < (pointNum / 4); i++)
        {
            for(int j = 0; j < 4; j++)
            {
                light_vec1[j + 4 * i]= i * (lightangleVal / (pointNum / 4)) + light_angle[j];
            }
        }

        if(azimuth != 0)
        {
            int pointNo = pointNum * (azimuth / 360000);
            double time_gap = light_vec1[pointNo];
            for(int i = 0; i < (pointNum / 4); i++)
            {
                for(int j = 0; j < 4; j++)
                {
                    light_vec1[j + 4 * i] -= time_gap;
                }
            }
        }
    }

    template <typename T_PointCloud>
    void DecoderVanjee719C<T_PointCloud>::reloadLightAngle(uint16_t pointNum, int32_t azimuth_cur, int32_t azimuth_pre, double time_cur)
    {
        if(azimuth_cur < 0 || azimuth_pre < 0 || azimuth_cur < azimuth_pre)
        {
            // if(time_cur - time_pre < -0.0003d)
            //     WJ_TITLE << "***********wlr719 gapTimeTurn************" << (time_cur - time_pre) << "   gapTimeCircle" << (time_cur - time_start) << WJ_REND;
            if(azimuth_cur < azimuth_pre && time_cur - time_pre > 0 && time_cur - time_pre <= (pointNum == 7200 ? 0.1 : 0.05))
            {
                time_step = time_cur - time_start;
            }
            else
            {
                if(pointNum == 7200)
                    time_step = 0.1;
                else if(pointNum == 3600)
                    time_step = 0.05;
            }
            time_start = time_cur;
            lightangleVal = time_step;
            setLightAngle(azimuth_cur, pointNum);
        }
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
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
            
            if(frameLen == 934)
            {
                uint8_t pkt[934];
                memcpy(pkt, &data[i], 934);
                if(decodeMsopPkt_1(pkt, 934))
                {
                    ret = true;
                    indexLast = i + frameLen;
                    break;
                }
            }
            else if(frameLen == 1384)
            {
                uint8_t pkt[1384];
                memcpy(pkt, &data[i], 1384);
                if(decodeMsopPkt_2(pkt, 1384))
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
    inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        Vanjee719C20MsopPkt& pkt = (*(Vanjee719C20MsopPkt *)packet);
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
        uint8_t line_num = 4;
        uint32_t resolution = 400;
        uint16_t point_num = 3600;
        uint8_t frequency = 20;
        if(pkt.motor_speed == 0)
            return false;
        uint16_t motor_speed = 23437500 / pkt.motor_speed;
        if(motor_speed < 600 + 150)
        {
            frequency = 10;
            point_num = 7200;
        }
        else if(motor_speed < 1200 + 150)
        {
            frequency = 20;
            point_num = 3600;
        }
        else
        {
            WJ_ERROR << "lidar speed error!" << WJ_REND;
            return ret;
        }
        resolution = 360000 * line_num / point_num;
        azimuth_cur = (pkt.bank_no - 1) * (300  * 360000 / point_num);
        
        reloadLightAngle(point_num, azimuth_cur, azimuth_pre, pkt_ts);

        if (this->split_strategy_->newBlock((int64_t)pkt.bank_no))
        {
            //pointcould
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;
        }
        for (uint16_t point_index = 0; point_index < 300; point_index++)
        {
            int32_t azimuth = (azimuth_cur + (point_index / line_num ) * (360000  * line_num / point_num)) % 360000;

            float x, y, z, xy;
            float distance = pkt.blocks[point_index].distance / 1000.0;
            float intensity = pkt.blocks[point_index].intensity & 0x7f;
            int32_t angle_vert[4] = {350000, 355000, 0, 300};
            int32_t angle_horiz_final = azimuth;

            if (angle_horiz_final < 0)
            {
                angle_horiz_final += 360000;
            }

            int32_t angle_horiz_final_count = (angle_horiz_final + 180000) % 360000;
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
            int32_t verticalVal_719c = angle_vert[point_index % line_num];
            double cur_point_real_time = time_start + light_vec1[(pkt.bank_no - 1) * 300 + point_index];

            if (this->distance_section_.in(distance))
            {
                xy = distance * COS(verticalVal_719c);
                x = xy * COS(azimuth_index);
                y = -xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_719c);
                this->transformPoint(x, y, z);

                typename T_PointCloud::PointT point;
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                setIntensity(point, intensity);
                setTimestamp(point, cur_point_real_time);
                setRing(point, point_index % line_num);

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
                setIntensity(point, 0.0);
                setTimestamp(point, cur_point_real_time);
                setRing(point, point_index % line_num);
                this->point_cloud_->points.emplace_back(point);
            }
            
            this->prev_point_ts_ = cur_point_real_time;
            azimuth_pre = azimuth;
            time_pre = this->prev_point_ts_;
        }

        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee719C<T_PointCloud>::decodeMsopPkt_2(const uint8_t *packet, size_t size)
    {
        Vanjee719C10And20MsopPkt& pkt = (*(Vanjee719C10And20MsopPkt *)packet);
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
        uint8_t line_num = 4;
        uint32_t resolution = 200;
        uint16_t point_num = 7200;
        uint8_t frequency = 10;
        if(pkt.motor_speed == 0)
            return false;
        uint16_t motor_speed = 23437500 / pkt.motor_speed;
        if(motor_speed < 600 + 150)
        {
            frequency = 10;
            point_num = 7200;
        }
        else if(motor_speed < 1200 + 150)
        {
            frequency = 20;
            point_num = 3600;
        }
        else
        {
            WJ_ERROR << "lidar speed error!" << WJ_REND;
            return ret;
        }
        resolution = 360000 * line_num / point_num;
        int32_t first_pack_angle = ((int32_t)(450 / line_num) + 1) * (360000 / (point_num / line_num));
        int32_t second_pack_angle = (int32_t)(450 / line_num) * (360000 / (point_num / line_num));
        int divid_bank_num = pkt.bank_no / 2;
        if(pkt.bank_no == 1)
            azimuth_cur = 0;
        else
        {
            if(pkt.bank_no % 2 == 1)
                azimuth_cur = divid_bank_num  * first_pack_angle + divid_bank_num * second_pack_angle;
            else
                azimuth_cur = (divid_bank_num - 1) * first_pack_angle + divid_bank_num  * second_pack_angle;
        }
        
        
        reloadLightAngle(point_num, azimuth_cur, azimuth_pre, pkt_ts);

        if (this->split_strategy_->newBlock((int64_t)pkt.bank_no))   //split_strategy_.newPacket(pkt.bank_no - 1)
        {
            //pointcould
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;

        }
        for (uint16_t point_index = 0; point_index < 450; point_index++)
        {
            int32_t azimuth = 0;
            if(pkt.bank_no % 2 == 1)
                azimuth = (azimuth_cur + (point_index / line_num ) * (360000  * line_num / point_num)) % 360000;
            else
                azimuth = (azimuth_cur + ((point_index + 2) / line_num ) * (360000  * line_num / point_num)) % 360000;

            float x, y, z, xy;
            float distance = (pkt.blocks[point_index].distance >> 1) / 1000.0;
            float intensity = pkt.blocks[point_index].intensity & 0x7f;
            int32_t angle_vert[4] = {350000, 355000, 0, 300};
            int32_t angle_horiz_final = azimuth;

            if (angle_horiz_final < 0)
            {
                angle_horiz_final += 360000;
            }

            int32_t angle_horiz_final_count = (angle_horiz_final + 180000) % 360000;
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
            int32_t verticalVal_719c = 0;
            if(pkt.bank_no % 2 == 1)
                verticalVal_719c = angle_vert[point_index % line_num];
            else
                verticalVal_719c = angle_vert[(point_index + 2) % line_num];
            double cur_point_real_time = time_start + light_vec1[(pkt.bank_no - 1) * 450 + point_index];

            if (this->distance_section_.in(distance))
            {
                xy = distance * COS(verticalVal_719c);
                x = xy * COS(azimuth_index);
                y = -xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_719c);
                this->transformPoint(x, y, z);

                typename T_PointCloud::PointT point;
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                setIntensity(point, intensity);
                setTimestamp(point, cur_point_real_time);
                if(pkt.bank_no % 2 == 1)
                    setRing(point, point_index % line_num);
                else
                    setRing(point, (point_index + 2) % line_num );

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
                
                setIntensity(point, 0.0);
                setTimestamp(point, cur_point_real_time);
                if(pkt.bank_no % 2 == 1)
                    setRing(point, point_index % line_num);
                else
                    setRing(point, (point_index + 2) % line_num );

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
    void DecoderVanjee719C<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
        std::shared_ptr<ProtocolAbstract719C> p;
        std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);

        if(*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_ScanDataGet))
        {
            p = std::make_shared<Protocol_ScanDataGet719C>();
        }
        else if(*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_HeartBit_Tcp))
        {
            p = std::make_shared<Protocol_HeartBit719CTcp>();
        }
        else if(*sp_cmd == *(CmdRepository719C::CreateInstance()->Sp_HeartBit_Udp))
        {
            p = std::make_shared<Protocol_HeartBit719CUdp>();
        }
        else
        {
            return;
        }
        p->Load(*protocol);

        std::shared_ptr<ParamsAbstract> params = p->Params;
        if(typeid(*params) == typeid(Params_ScanData719C))
        {
            std::shared_ptr<Params_ScanData719C> param = std::dynamic_pointer_cast<Params_ScanData719C>(params);
            // if (param->data_get_flag)
            // {
            //     WJ_INFOL << "get wlr719 scan data succ" << WJ_REND;
            //     (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
            // }
            // else
            // {
            //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
            // }

        }
        else if(typeid(*params) == typeid(Params_HeartBit719CTcp))
        {
        std::shared_ptr<Params_HeartBit719CTcp> param = std::dynamic_pointer_cast<Params_HeartBit719CTcp>(params);
            // if (param->heart_bit_flag)
            // {
            //     WJ_INFOL << "get wlr719 heart bit succ" << WJ_REND;
            // }
            // else
            // {
            //     WJ_INFOL << "get wlr719 scan data fail" << WJ_REND;
            // }
        }
        else if(typeid(*params) == typeid(Params_HeartBit719CUdp))
        {
        std::shared_ptr<Params_HeartBit719CUdp> param = std::dynamic_pointer_cast<Params_HeartBit719CUdp>(params);
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