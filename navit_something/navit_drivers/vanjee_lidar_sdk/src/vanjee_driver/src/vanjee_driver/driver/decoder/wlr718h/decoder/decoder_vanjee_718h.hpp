#pragma once
#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>

#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)

    struct Vanjee718hBlock1
    {
        uint16_t points[500];

        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            for(int i = 0; i < 500; i++)
                points[i] = ntohs(points[i]);
            
            check = ntohs(check);
        }
    };

    struct Vanjee718hBlock2
    {
        uint16_t points[440];

        uint16_t check;
        uint8_t tail[2];

        void ToLittleEndian()
        {
            for(int i = 0; i < 440; i++)
                points[i] = ntohs(points[i]);
            
            check = ntohs(check);
        }
    };

    struct Vanjee718hMsopPkt
    {
        uint8_t header[2];
        uint16_t frame_len;
        uint16_t frame_no;
        uint32_t time_stamp;
        uint8_t check_type;
        uint8_t frame_type;
        uint16_t device_type;
        uint8_t remain1[8];
        uint8_t main_cmd;
        uint8_t sub_cmd;
        uint8_t cmd_param1;
        uint8_t cmd_param2;

        uint16_t device_status;
        uint16_t watch_dag_reset_num;
        uint16_t software_reset_num;
        uint16_t loss_elec_reset_num;
        uint16_t usart_pulsewidth;
        uint16_t botboard_usart_threshold;
        uint16_t offset;
        uint16_t codedisc_time_gap;
        uint16_t motor_speed_compare_value;
        uint16_t motor_speed;
        uint16_t topboard_SPAD_voltage;
        uint16_t topboard_temp;
        uint16_t botboard_voltage;
        uint16_t botboard_voltage_compare_value;
        uint16_t dislink_num;
        uint16_t reconnect_num;
        uint16_t disconnect_num;
        uint16_t clog_num;
        uint16_t resend_faild_num;
        uint16_t heartbit_disconnect_num;
        uint16_t keeplive_disconnect_num;
        uint8_t input_IO_value;
        uint8_t remain2;
        uint8_t turn_source;
        uint8_t cur_bank_no;
        uint8_t zone_type;
        uint8_t zone_output_value;

        uint32_t circle_no;
        uint8_t strength_flag;
        uint8_t resolution;
        uint8_t pack_num;
        uint8_t pack_no;
        uint8_t cur_pack_value;
        uint16_t points_num;

        void ToLittleEndian()
        {
            frame_len = ntohs(frame_len);
            frame_no = ntohs(frame_no);
            time_stamp = ntohl(time_stamp);
            device_type = ntohs(device_type);

            device_status = ntohs(device_status);
            watch_dag_reset_num = ntohs(watch_dag_reset_num);
            software_reset_num = ntohs(software_reset_num);
            loss_elec_reset_num = ntohs(loss_elec_reset_num);
            usart_pulsewidth = ntohs(usart_pulsewidth);
            botboard_usart_threshold = ntohs(botboard_usart_threshold);
            offset = ntohs(offset);
            codedisc_time_gap = ntohs(codedisc_time_gap);
            motor_speed_compare_value = ntohs(motor_speed_compare_value);
            motor_speed = ntohs(motor_speed);
            topboard_SPAD_voltage = ntohs(topboard_SPAD_voltage);
            topboard_temp = ntohs(topboard_temp);
            botboard_voltage = ntohs(botboard_voltage);
            botboard_voltage_compare_value = ntohs(botboard_voltage_compare_value);
            dislink_num = ntohs(dislink_num);
            reconnect_num = ntohs(reconnect_num);
            disconnect_num = ntohs(disconnect_num);
            clog_num = ntohs(clog_num);
            resend_faild_num = ntohs(resend_faild_num);
            heartbit_disconnect_num = ntohs(heartbit_disconnect_num);
            keeplive_disconnect_num = ntohs(keeplive_disconnect_num);
            circle_no = ntohl(circle_no);
            points_num = ntohs(points_num);
        }
    };
    #pragma pack(pop)

    struct PointDXYZIRT
    {
        float distance;
        float x;
        float y;
        float z;
        float distance_second;
        float x_second;
        float y_second;
        float z_second;
        float intensity;
        double timestamp;
        int ring;
    };
    

    template <typename T_PointCloud>
    class DecoderVanjee718H : public DecoderMech<T_PointCloud>
    {
    private:
        double light_vec[1440];             // 缓存一圈点云时间差
        double lightangleVal = 0.00000694;  // 相邻通道间隔时间
        bool scan_data_recv_flag = false;
        double pkt_ts = 0;
        int8_t prepack_no;
        int32_t preFramsNo = -1;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment

        std::vector<PointDXYZIRT> points_temp;

        void setPointsTemp(Vanjee718hBlock1& block, uint8_t pkg_no);
        void setPointsTemp(Vanjee718hBlock2& block, uint8_t pkg_no);
        
    public:
        constexpr static double FRAME_DURATION = 0.1;
        constexpr static uint32_t SINGLE_PKT_NUM = 1440;
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee718H() = default;
        explicit DecoderVanjee718H(const WJDecoderParam &param);
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_1(const uint8_t *pkt, size_t size);

    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee718H<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1089 // msop len
                , 1 // laser number
                , 500 // blocks per packet
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
    inline DecoderVanjee718H<T_PointCloud>::DecoderVanjee718H(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
        for(int i = 0; i < 1440; i++)
        {
            light_vec[i] = i * lightangleVal;
        }
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
    {
        return decodeMsopPkt_1(pkt, size);
    }

    template <typename T_PointCloud>
    inline void DecoderVanjee718H<T_PointCloud>::setPointsTemp(Vanjee718hBlock1& block, uint8_t pkg_no)
    {
        for(int i = 0; i < 500; i++)
        {
            int32_t angle = (int32_t)((180 + ((pkg_no - 1) * 500 + i) * 0.25) * 1000) % 360000;
            PointDXYZIRT pointDXYZIRT;
            
            float distance = block.points[i] / 1000.0;
            if (this->param_.start_angle < this->param_.end_angle)
            {
                if (angle < this->param_.start_angle * 1000 || angle > this->param_.end_angle * 1000)
                {
                    distance = 0;
                }
            }
            else
            {
                if (angle > this->param_.end_angle * 1000 && angle < this->param_.start_angle * 1000)
                {
                    distance = 0;
                }
            }

            pointDXYZIRT.distance = distance;
            pointDXYZIRT.x = distance * COS(angle);
            pointDXYZIRT.y = distance * SIN(angle);
            pointDXYZIRT.z = 0.0;
            pointDXYZIRT.distance_second = 0.0;
            pointDXYZIRT.x_second = 0.0;
            pointDXYZIRT.y_second = 0.0;
            pointDXYZIRT.z_second = 0.0;
            pointDXYZIRT.intensity = 0;
            pointDXYZIRT.timestamp = pkt_ts + light_vec[(pkg_no - 1) * 500 + i];
            pointDXYZIRT.ring = 0;
            points_temp.push_back(pointDXYZIRT);

            this->prev_point_ts_ = pointDXYZIRT.timestamp;
        }
        this->prev_pkt_ts_ = pkt_ts + light_vec[(pkg_no - 1) * 500];
    }

    template <typename T_PointCloud>
    inline void DecoderVanjee718H<T_PointCloud>::setPointsTemp(Vanjee718hBlock2& block, uint8_t pkg_no)
    {
        for(int i = 0; i < 440; i++)
        {
            int32_t angle = (int32_t)((180 + (1000 + i) * 0.25) * 1000) % 360000;
            PointDXYZIRT pointDXYZIRT;
            
            float distance = block.points[i] / 1000.0;
            if (this->param_.start_angle < this->param_.end_angle)
            {
                if (angle < this->param_.start_angle * 1000 || angle > this->param_.end_angle * 1000)
                {
                    distance = 0;
                }
            }
            else
            {
                if (angle > this->param_.end_angle * 1000 && angle < this->param_.start_angle * 1000)
                {
                    distance = 0;
                }
            }

            pointDXYZIRT.distance = distance;
            pointDXYZIRT.x = distance * COS(angle);
            pointDXYZIRT.y = distance * SIN(angle);
            pointDXYZIRT.z = 0.0;
            pointDXYZIRT.distance_second = 0.0;
            pointDXYZIRT.x_second = 0.0;
            pointDXYZIRT.y_second = 0.0;
            pointDXYZIRT.z_second = 0.0;
            pointDXYZIRT.intensity = 0;
            pointDXYZIRT.timestamp = pkt_ts + light_vec[1000 + i];
            pointDXYZIRT.ring = 0;
            points_temp.push_back(pointDXYZIRT);

            this->prev_point_ts_ = pointDXYZIRT.timestamp;
        }
        this->prev_pkt_ts_ = pkt_ts + light_vec[1000];
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee718H<T_PointCloud>::decodeMsopPkt_1(const uint8_t *packet, size_t size)
    {
        Vanjee718hMsopPkt& pkt = (*(Vanjee718hMsopPkt *)packet);
        pkt.ToLittleEndian();

        bool ret = false;
        

        int32_t loss_packets_num = (pkt.frame_no + 65536 - preFramsNo) % 65536;
        if(loss_packets_num > 1 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = pkt.frame_no;

        if (this->split_strategy_->newBlock((int64_t)pkt.pack_no) && points_temp.size() == 1440)
        {
            for(int i = 0; i < points_temp.size() ; i++)
            {
                if (this->distance_section_.in(points_temp[i].distance))
                {
                    float x = points_temp[i].x;
                    float y = points_temp[i].y;
                    float z = points_temp[i].z;

                    this->transformPoint(x, y, z);
                    typename T_PointCloud::PointT point;
                    setX(point, x);
                    setY(point, y);
                    setZ(point, z);
                    setIntensity(point, points_temp[i].intensity);
                    setTimestamp(point, points_temp[i].timestamp);
                    setRing(point, 0);
                    this->point_cloud_->points.emplace_back(point);

                    this->scan_data_->ranges.emplace_back(points_temp[i].distance);
                    this->scan_data_->intensities.emplace_back(points_temp[i].intensity);
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
                    setTimestamp(point, points_temp[i].timestamp);
                    setRing(point, 0);

                    this->scan_data_->intensities.emplace_back(0);

                    this->point_cloud_->points.emplace_back(point);
                }
            }

            //scandata struct
            this->scan_data_->angle_min = -180;
            this->scan_data_->angle_max = 180;
            this->scan_data_->angle_increment = 0.25;
            this->scan_data_->time_increment = lightangleVal;
            this->scan_data_->scan_time = pkt_ts;
            this->scan_data_->range_min = this->param_.min_distance;
            this->scan_data_->range_max = this->param_.max_distance;

            this->cb_scan_data_(this->cloudTs());

            this->scan_data_->ranges.clear();
            this->scan_data_->intensities.clear();
            
            //pointcould
            this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;

        }

        if(pkt.pack_no == 1 && pkt.points_num == 500 && (prepack_no = pkt.pack_no - 1 || prepack_no == -1))
        {
            points_temp.clear();

            if(!this->param_.use_lidar_clock)
                pkt_ts = getTimeHost() * 1e-6;
            else
                pkt_ts = pkt.time_stamp;

            Vanjee718hBlock1& block = (*(Vanjee718hBlock1 *)(packet + 85));
            block.ToLittleEndian();
            setPointsTemp(block, pkt.pack_no);
            
            prepack_no = pkt.pack_no;
        }
        else if(pkt.pack_no == 2 && pkt.points_num == 500 && prepack_no == pkt.pack_no - 1)
        {
            Vanjee718hBlock1& block = (*(Vanjee718hBlock1 *)(packet + 85));
            block.ToLittleEndian();

            setPointsTemp(block, pkt.pack_no);
            prepack_no = pkt.pack_no;
        }
        else if(pkt.pack_no == 3 && pkt.points_num == 440 && prepack_no == pkt.pack_no - 1)
        {
            Vanjee718hBlock2& block = (*(Vanjee718hBlock2 *)(packet + 85));
            block.ToLittleEndian();

            setPointsTemp(block, pkt.pack_no);
            prepack_no = 0;
        }
        else if(pkt.pack_no == 4 && pkt.points_num == 500 && prepack_no != -1)
        {
            Vanjee718hBlock1& block = (*(Vanjee718hBlock1 *)(packet + 85));
            block.ToLittleEndian();

            for(int i = 0; i < 500; i++)
                points_temp[i].intensity = block.points[i];
        }
        else if(pkt.pack_no == 5 && pkt.points_num == 500 && prepack_no != -1)
        {
            Vanjee718hBlock1& block = (*(Vanjee718hBlock1 *)(packet + 85));
            block.ToLittleEndian();

            for(int i = 0; i < 500; i++)
                points_temp[i + 500].intensity = block.points[i];
        }
        else if(pkt.pack_no == 6 && pkt.points_num == 440 && prepack_no != -1)
        {
            Vanjee718hBlock2& block = (*(Vanjee718hBlock2 *)(packet + 85));
            block.ToLittleEndian();

            for(int i = 0; i < 440; i++)
                points_temp[i + 1000].intensity = block.points[i];
        }
        else
        {
            prepack_no = -1;
        }

        return ret;
    }

    template<typename T_PointCloud>
    void DecoderVanjee718H<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
        std::shared_ptr<ProtocolAbstract718H> p;
        std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd, protocol->SubCmd);
        
        if(*sp_cmd == *(CmdRepository718H::CreateInstance()->Sp_ScanDataGet))
        {
            p = std::make_shared<Protocol_ScanDataGet718H>();
        }
        else
        {
            return;
        }
        p->Load(*protocol);

        std::shared_ptr<ParamsAbstract> params = p->Params;
        if(typeid(*params) == typeid(Params_ScanData718H))
        {
            std::shared_ptr<Params_ScanData718H> param = std::dynamic_pointer_cast<Params_ScanData718H>(params);
            if (param->data_get_flag && !scan_data_recv_flag)
            {
                WJ_INFOL << "get wlr718h scan data succ !" << WJ_REND;
                (*(Decoder<T_PointCloud>::get_difo_ctrl_map_ptr_))[sp_cmd->GetCmdKey()].setStopFlag(true);
                scan_data_recv_flag = true;
            }
            else if(!param->data_get_flag)
            {
                WJ_INFOL << "wlr718h device status err !" << WJ_REND;
            }
        }
        else
        {
            WJ_WARNING << "Unknown Params Type..." << WJ_REND;
        }
    }

}   // namespace lidar
}   // namespace vanjee