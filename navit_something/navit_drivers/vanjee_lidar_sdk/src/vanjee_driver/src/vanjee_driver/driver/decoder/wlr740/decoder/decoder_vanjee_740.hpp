/*
 * @Author: Guo Xuemei 1242143575@qq.com
 * @Date: 2023-02-08 15:32:17
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-04-06 11:08:23
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/decoder/decoder_vanjee740.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AEnam
 */
#pragma once 

#include <Eigen/Dense>

#include <vanjee_driver/driver/decoder/decoder_mech.hpp>
#include <vanjee_driver/driver/difop/protocol_base.hpp>
#include <vanjee_driver/driver/difop/cmd_class.hpp>
#include <vanjee_driver/driver/difop/protocol_abstract.hpp>
#include <vanjee_driver/driver/decoder/wlr740/protocol/frame/cmd_repository_740.hpp>

#include <vanjee_driver/driver/decoder/wlr740/decoder/params_scan_data.hpp>
#include <vanjee_driver/driver/decoder/wlr740/decoder/one_point_info_manage.hpp>
#include <vanjee_driver/driver/decoder/wlr740/algorithm/filterate.hpp>

#define POINTS_COMPENSE false

constexpr unsigned int ONE_CIRCLE_CLOUDS = (144*3600);

namespace vanjee
{
namespace lidar
{
  #pragma pack(push,1)

  typedef struct  
  {
    uint8 time_flag[2];
    uint32 microsecond;
    uint32 second;
    uint8 reserved[2];
  }NtpTime;

  typedef struct
  {
    uint16 distance;
    uint8 intensity;
  }ChannelData0728;

  typedef struct
  {
    uint16 distance;
    uint8 intensity;
    uint16 low_thres_pulse_width;
    uint16 high_thres_pulse_width;
  }ChannelData0728WithPulseWidth;

  typedef struct 
  {
    uint8 header;
    uint8 rotate_mirror_id;
    uint16 horizontal_angle;
    uint8 horizontal_angle_res;
    uint8 timing_thres_type;
    uint8 echo_type;
    int8 vertical_angle_compensation;
    int32 horizontal_angle_compensation;
    ChannelData0728 channel_data[48];
  }DataBlock0728;

  typedef struct 
  {
    uint8 header;
    uint8 rotate_mirror_id;
    uint16 horizontal_angle;
    uint8 horizontal_angle_res;
    uint8 timing_thres_type;
    uint8 echo_type;
    int8 vertical_angle_compensation;
    int32 horizontal_angle_compensation;
    ChannelData0728WithPulseWidth channel_data[48];
    uint8 strong_weak_flag[6];
  }DataBlock0728WithPulseWidth;
  
  typedef struct 
  {
    uint8 header[2];
    uint16 frame_len;
    uint8 device_type[2];
    uint8 frame_type[2];
    union{
      uint8 time_info[12];
      NtpTime ntp_time;
    }time_field;
    uint16 frame_index;
    uint8 protocol_version[4];
    uint16 horizontal_angle_offset;
  }Vanjee740Difop;

  typedef struct 
  {
    Vanjee740Difop difop;
    DataBlock0728 data_block[8];
  }Vanjee740MsopPkt0728;

  typedef struct 
  {
    Vanjee740Difop difop;
    DataBlock0728WithPulseWidth data_block[4];
  }Vanjee740MsopPkt0728WithPulseWidth;

  #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee740 :public DecoderMech<T_PointCloud>
    {
    private:
        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam& getConstParam(uint8_t mode);
        static WJEchoMode getEchoMode(uint8_t mode);
        OnePointInfoManage* m_qMapOneCirclePoint = new OnePointInfoManage();
        OnePointInfoManage* m_qMapPcapOneCirclePoint = new OnePointInfoManage();
        bool AlgorithmEnb = false;
        bool BlankPointCompenEnb = false;
        bool OutlierEnb = false;
        bool CloseRangeOutliersEnb = false;

        int32_t preFramsNo = -1;

        int HangleLumComp[48] = {1,3,5,7,2,4,6,8, 
                                        1,3,5,7,2,4,6,8,
                                        1,3,5,7,2,4,6,8,
                                        1,3,5,7,2,4,6,8,
                                        1,3,5,7,2,4,6,8,
                                        1,3,5,7,2,4,6,8};

        int Ev = 600; // 600转 / 分
        double _1_6us_HAngle = ((360.0 / ((1000.0 / (Ev / 60.0)) * 1000.0)) * 1.6 * 2) * 1000; // 1.6us转多少角度 精度 0.001 * 1000

        bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
        double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）

    public:
        constexpr static double FRAME_DURATION = 0.1;
        constexpr static uint32_t SINGLE_PKT_NUM = 450;

        virtual bool decodeMsopPkt(const uint8_t* pkt, size_t size);
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee740() = default;
        explicit DecoderVanjee740(const WJDecoderParam& param);

        inline uint32 GetOnePointInfoKey(const OnePointInfo& val);
        
        bool decodeMsopPkt_0728(const uint8_t* pkt, size_t size);
        bool decodeMsopPkt_0728Reflectivity(const uint8_t* pkt, size_t size);
    };

    template <typename T_PointCloud>
    inline WJDecoderMechConstParam& DecoderVanjee740<T_PointCloud>::getConstParam(uint8_t mode)
    {
        switch (mode)
        {
        case 1:
        {
            static WJDecoderMechConstParam param =
                {
                    1324 
                    ,48 
                    ,4 
                    ,144 
                    ,0.5f 
                    ,210.0f 
                    ,0.004f 
                    ,80.0f 
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
        }
        break;

        case 2:
        {
            static WJDecoderMechConstParam param =
                {
                    1064 
                    ,48 
                    ,4 
                    ,48 
                    ,0.2f 
                    ,200.0f 
                    ,0.004f 
                    ,80.0f 
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
        }
        break;
        }
        static WJDecoderMechConstParam param =
                {
                    1324 
                    ,48 
                    ,4 
                    ,144 
                    ,0.2f 
                    ,200.0f 
                    ,0.004f 
                    ,80.0f 
                };
            param.BLOCK_DURATION = 0.1 / 360;
            return param;
    }
    
    template <typename T_PointCloud> 
    inline WJEchoMode DecoderVanjee740<T_PointCloud>::getEchoMode(uint8_t mode)
    {
        switch (mode)
        {
        case 0x10:    
            return WJEchoMode::ECHO_DUAL;
        case 0x20:
        case 0x30:
        
        default:
            return WJEchoMode::ECHO_SINGLE;
        }
    }
    
    template <typename T_PointCloud>
    inline DecoderVanjee740<T_PointCloud>::DecoderVanjee740(const WJDecoderParam &param)
    : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param)
    {
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByAngle>(0);

        if(this->param_.config_from_file)
        {
            int ret_angle = this->chan_angles_.loadFromFile(this->param_.angle_path_ver, this->param_.angle_path_hor);
            this->angles_ready_ = (ret_angle == 0);
        }

        AlgorithmEnb = this->param_.AlgorithmEnb;
        BlankPointCompenEnb = this->param_.BlankPointCompenEnb;
        OutlierEnb = this->param_.OutlierEnb;
        CloseRangeOutliersEnb = this->param_.CloseRangeOutliersEnb;
        // WJ_INFOL << "740AlgorithmEnb: " << AlgorithmEnb << WJ_REND;
        // WJ_INFOL << "BlankPointCompenEnb: " << BlankPointCompenEnb << WJ_REND;
        // WJ_INFOL << "OutlierEnb: " << OutlierEnb << WJ_REND;
        // WJ_INFOL << "CloseRangeOutliersEnb: " << CloseRangeOutliersEnb << WJ_REND;
        
        Filterate::GetInstance()->SetAlgorithmEnb(AlgorithmEnb);
        Filterate::GetInstance()->SetBlankPointCompenEnb(BlankPointCompenEnb);
        Filterate::GetInstance()->SetOutlierEnb(OutlierEnb);
        Filterate::GetInstance()->SetCloseRangeOutliersEnb(CloseRangeOutliersEnb);
        
        m_qMapOneCirclePoint->Init(144);
        m_qMapOneCirclePoint->Clear();
        m_qMapPcapOneCirclePoint->Init(144);
        m_qMapPcapOneCirclePoint->Clear();
    }

    template <typename T_PointCloud>
    inline uint32 DecoderVanjee740<T_PointCloud>::GetOnePointInfoKey(const OnePointInfo& val)
    {
        if(val.EchoType == 1)
        {
            uint32 index = ((val.line - 1) * 3600) + val.HAngle;
            return index;
        }
        else if(val.EchoType == 2)
        {
            return ((val.line - 1) * 3600) + val.HAngle + ONE_CIRCLE_CLOUDS;
        }
        else
        {
            //qDebug() << "异常";
            return 0;
        }
    }
    
    template <typename T_PointCloud>
    inline bool DecoderVanjee740<T_PointCloud>::decodeMsopPkt(const uint8_t* pkt, size_t size)
    {
        static const int ScanData0728Size = sizeof(Params_ScanData0728::ScanData);
        static const int ScanData0728ReflectivitySize = sizeof(Params_ScanData0728Reflectivity::ScanData);
        switch(pkt[6])
        {
            case 0x02:
            {
                switch(size)
                {
                    case ScanData0728Size:
                    {
                        return decodeMsopPkt_0728(pkt , size);
                    }
                    break;
                }         
            }
            break;

            case 0x16:
            {
                switch(size)
                {
                    case ScanData0728ReflectivitySize:
                    {
                        return decodeMsopPkt_0728Reflectivity(pkt , size);
                    }
                    break;
                }
            }
            break;

        }
        return false;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee740<T_PointCloud>::decodeMsopPkt_0728(const uint8_t* pkt, size_t size)
    {
        bool ret = false; 
        double pkt_ts = 0;
        static int PreFrameNo = 0;
        int oneCircleDataAmount = 0;
        static int CuCircleFrameNumPublish = 0;
        static bool compulsionPublish = false;
        static int preHAngle = 0;
        static int preResolutionRatio = 0;

        const Vanjee740MsopPkt0728& packet = *((Vanjee740MsopPkt0728*)pkt);

        int32_t loss_packets_num = (packet.difop.frame_index + 65536 - preFramsNo) % 65536;
        if(loss_packets_num > 100 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = packet.difop.frame_index;

        double sec = packet.difop.time_field.ntp_time.second;
        double usec = packet.difop.time_field.ntp_time.microsecond;
        pkt_ts = sec + (usec * 1e-6);
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

        if(packet.data_block[0].echo_type != packet.data_block[1].echo_type)
        {
            //oneCircleDataAmount = (1200 * 3) / 4;
            oneCircleDataAmount = 900;
        }
        else
        {
            //oneCircleDataAmount = (1200 * 3) / 8;
            oneCircleDataAmount = 450;
        }

        unsigned short PacketNumdiff = packet.difop.frame_index - PreFrameNo; // 和上一帧序号差

        if(PacketNumdiff == 1)
        {
            CuCircleFrameNumPublish++;
        }
        else
        {
            CuCircleFrameNumPublish += (PacketNumdiff - 1);
        }

        if(CuCircleFrameNumPublish > oneCircleDataAmount)
        {
            compulsionPublish = true;
        }
        PreFrameNo = packet.difop.frame_index;
        // if (this->param_.use_lidar_clock)
        // {
        //     pkt_ts = parseTimeUTCWithUs_BigEndian(&packet.Data.CreateTime) * 1e-6;
        // }
        // else
        // {
        //     uint64_t ts = getTimeHost();

        //     pkt_ts = getTimeHost() * 1e-6 - this->getPacketDuration();

        //     if (this->write_pkt_ts_)
        //     {
        //         createTimeYMD (ts, (WJTimestampYMD*)&packet.Data.CreateTime);
        //     }
        //     this->prev_pkt_ts_ = pkt_ts;
        // }

        for(int i = 0; i < 8;i++)
        {
            // 转镜号
            unsigned char RotatingMirror = packet.data_block[i].rotate_mirror_id;
            // 一个数据块 1个水平角度
            short HAngle = packet.data_block[i].horizontal_angle - 1; // 精度0.1
            // 一个数据块 1个垂直方向补偿角
            int VCompensateAngle = packet.data_block[i].vertical_angle_compensation;
            // 一个数据块 1个水平方向补偿角
            int HCompensateAngle = packet.data_block[i].horizontal_angle_compensation;
            // 获取水平角度分辨率
            unsigned char ResolutionRatio = packet.data_block[i].horizontal_angle_res;
            // 获取回波类型
            unsigned char EchoType = packet.data_block[i].echo_type;

            if(packet.data_block[0].echo_type != packet.data_block[1].echo_type &&
                Decoder<T_PointCloud>::param_.publish_mode != 2 &&
                Decoder<T_PointCloud>::param_.publish_mode+1 != EchoType)
              continue;
           
            // 跳过异常数据
            if((RotatingMirror > 3) || (HAngle > 3600) || (ResolutionRatio <=0) || (ResolutionRatio > 2))
            {
                continue;
            }

            if(preResolutionRatio != ResolutionRatio)
            {
                Filterate::GetInstance()->SetHResolution(ResolutionRatio);
                if(ResolutionRatio == 1)
                {
                    Ev = 600; // 600转 / 分
                    _1_6us_HAngle = ((360.0 / ((1000.0 / (Ev / 60.0)) * 1000.0)) * 1.6 * 2) * 1000;
                     
                     if(Filterate::GetInstance()->GetStartAngle() != 1)
                    {
                        Filterate::GetInstance()->SetStartAngle(1);
                    }
                }
                else
                {
                    Ev = 1200; // 1200转 / 分
                    _1_6us_HAngle = ((360.0 / ((1000.0 / (Ev / 60.0)) * 1000.0)) * 1.6 * 2) * 1000;
                
                    if(HAngle % 2 == 1)
                    {
                        if(Filterate::GetInstance()->GetStartAngle() != 1)
                        {
                            Filterate::GetInstance()->SetStartAngle(1);
                        }
                    }
                    else
                    {
                        if(Filterate::GetInstance()->GetStartAngle() != 0)
                        {
                            Filterate::GetInstance()->SetStartAngle(0);
                        }
                    }
                }               
            }

            if((HAngle < preHAngle && RotatingMirror == 0) || compulsionPublish)
            {

                if(m_qMapOneCirclePoint->indexRecordsize < 4000)
                {
                    m_qMapOneCirclePoint->Clear();
                }

                if(AlgorithmEnb)
                {
                    // 发布前 走补点算法
                    int mode = 1;
                    if(packet.data_block[0].echo_type != packet.data_block[1].echo_type)
                    {
                      mode  = Decoder<T_PointCloud>::param_.publish_mode + 1;
                    }
                    else
                    {
                      mode = packet.data_block[0].echo_type;
                    }
                    Filterate::GetInstance()->filter(*m_qMapOneCirclePoint, *m_qMapPcapOneCirclePoint,mode);
                }

                OnePointInfoManage* tmp  = nullptr;
                tmp = m_qMapPcapOneCirclePoint;
                m_qMapPcapOneCirclePoint = m_qMapOneCirclePoint;
                m_qMapOneCirclePoint = tmp;
                tmp = nullptr;

                // int MaxIntensityFlag = 0;
                // int MinIntensityFlag = 0;

                // for(int i = 0; i < m_qMapOneCirclePoint.indexRecordsize;i++)
                // {
                //     OnePointInfo& tem = m_qMapOneCirclePoint.GetDataIt(i);
                //     if(MaxIntensityVal < tem.intensity /*&& tem.distance*/ > 0)
                //     {
                //         MaxIntensityVal = tem.intensity;
                //     }

                //     if(MinIntensityVal > tem.intensity /*&& tem.distance*/ > 0)
                //     {
                //         MinIntensityVal = tem.intensity;
                //     }
                // }


                // 将m_qMapOneCirclePoint数据转为ros发布结构体
                for(int i = 0; i < m_qMapOneCirclePoint->indexRecordsize;i++)
                {
                    OnePointInfo& tem = m_qMapOneCirclePoint->GetDataIt(i);

                    if(OutlierEnb)
                    {
                        if(tem.NonFilterable == 0 && (tem.distance > Filterate::GetInstance()->GetFullSearchScope()))
                        {
                             tem.x = 0;
                             tem.y = 0;
                             tem.z = 0;
                             tem.distance = 0;
                            //continue; // 不知道为啥 这个程序架构不允许 continue 否则发布点云的时候会报错
                        }
                            
                    }
                    if(CloseRangeOutliersEnb)
                    {
                        if(tem.CloseRangeOutliers == 0 && (tem.distance < Filterate::GetInstance()->GetFullSearchScope()))
                        {
                             tem.x = 0;
                             tem.y = 0;
                             tem.z = 0;
                             tem.distance = 0;
                            //continue;
                        }
                    }                    

                    typename T_PointCloud::PointT point;
                    setX(point, tem.x);
                    setY(point, tem.y);
                    setZ(point, tem.z);

                    // // 如果开启了算法 可能会失效
                    // if((MaxIntensityVal != 0) && (MaxIntensityVal == tem.intensity) && (MaxIntensityFlag == 0) /*&& tem.distance > 0*/)
                    // {
                    //     tem.intensity = 255;
                    //     MaxIntensityFlag = 1;
                    // }
                    // // 如果开启了算法 可能会失效
                    // if((MinIntensityVal != 0) && (MinIntensityVal == tem.intensity) && (MinIntensityFlag == 0) /*&& tem.distance > 0*/ )
                    // {
                    //     tem.intensity = 0;
                    //     MinIntensityFlag = 1;
                    // }
                    

                    setIntensity(point, tem.intensity);
                    //setTimestamp(point, this->prev_pkt_ts_);
                    setTimestamp(point, tem.Time / 1000.0);
                    setRing(point, tem.line);
                   
                    this->point_cloud_->points.emplace_back(std::move(point));
                }

                compulsionPublish = false;
                CuCircleFrameNumPublish = 0;
                this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
                this->first_point_ts_ = pkt_ts;

                m_qMapOneCirclePoint->Clear();
                ret = true;
            }
            preHAngle = HAngle;

            for(int j = 0; j < 48;j++)
            {
                int l_line = j+1;
                int l_LineNo;
                int temHAngle = HAngle; // 由于在for循环中 17-32线需要+38 因此增加临时变量 获取当前水平角度
                int temZeroHAngle = HAngle; // 从0开始算的水平角度
                float distance = packet.data_block[i].channel_data[j].distance << 2;
                distance/=1000;

                // int l_i = j % 8;
                // int l_j = j / 8;
                // int StrongWeak =  (packet.Data.dataBlock[i].StrongWeak[l_j] & (0x01 << l_i)) > 0?1:2;

                // if(StrongWeak == 1)
                // {
                //     // 4.0米内禁止出强光
                //     if(distance <= 4.0) 
                //     {
                //         distance = 0;
                //     }
                // }

                if(l_line >16 && l_line <= 32)
                {
                    temHAngle += 38; // 3.8度的偏移
                }
                // 水平发光时序补偿
                int HLumTimingComp =  (((HangleLumComp[j] - 1) * _1_6us_HAngle));
                double points_time = pkt_ts + ((HangleLumComp[j] - 1) * 1.6) * 1e-6 ;

                temHAngle = (int)(((temHAngle*100 + HCompensateAngle ) + HLumTimingComp )+ 360000)%360000;//获取水平角 精度0.01


                switch(RotatingMirror)
                {
                case 0:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 2;
                    break;
                case 1:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 0;
                    break;
                case 2:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 1;
                    break;
                }

                int l_iomdex = this->chan_angles_.vertAdjust(j);
                l_iomdex = this->chan_angles_.horiz_vertAdjust(l_LineNo - 1, HAngle);
                l_iomdex = (int)(l_iomdex + 360000) % 360000;

                int32_t azimuth_index = temHAngle;
                int32_t verticalVal_740 = l_iomdex;

                float xy,x,y,z;


                xy = distance * COS(verticalVal_740);
                x = xy * (COS(azimuth_index));
                y = xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_740);
                this->transformPoint(x, y, z);

                OnePointInfo onePointInfoTmp;
                onePointInfoTmp.Time = points_time;
                onePointInfoTmp.Mirror = RotatingMirror;
                onePointInfoTmp.Channal = l_line;
                // 重新映射ros反射率颜色 将 255 -> 0 , 0 -> 255  ,255 的最大值修改至 0.75 目的是 跳过ros紫色区域
                // 255 - 0 颜色依次为 深蓝-浅蓝-绿色-黄色-红色-紫色 -因此将255再次映射到145 防止出现紫色
                onePointInfoTmp.intensity = (255-packet.data_block[i].channel_data[j].intensity) * 0.75;
                onePointInfoTmp.LowPulseWidth = 0;
                onePointInfoTmp.TallPulseWidth = 0;
                onePointInfoTmp.old_distance = distance * 1000;
                onePointInfoTmp.distance = distance * 1000;
                onePointInfoTmp.azimuth = temHAngle / 10; // 精度0.01
                onePointInfoTmp.vazimuth = l_iomdex;
                onePointInfoTmp.HAngle = packet.data_block[i].horizontal_angle;
                onePointInfoTmp.HAngle = (int)(onePointInfoTmp.HAngle + 3600)%3600;//获取水平角 精度0.01
                onePointInfoTmp.VAngle = this->chan_angles_.horiz_vertAdjust(l_LineNo - 1, HAngle);//Cfg.opticalPathAngle_Line[l_LineNo - 1].VAngle;
                onePointInfoTmp.EchoType = EchoType; // 1- 最强 2-最后 // 主要用于显示在表中
                onePointInfoTmp.StrongWeak = 0;
                onePointInfoTmp.y = y;//新坐标系计算方式，逆时针改顺时针的方式
                onePointInfoTmp.x = x;
                onePointInfoTmp.z = z;//不补偿
                onePointInfoTmp.line = l_LineNo;//3*(l_line-1)+RotatingMirror+1;

                onePointInfoTmp.index = GetOnePointInfoKey(onePointInfoTmp);
                m_qMapOneCirclePoint->insert(onePointInfoTmp.index,std::move(onePointInfoTmp));
                
            }
        }       
        return ret;        
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee740<T_PointCloud>::decodeMsopPkt_0728Reflectivity(const uint8_t* pkt, size_t size)
    {
        bool ret = false; 
        double pkt_ts = 0;
        static int PreFrameNo = 0;
        int oneCircleDataAmount = 0;
        static int CuCircleFrameNumPublish = 0;
        static bool compulsionPublish = false;
        static int preHAngle = 0;
        static int preResolutionRatio = 0;

        const Vanjee740MsopPkt0728WithPulseWidth& packet = *(Vanjee740MsopPkt0728WithPulseWidth*)pkt;

        int32_t loss_packets_num = (packet.difop.frame_index + 65536 - preFramsNo) % 65536;
        if(loss_packets_num > 100 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = packet.difop.frame_index;

        double sec = packet.difop.time_field.ntp_time.second;
        double usec = packet.difop.time_field.ntp_time.microsecond;
        pkt_ts = sec + (usec * 1e-6);
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

        if(packet.data_block[0].echo_type != packet.data_block[1].echo_type)
        {
            //oneCircleDataAmount = (1200 * 3) / 2;
            oneCircleDataAmount = 1800;
        }
        else
        {
            //oneCircleDataAmount = (1200 * 3) / 4;
            oneCircleDataAmount = 900;
        }

        unsigned short PacketNumdiff = packet.difop.frame_index - PreFrameNo; // 和上一帧序号差
        if(PacketNumdiff == 1)
        {
            CuCircleFrameNumPublish++;
        }
        else
        {
            CuCircleFrameNumPublish += (PacketNumdiff - 1);
        }

        if(CuCircleFrameNumPublish > oneCircleDataAmount)
        {
            compulsionPublish = true;
        }
        PreFrameNo = packet.difop.frame_index;

        for(int i = 0; i < 4;i++)
        {
            // 转镜号
            unsigned char RotatingMirror = packet.data_block[i].rotate_mirror_id;
            // 一个数据块 1个水平角度
            short HAngle = packet.data_block[i].horizontal_angle - 1; // 精度0.1
            // 一个数据块 1个垂直方向补偿角
            int VCompensateAngle = packet.data_block[i].vertical_angle_compensation;
            // 一个数据块 1个水平方向补偿角
            int HCompensateAngle = packet.data_block[i].horizontal_angle_compensation;
            // 获取水平角度分辨率
            unsigned char ResolutionRatio = packet.data_block[i].horizontal_angle_res;
            // 获取回波类型
            unsigned char EchoType = packet.data_block[i].echo_type;

            if(packet.data_block[0].echo_type != packet.data_block[1].echo_type &&
                Decoder<T_PointCloud>::param_.publish_mode != 2 &&
                Decoder<T_PointCloud>::param_.publish_mode+1 != EchoType)
              continue;
            
            // 跳过异常数据
            if((RotatingMirror > 3) || (HAngle > 3600) || (ResolutionRatio <=0) || (ResolutionRatio > 2))
            {
                continue;
            }

            if(preResolutionRatio != ResolutionRatio)
            {
                Filterate::GetInstance()->SetHResolution(ResolutionRatio);
                if(ResolutionRatio == 1)
                {
                    Ev = 600; // 600转 / 分
                    _1_6us_HAngle = ((360.0 / ((1000.0 / (Ev / 60.0)) * 1000.0)) * 1.6 * 2) * 1000;
                     
                     if(Filterate::GetInstance()->GetStartAngle() != 1)
                    {
                        Filterate::GetInstance()->SetStartAngle(1);
                    }
                }
                else
                {
                    Ev = 1200; // 1200转 / 分
                    _1_6us_HAngle = ((360.0 / ((1000.0 / (Ev / 60.0)) * 1000.0)) * 1.6 * 2) * 1000;
                
                    if(HAngle % 2 == 1)
                    {
                        if(Filterate::GetInstance()->GetStartAngle() != 1)
                        {
                            Filterate::GetInstance()->SetStartAngle(1);
                        }
                    }
                    else
                    {
                        if(Filterate::GetInstance()->GetStartAngle() != 0)
                        {
                            Filterate::GetInstance()->SetStartAngle(0);
                        }
                    }
                }               
            }

            if((HAngle < preHAngle && RotatingMirror == 0) || compulsionPublish)
            {

                if(m_qMapOneCirclePoint->indexRecordsize < 4000)
                {
                    m_qMapOneCirclePoint->Clear();
                }

                if(AlgorithmEnb)
                {
                    // 发布前 走补点算法
                    int mode = 1;
                    if(packet.data_block[0].echo_type != packet.data_block[1].echo_type)
                    {
                      mode  = Decoder<T_PointCloud>::param_.publish_mode + 1;
                    }
                    else
                    {
                      mode = packet.data_block[0].echo_type;
                    }
                    Filterate::GetInstance()->filter(*m_qMapOneCirclePoint, *m_qMapPcapOneCirclePoint,mode);
                }

                OnePointInfoManage* tmp = nullptr;
                tmp = m_qMapPcapOneCirclePoint;
                m_qMapPcapOneCirclePoint = m_qMapOneCirclePoint;
                m_qMapOneCirclePoint = tmp;
                tmp = nullptr;

                // int MaxIntensityFlag = 0;
                // int MinIntensityFlag = 0;

                // for(int i = 0; i < m_qMapOneCirclePoint.indexRecordsize;i++)
                // {
                //     OnePointInfo& tem = m_qMapOneCirclePoint.GetDataIt(i);
                //     if(MaxIntensityVal < tem.intensity /*&& tem.distance*/ > 0)
                //     {
                //         MaxIntensityVal = tem.intensity;
                //     }

                //     if(MinIntensityVal > tem.intensity /*&& tem.distance*/ > 0)
                //     {
                //         MinIntensityVal = tem.intensity;
                //     }
                // }

                // 将m_qMapOneCirclePoint数据转为ros发布结构体
                for(int i = 0; i < m_qMapOneCirclePoint->indexRecordsize;i++)
                {
                    OnePointInfo& tem = m_qMapOneCirclePoint->GetDataIt(i);

                    if(OutlierEnb)
                    {
                        if(tem.NonFilterable == 0 && (tem.distance > Filterate::GetInstance()->GetFullSearchScope()))
                        {
                             tem.x = 0;
                             tem.y = 0;
                             tem.z = 0;
                             tem.distance = 0;
                            //continue; // 不知道为啥 这个程序架构不允许 continue 否则发布点云的时候会报错
                        }                            
                    }
                    if(CloseRangeOutliersEnb)
                    {
                        if(tem.CloseRangeOutliers == 0 && (tem.distance < Filterate::GetInstance()->GetFullSearchScope()))
                        {
                             tem.x = 0;
                             tem.y = 0;
                             tem.z = 0;
                             tem.distance = 0;
                            //continue;
                        }
                    }
                   
                    typename T_PointCloud::PointT point;
                    setX(point, tem.x);
                    setY(point, tem.y);
                    setZ(point, tem.z);

                    // // 如果开启了算法 可能会失效
                    // if((MaxIntensityVal != 0) && (MaxIntensityVal == tem.intensity) && (MaxIntensityFlag == 0) /*&& tem.distance > 0*/)
                    // {
                    //     tem.intensity = 255;
                    //     MaxIntensityFlag = 1;
                    // }
                    // // 如果开启了算法 可能会失效
                    // if((MinIntensityVal != 0) && (MinIntensityVal == tem.intensity) && (MinIntensityFlag == 0) /*&& tem.distance > 0*/ )
                    // {
                    //     tem.intensity = 0;
                    //     MinIntensityFlag = 1;
                    // }
                    
                    setIntensity(point, tem.intensity);
                    setTimestamp(point, tem.Time);
                    setRing(point, tem.line);
                    
                    this->point_cloud_->points.emplace_back(std::move(point));
                    this->prev_point_ts_ = tem.Time;
                }

                compulsionPublish = false;
                CuCircleFrameNumPublish = 0;
                this->cb_split_frame_(this->const_param_.LASER_NUM, this->cloudTs());
                this->first_point_ts_ = pkt_ts;

                m_qMapOneCirclePoint->Clear();
                ret = true;
            }
            preHAngle = HAngle;

            for(int j = 0; j < 48;j++)
            {
                int l_line = j+1;
                int l_LineNo;
                int temHAngle = HAngle; // 由于在for循环中 17-32线需要+38 因此增加临时变量 获取当前水平角度
                int temZeroHAngle = HAngle; // 从0开始算的水平角度
                float distance = packet.data_block[i].channel_data[j].distance << 2;
                distance/=1000;

                int l_i = j % 8;
                int l_j = j / 8;
                int StrongWeak =  (packet.data_block[i].strong_weak_flag[l_j] & (0x01 << l_i)) > 0?1:2;

                if(StrongWeak == 1)
                {
                    // 4.0米内禁止出强光
                    if(distance <= 4.0) 
                    {
                        distance = 0;
                    }
                }

                if(l_line >16 && l_line <= 32)
                {
                    temHAngle += 38; // 3.8度的偏移
                }
                // 水平发光时序补偿
                int HLumTimingComp =  (((HangleLumComp[j] - 1) * _1_6us_HAngle));
                double points_time = pkt_ts + ((HangleLumComp[j] - 1) * 1.6) * 1e-6 ;

                temHAngle = (int)(((temHAngle*100 + HCompensateAngle ) + HLumTimingComp )+ 360000)%360000;//获取水平角 精度0.01

                switch(RotatingMirror)
                {
                case 0:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 2;
                    break;
                case 1:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 0;
                    break;
                case 2:
                    l_LineNo = (l_line - 1)*3+RotatingMirror + 1;
                    break;
                }

                //int l_iomdex = this->chan_angles_.vertAdjust(j);
                int l_iomdex = this->chan_angles_.horiz_vertAdjust(l_LineNo - 1, HAngle);
                l_iomdex = (int)(l_iomdex + 360000) % 360000;

                int32_t azimuth_index = temHAngle;
                int32_t verticalVal_740 = l_iomdex;
                //WJ_INFO << "l_LineNo" << (l_LineNo-1) << "    HAngle" << HAngle << "    vangle:" << verticalVal_740 << WJ_REND;
                float xy,x,y,z;


                xy = distance * COS(verticalVal_740);
                x = xy * (COS(azimuth_index));
                y = xy * SIN(azimuth_index);
                z = distance * SIN(verticalVal_740);
                this->transformPoint(x, y, z);

                OnePointInfo onePointInfoTmp;
                onePointInfoTmp.Time = points_time;
                onePointInfoTmp.Mirror = RotatingMirror;
                onePointInfoTmp.Channal = l_line;
                // 重新映射ros反射率颜色 将 255 -> 0 , 0 -> 255  ,255 的最大值修改至 0.75 目的是 跳过ros紫色区域
                // 255 - 0 颜色依次为 深蓝-浅蓝-绿色-黄色-红色-紫色 -因此将255再次映射到145 防止出现紫色
                onePointInfoTmp.intensity = (255-packet.data_block[i].channel_data[j].intensity) * 0.75;
                onePointInfoTmp.LowPulseWidth = packet.data_block[i].channel_data[j].low_thres_pulse_width;
                onePointInfoTmp.TallPulseWidth = packet.data_block[i].channel_data[j].high_thres_pulse_width;
                onePointInfoTmp.old_distance = distance * 1000;
                onePointInfoTmp.distance = distance * 1000;
                onePointInfoTmp.azimuth = temHAngle / 10; // 精度0.01
                onePointInfoTmp.vazimuth = l_iomdex;
                onePointInfoTmp.HAngle = packet.data_block[i].horizontal_angle;
                onePointInfoTmp.HAngle = (int)(onePointInfoTmp.HAngle + 3600)%3600;//获取水平角 精度0.01
                onePointInfoTmp.VAngle = this->chan_angles_.horiz_vertAdjust(l_LineNo - 1, HAngle);//Cfg.opticalPathAngle_Line[l_LineNo - 1].VAngle;
                onePointInfoTmp.EchoType = EchoType; // 1- 最强 2-最后 // 主要用于显示在表中
                onePointInfoTmp.StrongWeak = StrongWeak;
                onePointInfoTmp.y = y;//新坐标系计算方式，逆时针改顺时针的方式
                onePointInfoTmp.x = x;
                onePointInfoTmp.z = z;//不补偿
                onePointInfoTmp.line = l_LineNo;//3*(l_line-1)+RotatingMirror+1;

                onePointInfoTmp.index = GetOnePointInfoKey(onePointInfoTmp);
                m_qMapOneCirclePoint->insert(onePointInfoTmp.index,std::move(onePointInfoTmp));               
            }
        }       
        return ret;       
    }

    template <typename T_PointCloud>
    void DecoderVanjee740<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
      std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd,protocol->SubCmd);
      std::shared_ptr<ProtocolAbstract> p;

      if(p.get() == nullptr)
        return;

    }

} 
   
} 
