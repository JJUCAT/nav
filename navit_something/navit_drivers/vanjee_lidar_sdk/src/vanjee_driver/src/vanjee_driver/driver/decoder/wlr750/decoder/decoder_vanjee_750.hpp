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
#include <vanjee_driver/driver/decoder/wlr750/protocol/frames/cmd_repository_750.hpp>

namespace vanjee
{
namespace lidar
{
    #pragma pack(push, 1)
   
    typedef struct 
    {
        int16_t x;   ///< 点云x坐标值
        int16_t y;   ///< 点云y坐标值
        int16_t z;   ///< 点云z坐标值
        uint16_t peak_value1;           ///< 峰值1
        uint16_t peak_value2;           ///< 峰值2
        uint16_t use_peak_value_index;  ///< 峰值索引
        uint8_t data_flag;              ///< 携带信息flag
        uint8_t mode;                   ///< 
        uint16_t pixel_num;             ///< 像素序号
        uint16_t group_num;             ///< 组序号


        // int16_t x;   ///< 点云x坐标值
        // int16_t y;   ///< 点云y坐标值
        // int16_t z;   ///< 点云z坐标值
        // uint16_t cur_peak_value;        ///< 当前使用峰值
        // uint32_t cur_pulsewidth;        ///< 当前使用脉宽
        // uint8_t data_flag;              ///< 携带信息flag
        // uint8_t mode;                   ///< 
        // uint16_t pixel_num;             ///< 像素序号
        // uint16_t group_num;             ///< 组序号
        // uint8_t front_edg_value1[3];    ///< 前沿值1
        // uint8_t pulsewidth1[3];         ///< 脉宽1
        // uint16_t peak_value1;           ///< 峰值1
        // uint8_t echo_num1;              ///< 回波重数1
        // uint8_t remain1[4];             ///< 预留1
        // uint8_t front_edg_value2[3];    ///< 前沿值2
        // uint8_t pulsewidth2[3];         ///< 脉宽2
        // uint16_t peak_value2;           ///< 峰值2
        // uint8_t echo_num2;              ///< 回波重数2
        // uint8_t remain2[4];             ///< 预留2
        
    }Vanjee750Block6031;

    typedef struct 
    {
        int16_t x;   ///< 点云x坐标值
        int16_t y;   ///< 点云y坐标值
        int16_t z;   ///< 点云z坐标值
        uint16_t cur_peak_value;        ///< 当前使用峰值
        uint32_t cur_pulsewidth;        ///< 当前使用脉宽
        uint8_t data_flag;              ///< 携带信息flag
        uint8_t mode;                   ///< 
        uint16_t pixel_num;             ///< 像素序号
        uint16_t group_num;             ///< 组序号
        uint8_t front_edg_value1[3];    ///< 前沿值1
        uint8_t pulsewidth1[3];         ///< 脉宽1
        uint16_t peak_value1;           ///< 峰值1
        uint8_t echo_num1;              ///< 回波重数1
        uint8_t remain1[4];             ///< 预留1
        uint8_t front_edg_value2[3];    ///< 前沿值2
        uint8_t pulsewidth2[3];         ///< 脉宽2
        uint16_t peak_value2;           ///< 峰值2
        uint8_t echo_num2;              ///< 回波重数2
        uint8_t remain2[4];             ///< 预留2
        
    }Vanjee750Block6031xyz1;

    typedef struct 
    {
        uint8_t device_id[16];      ///< 设备ID
        uint16_t main_group_no;     ///< group编号
        uint16_t sub_group_no;      ///< group子编号
        uint8_t remain1[4];         ///< 预留
        uint16_t frame_num;         ///< 总帧号
        uint16_t cur_frame_no;      ///< 当前帧号
        uint32_t second;            ///< UNIX时间戳 s
        uint32_t microsecond;       ///< UNIX时间戳 us
        uint8_t time_syn_state;     ///< 时间同步状态
        uint8_t time_syn_source;    ///< 时间同步来源
        uint8_t time_syn_error;     ///< 时间同步误差
        uint8_t protocol_ver;       ///< 协议版本号
        uint8_t remain2[20];        ///< 预留
    }Vanjee750DifopPkt6031;

    typedef struct 
    {
        uint16_t header;            ///< 数据块头FFCC
        Vanjee750Block6031 blocks[60];  ///< 数据块
        Vanjee750DifopPkt6031 difop;    ///< 设备信息           
    }Vanjee750MsopPkt6031;

    typedef struct 
    {
        uint16_t header;            ///< 数据块头FFCC
        Vanjee750Block6031xyz1 blocks[30];  ///< 数据块
        Vanjee750DifopPkt6031 difop;    ///< 设备信息           
    }Vanjee750MsopPkt6031_30xyz1;
   
    typedef struct 
    {
        int16_t x;   ///< 点云x坐标值
        int16_t y;   ///< 点云y坐标值
        int16_t z;   ///< 点云z坐标值
        uint32_t front_edg_value1;    ///< 前沿值1
        uint32_t pulsewidth1;         ///< 脉宽1
        uint16_t peak_value1;         ///< 峰值1
        uint8_t echo_num1;            ///< 回波重数1
        uint32_t front_edg_value2;    ///< 前沿值2
        uint32_t pulsewidth2;         ///< 脉宽2
        uint16_t peak_value2;         ///< 峰值2
        uint8_t echo_num2;            ///< 回波重数2
        
    }Vanjee750Block459xyz;

    typedef struct 
    {
        uint8_t device_id[16];      ///< 设备ID
        uint16_t hor_start_angle;   ///< 水平起始角度
        uint16_t hor_end_angle;     ///< 水平结束角度
        uint16_t ver_angle;         ///< 垂直角度
        uint16_t frame_num;         ///< 总帧号
        uint16_t cur_frame_no;      ///< 当前帧号
        uint32_t second;            ///< UNIX时间戳 s
        uint32_t microsecond;       ///< UNIX时间戳 us
        uint8_t time_syn_state;     ///< 时间同步状态
        uint8_t time_syn_source;    ///< 时间同步来源
        uint8_t time_syn_error;     ///< 时间同步误差
        uint8_t protocol_ver;       ///< 协议版本号
    }Vanjee750DifopPkt459;

    struct Vanjee750MsopPkt459xyz
    {
        uint16_t header;            ///< 数据块头FFCC
        Vanjee750Block459xyz blocks[48];  ///< 数据块
        Vanjee750DifopPkt459 difop;    ///< 设备信息           

        void ToLittleEndian()
        {
            difop.hor_start_angle = ntohs(difop.hor_start_angle);
            difop.hor_end_angle = ntohs(difop.hor_end_angle);
            difop.ver_angle = ntohs(difop.ver_angle);
            difop.frame_num = ntohs(difop.frame_num);
            difop.cur_frame_no = ntohs(difop.cur_frame_no);
            difop.second = ntohl(difop.second);
            difop.microsecond = ntohl(difop.microsecond);
        }
    };

    typedef struct 
    {
        uint16_t distance;             ///< 点云距离值
        uint8_t hor_id;               ///< 水平编号
        uint8_t ver_id;               ///< 垂直编号
        uint32_t front_edg_value1;    ///< 前沿值1
        uint32_t pulsewidth1;         ///< 脉宽1
        uint16_t peak_value1;         ///< 峰值1
        uint8_t echo_num1;            ///< 回波重数1
        uint32_t front_edg_value2;    ///< 前沿值2
        uint32_t pulsewidth2;         ///< 脉宽2
        uint16_t peak_value2;         ///< 峰值2
        uint8_t echo_num2;            ///< 回波重数2
        
    }Vanjee750Block459d;

    struct Vanjee750MsopPkt459d
    {
        uint16_t header;            ///< 数据块头FFCC
        Vanjee750Block459d blocks[48];  ///< 数据块
        Vanjee750DifopPkt459 difop;    ///< 设备信息    

        void ToLittleEndian()
        {
            difop.hor_start_angle = ntohs(difop.hor_start_angle);
            difop.hor_end_angle = ntohs(difop.hor_end_angle);
            difop.ver_angle = ntohs(difop.ver_angle);
            difop.frame_num = ntohs(difop.frame_num);
            difop.cur_frame_no = ntohs(difop.cur_frame_no);
            difop.second = ntohl(difop.second);
            difop.microsecond = ntohl(difop.microsecond);
        }
    };

    #pragma pack(pop)

    template <typename T_PointCloud>
    class DecoderVanjee750 : public DecoderMech<T_PointCloud>
    {
    private:
        int32_t preFramsNo = -1;

        std::shared_ptr<SplitStrategy> split_strategy_; 
        static WJDecoderMechConstParam &getConstParam(uint8_t mode);
        ChanAngles chan_angles_; // vert_angles/horiz_angles adjustment
        int32_t preAzimuth = -1;
        uint8_t publish_mode = 0;
        bool first_time_come_flag = true;

        bool count_local_time_flag = false; // 计算本地时间到雷达时间时间差标志
        double local2lidar_time_gap = 0;    // 计算本地时间到雷达时间时间差（避免接收点云时间不均匀，导致雷达每个点时间失真）

        bool pubilsh_flag = false;
        std::vector<::vector<double>>SinVA_CosHA;
        std::vector<::vector<double>>CosVA;
        std::vector<::vector<double>>SinVA_SinHA;

        std::vector<std::string> vStrSplit(std::string strSur, char cConChar);
        int loadFromFile(const std::string &angle_path, std::vector<std::vector<double>> &angles_value);

    public:
        constexpr static double FRAME_DURATION = 0.05;
        constexpr static uint32_t SINGLE_PKT_NUM = 450;
        virtual bool decodeMsopPkt(const uint8_t *pkt, size_t size);
        virtual void processDifopPkt(std::shared_ptr<ProtocolBase> protocol);
        virtual ~DecoderVanjee750() = default;
        explicit DecoderVanjee750(const WJDecoderParam &param);

        bool decodeMsopPkt_459_xyz(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_459_distance(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_6031(const uint8_t *pkt, size_t size);
        bool decodeMsopPkt_6031_30xyz1(const uint8_t *pkt, size_t size);
    };
    template <typename T_PointCloud>
    inline WJDecoderMechConstParam &DecoderVanjee750<T_PointCloud>::getConstParam(uint8_t mode)
    {
        // WJDecoderConstParam
        // WJDecoderMechConstParam
        static WJDecoderMechConstParam param =
            {
                1382 // msop len
                , 360 // laser number
                , 30 // blocks per packet
                , 360 // channels per block
                , 0.3f // distance min
                , 10.0f // distance max
                , 0.002f // distance resolution
                , 80.0f // initial value of temperature
            };
        param.BLOCK_DURATION = 0.1 / 360;
        return param;
    }

    template <typename T_PointCloud>
    inline DecoderVanjee750<T_PointCloud>::DecoderVanjee750(const WJDecoderParam &param)
        : DecoderMech<T_PointCloud>(getConstParam(param.publish_mode), param), chan_angles_(this->const_param_.LASER_NUM)
    {
        publish_mode = param.publish_mode;
        this->packet_duration_ = FRAME_DURATION / SINGLE_PKT_NUM;
        split_strategy_ = std::make_shared<SplitStrategyByBlock>(0);
        
        if(this->param_.config_from_file)
        {
            std::string angle_path_SinVA_COSHA = this->param_.angle_path_hor + "/SinVA_COSHA.ini";
            std::string angle_path_CosVA = this->param_.angle_path_hor + "/CosVA.ini";
            std::string angle_path_SinVA_SinHA = this->param_.angle_path_hor + "/SinVA_SinHA.ini";
            if(loadFromFile(angle_path_SinVA_COSHA, SinVA_CosHA) == 0 &&
                    loadFromFile(angle_path_CosVA, CosVA) == 0 &&
                            loadFromFile(angle_path_SinVA_SinHA, SinVA_SinHA) == 0)
            {
                this->angles_ready_ = 1;
            }
        }

    }

    /// <summary>
    /// 字符串拆分
    /// </summary>
    /// <param name="strSur">需要进行拆分的字符串</param>
    /// <param name="cConChar">拆分字符</param>
    /// <returns>返回拆分后的各个子字符串</returns>
    template <typename T_PointCloud>
    std::vector<std::string> DecoderVanjee750<T_PointCloud>::vStrSplit(std::string strSur, char cConChar)
    {
        std::vector<std::string> vStrVec;//向量容器
        std::string::size_type pos1, pos2;//记录字符串出现位置
        pos1 = 0;
        pos2 = strSur.find(cConChar, 0);//从0位置开始查找出现字符串cConChar的位置
        //查找到字符串
        while (std::string::npos != pos2)
        {
            //将从pos1位置开始，长pos2 - pos1 的 字符串 插入vStrVec
            vStrVec.push_back(strSur.substr(pos1, pos2 - pos1));
            pos1 = pos2 + 1;
            pos2 = strSur.find(cConChar, pos1);
        }
        // 将从某位置开始的剩余字符串 插入 vStrVec
        vStrVec.push_back(strSur.substr(pos1));
        return vStrVec;
    }

    template <typename T_PointCloud>
    int DecoderVanjee750<T_PointCloud>::loadFromFile(const std::string &angle_path, std::vector<std::vector<double>> &angles_value)
    {
        angles_value.clear();
        angles_value.resize(112);
        for(int i = 0; i < 112; i++)
        {
            angles_value[i].resize(192);
        }

        std::ifstream fd(angle_path.c_str(), std::ios::in);
        if (!fd.is_open())
        {
            WJ_WARNING << "fail to open vangle file:" << angle_path << WJ_REND;
            return -1;
        }

        try
        {
            std::string line;
            int rowid = 0;
            while (std::getline(fd, line))
            {
                std::stringstream sinFile(line);
                std::vector<std::string> LineData = vStrSplit(sinFile.str(), ',');

                for(int i = 0; i < 192;i++)
                {
                    angles_value[rowid][i] = std::stod(LineData[i]);
                }
                rowid++;
            }
        }
        catch(...)
        {
            WJ_ERROR << "The format of angle config file " << angle_path
            << " is wrong. Please check (e.g. indentation)." << WJ_REND;
        }
        fd.close();
        return 0;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt(const uint8_t *pkt, size_t size)
    {
        bool ret = false;
        uint16_t l_pktheader = pkt[0]<<8 | pkt[1];
        switch(l_pktheader)
        {
            case 0xFFCC:
                {
                    if(size == 1142)
                        ret = decodeMsopPkt_6031(pkt , size);
                    else if(size == 1382)
                        ret = decodeMsopPkt_6031_30xyz1(pkt , size);
                    else if(size == 1384)
                        ret = decodeMsopPkt_459_xyz(pkt , size);
                }
                break;
            case 0xFFDD:
                {
                    if(size == 1288)
                        ret = decodeMsopPkt_459_distance(pkt , size);
                }
                break;
            default:
                break;
        }  
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt_6031(const uint8_t *packet, size_t size)
    {
        const Vanjee750MsopPkt6031 &pkt = *(Vanjee750MsopPkt6031 *)packet;
        bool ret = false;
        double pkt_ts = 0;

        // int32_t loss_packets_num = (pkt.difop.cur_frame_no + 900 - preFramsNo) % 900;
        // if(loss_packets_num > 20 && preFramsNo >= 0)
        //     WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        // preFramsNo = pkt.difop.cur_frame_no;

        if(!this->param_.use_lidar_clock)
        {
            //uint64_t ts = getTimeHost();
            pkt_ts = getTimeHost() * 1e-6;
        }
        else
        {
            double sec = pkt.difop.second;
            double usec = pkt.difop.microsecond;
            pkt_ts = sec + usec * 1e-6;
        }

        if (split_strategy_->newBlock(pkt.difop.cur_frame_no))
        {
            this->cb_split_frame_(360, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;
        }
        
        // int column = pixel % 4 + group * 4;
        // int row = pixel / 4;
        for (uint16_t blk = 0; blk < 60; blk++)
        {
            float x = pkt.blocks[blk].x / 500.0;
            float y = pkt.blocks[blk].y / 500.0;
            float z = pkt.blocks[blk].z / 500.0;

            typename T_PointCloud::PointT point;
            if(x == 0 && y == 0 && z ==0)
            {
                if(!this->param_.dense_points)
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
            }
            else
            {
                this->transformPoint(x, y, z);
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                if(pkt.blocks[blk].use_peak_value_index == 0)
                    setIntensity(point, pkt.blocks[blk].peak_value1);
                else
                    setIntensity(point, pkt.blocks[blk].peak_value2);
            }
            setTimestamp(point, pkt_ts);
            setRing(point, (pkt.blocks[blk].pixel_num % 4) + (pkt.blocks[blk].group_num * 4));
            this->point_cloud_->points.emplace_back(point);

            this->prev_point_ts_ =  pkt_ts;
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt_6031_30xyz1(const uint8_t *packet, size_t size)
    {
        const Vanjee750MsopPkt6031_30xyz1 &pkt = *(Vanjee750MsopPkt6031_30xyz1 *)packet;
        bool ret = false;
        double pkt_ts = 0;

        int32_t loss_packets_num = (pkt.difop.cur_frame_no + 1800 - preFramsNo) % 1800;
        if(loss_packets_num > 20 && preFramsNo >= 0)
            // && loss_packets_num != 1561 && loss_packets_num != 241 &&
            // loss_packets_num != 1041 && loss_packets_num != 761 &&
            // loss_packets_num != 1321 && loss_packets_num != 481 &&
            // loss_packets_num != 1361 && loss_packets_num != 441 &&
            // loss_packets_num != 1601 && loss_packets_num != 201 &&
            // loss_packets_num != 1121 && loss_packets_num != 681 &&
            // loss_packets_num != 1241 && loss_packets_num != 561 &&
            // loss_packets_num != 1521 && loss_packets_num != 281 &&
            // loss_packets_num != 1481 && loss_packets_num != 321 &&
            // loss_packets_num != 1781 && loss_packets_num != 21 &&
            // loss_packets_num != 1761 && loss_packets_num != 41 &&
            // loss_packets_num != 1401 && loss_packets_num != 401 &&
            // loss_packets_num != 1281 && loss_packets_num != 521 &&
            // loss_packets_num != 921 && loss_packets_num != 881 &&
            // loss_packets_num != 1721 && loss_packets_num != 81 &&
            // loss_packets_num != 1001 && loss_packets_num != 801)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = pkt.difop.cur_frame_no;

        if(!this->param_.use_lidar_clock)
        {
            pkt_ts = getTimeHost() * 1e-6;
        }
        else
        {
            double sec = pkt.difop.second;
            double usec = pkt.difop.microsecond;
            pkt_ts = sec + usec * 1e-6;
        }

        if (split_strategy_->newBlock(pkt.difop.cur_frame_no))
        {
            this->cb_split_frame_(360, this->cloudTs());
            this->first_point_ts_ = pkt_ts;
            ret = true;
        }
        
        // int column = pixel % 4 + group * 4;
        // int row = pixel / 4;
        for (uint16_t blk = 0; blk < 30; blk++)
        {
            float x = pkt.blocks[blk].x / 500.0;
            float y = pkt.blocks[blk].y / 500.0;
            float z = pkt.blocks[blk].z / 500.0;

            typename T_PointCloud::PointT point;
            if(x == 0 && y == 0 && z ==0)
            {
                if(!this->param_.dense_points)
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
            }
            else
            {
                this->transformPoint(x, y, z);
                setX(point, x);
                setY(point, y);
                setZ(point, z);
                setIntensity(point, pkt.blocks[blk].cur_peak_value);
            }
            setTimestamp(point, pkt_ts);
            setRing(point, (pkt.blocks[blk].pixel_num % 4) + (pkt.blocks[blk].group_num * 4));
            this->point_cloud_->points.emplace_back(point);

            this->prev_point_ts_ =  pkt_ts;
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt_459_xyz(const uint8_t *packet, size_t size)
    {
        Vanjee750MsopPkt459xyz &pkt = *(Vanjee750MsopPkt459xyz *)packet;
        pkt.ToLittleEndian();
        bool ret = false;
        double pkt_ts = 0;

        int32_t loss_packets_num = (pkt.difop.cur_frame_no + 448 - preFramsNo) % 448;
        if(loss_packets_num > 20 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = pkt.difop.cur_frame_no;

        if(!this->param_.use_lidar_clock)
        {
            //uint64_t ts = getTimeHost();
            pkt_ts = getTimeHost() * 1e-6;
        }
        else
        {
            double sec = pkt.difop.second;
            double usec = pkt.difop.microsecond;
            pkt_ts = sec + usec * 1e-6;
        }

        if (split_strategy_->newBlock(pkt.difop.cur_frame_no))
        {
            if(pubilsh_flag)
            {
                this->cb_split_frame_(112, this->cloudTs());
                this->first_point_ts_ = pkt_ts;
                ret = true;
                pubilsh_flag = false;
            }
            else
            {
                pubilsh_flag = true;
            }
        }

        for (uint16_t blk = 0; blk < 48; blk++)
        {
            if(pkt.blocks[blk].front_edg_value1 != 0)
            {
                float x = pkt.blocks[blk].x / 500.0;
                float y = pkt.blocks[blk].y / 500.0;
                float z = pkt.blocks[blk].z / 500.0;

                typename T_PointCloud::PointT point;
                if(x == 0 && y == 0 && z ==0)
                {
                    if(!this->param_.dense_points)
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
                }
                else
                {
                    this->transformPoint(x, y, z);
                    setX(point, x);
                    setY(point, y);
                    setZ(point, z);
                    setIntensity(point, pkt.blocks[blk].peak_value1);
                }
                setTimestamp(point, pkt_ts);
                setRing(point, pkt.difop.ver_angle);

                this->point_cloud_->points.emplace_back(point);
            }
            this->prev_point_ts_ =  pkt_ts;
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    inline bool DecoderVanjee750<T_PointCloud>::decodeMsopPkt_459_distance(const uint8_t *packet, size_t size)
    {
        Vanjee750MsopPkt459d &pkt = *(Vanjee750MsopPkt459d *)packet;
        pkt.ToLittleEndian();
        bool ret = false;
        double pkt_ts = 0;

        int32_t loss_packets_num = (pkt.difop.cur_frame_no + 448 - preFramsNo) % 448;
        if(loss_packets_num > 20 && preFramsNo >= 0)
            WJ_WARNING << "loss " << loss_packets_num << " packets" << WJ_REND;
        preFramsNo = pkt.difop.cur_frame_no;

        if(!this->param_.use_lidar_clock)
        {
            //uint64_t ts = getTimeHost();
            pkt_ts = getTimeHost() * 1e-6;
        }
        else
        {
            double sec = pkt.difop.second;
            double usec = pkt.difop.microsecond;
            pkt_ts = sec + usec * 1e-6;
        }

        if (split_strategy_->newBlock(pkt.difop.cur_frame_no))
        {
            if(pubilsh_flag)
            {
                this->cb_split_frame_(112, this->cloudTs());
                this->first_point_ts_ = pkt_ts;
                ret = true;
                pubilsh_flag = false;
            }
            else
            {
                pubilsh_flag = true;
            }
        }

        if(SinVA_CosHA.size() < 112 || CosVA.size() < 112  || SinVA_SinHA.size() < 112)
        {
            WJ_ERROR << "未加载配置表,请配置相关参数!\r\n" << WJ_REND;
            return ret;
        }

        for (uint16_t blk = 0; blk < 48; blk++)
        {
            float distance = pkt.blocks[blk].distance / 500.0;
            float x = -distance * SinVA_CosHA[pkt.difop.ver_angle][pkt.difop.hor_start_angle + blk];
            float y = distance * CosVA[pkt.difop.ver_angle][pkt.difop.hor_start_angle + blk];
            float z = distance * SinVA_SinHA[pkt.difop.ver_angle][pkt.difop.hor_start_angle + blk];

            if(pkt.blocks[blk].front_edg_value1 != 0)
            {
                typename T_PointCloud::PointT point;
                if(!this->distance_section_.in(distance))
                {
                    if(!this->param_.dense_points)
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
                }
                else
                {
                    this->transformPoint(x, y, z);
                    setX(point, x);
                    setY(point, y);
                    setZ(point, z);
                    setIntensity(point, pkt.blocks[blk].peak_value1);
                }
                setTimestamp(point, pkt_ts);
                setRing(point, pkt.difop.ver_angle);

                this->point_cloud_->points.emplace_back(point);
            }
            this->prev_point_ts_ =  pkt_ts;
        }
        this->prev_pkt_ts_ = pkt_ts;
        return ret;
    }

    template <typename T_PointCloud>
    void DecoderVanjee750<T_PointCloud>::processDifopPkt(std::shared_ptr<ProtocolBase> protocol)
    {
      std::shared_ptr<CmdClass> sp_cmd = std::make_shared<CmdClass>(protocol->MainCmd,protocol->SubCmd);
      std::shared_ptr<ProtocolAbstract> p;

    }
  } // namespace lidar

} // namespace vanjee
