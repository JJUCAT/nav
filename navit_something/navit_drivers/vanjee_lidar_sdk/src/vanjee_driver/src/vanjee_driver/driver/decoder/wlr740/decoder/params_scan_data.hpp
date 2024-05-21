#pragma once 

namespace vanjee
{
    namespace lidar
    {
        class Params_ScanData0728
        {

        public:
        #pragma pack(push)
        #pragma pack(1)
                // 数据块 角度信息
            struct ViewField
            {
                // 计时值
                unsigned short TimingValue; //= 0;
                // 反射率
                unsigned char reflectivity; //= 0;
            };
        #pragma pack(pop)

        #pragma pack(push)
        #pragma pack(1)
            // 数据块
            struct DataBlock
            {
                // 数据块的起始标志
                unsigned char DataBlockStartFlag = 0;
                // 转镜
                unsigned char RotatingMirror = 0;
                // 水平角度
                short HAngle = 0;
                // 水平角度分辨率 0.1 / 0.2
                unsigned char HAngleResolution = 0;
                // 阈值选择 1 低阈值 2高阈值
                unsigned char ThresholdSelection = 0;
                // 回波类型 1 最强一重 2最后一重
                unsigned char EchoType = 0;
                // 垂直方向补偿角 有符号
                char VCompensateAngle = 0;
                // 水平方向补偿角 有符号
                int HCompensateAngle = 0;

                ViewField viewField[48] = {{0}};
            };
        #pragma pack(pop)

        #pragma pack(push)
        #pragma pack(1)
            struct ScanData // 扫描协议
            {
                unsigned char Head[2] = {0,0};
                unsigned short FrameLen = 0;     
                // 设备编号
                unsigned short DeviceNumber = 0;
                // 帧类别
                unsigned short FrameType = 0;
                // 当前帧创建时间
                unsigned char CreateTime[12] = {0};
                // 帧序号
                unsigned short FrameNo = 0;
                // 网络协议版本号
                unsigned char NetworkVersion[4];
                // 水平偏移角
                unsigned short Horizontal_offset_angle;

                DataBlock dataBlock[8];
                unsigned char check = 0;
            };
        #pragma pack(pop)

        public:
            ScanData Data;
            const int viewFieldSize = 48;
            const int dataBlockSize = 8;

            // 获取当前帧的回波模式 1：双回波 2：单回波
            unsigned char GetEchoModel()
            {
                if(Data.dataBlock[0].EchoType != Data.dataBlock[1].EchoType)
                {
                    return 1;
                }
                else
                {
                    return 2;
                }
            }
            int GetHCompensateAngle(int DateBlockIndex);
        };

        class Params_ScanData0728Reflectivity
        {

        public:
        #pragma pack(push)
        #pragma pack(1)
                // 数据块 角度信息
            struct ViewField
            {
                // 计时值
                unsigned short TimingValue;// = 0;
                // 反射率
                unsigned char reflectivity;// = 0;
                // 低阈值脉宽
                unsigned short LowPulseWidth;// = 0;
                // 高阈值脉宽
                unsigned short TallPulseWidth;// = 0;
            };
        #pragma pack(pop)

        #pragma pack(push)
        #pragma pack(1)
            // 数据块
            struct DataBlock
            {
                // 数据块的起始标志
                unsigned char DataBlockStartFlag = 0;
                // 转镜
                unsigned char RotatingMirror = 0;
                // 水平角度
                short HAngle = 0;
                // 水平角度分辨率 0.1 / 0.2
                unsigned char HAngleResolution = 0;
                // 阈值选择 1 低阈值 2高阈值
                unsigned char ThresholdSelection = 0;
                // 回波类型 1 最强一重 2最后一重
                unsigned char EchoType = 0;
                // 垂直方向补偿角 有符号
                char VCompensateAngle = 0;
                // 水平方向补偿角 有符号
                int HCompensateAngle = 0;

                ViewField viewField[48] = {{0}};

                // 48个通道强弱发光
                unsigned char StrongWeak[6] = {0};
            };
        #pragma pack(pop)

        #pragma pack(push)
        #pragma pack(1)
            struct ScanData // 扫描协议
            {
                unsigned char Head[2] = {0,0};
                unsigned short FrameLen = 0;     
                // 设备编号
                unsigned short DeviceNumber = 0;
                // 帧类别
                unsigned short FrameType = 0;
                // 当前帧创建时间
                unsigned char CreateTime[12] = {0};
                // 帧序号
                unsigned short FrameNo = 0;
                // 网络协议版本号
                unsigned char NetworkVersion[4];
                // 水平偏移角
                unsigned short Horizontal_offset_angle;

                DataBlock dataBlock[4];
                unsigned char check = 0;
            };
        #pragma pack(pop)

        public:
            ScanData Data;
            const int viewFieldSize = 48;
            const int dataBlockSize = 4;
            // 获取当前帧的回波模式 1：双回波 2：单回波
            unsigned char GetEchoModel()
            {
                if(Data.dataBlock[0].EchoType != Data.dataBlock[1].EchoType)
                {
                    return 1;
                }
                else
                {
                    return 2;
                }
            }
        };

    }
}
