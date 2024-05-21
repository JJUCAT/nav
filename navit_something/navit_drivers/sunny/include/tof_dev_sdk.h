#ifndef __TOF_DEVICE_SDK_H__
#define __TOF_DEVICE_SDK_H__

#include "tof_sdk_typedef.h"

#ifdef WIN32
    #ifdef TOF_DEVICE_SDK_EXPORT
        #define TOFDDLL __declspec(dllexport)
    #else
        #define TOFDDLL __declspec(dllimport)
    #endif
#else
    #define TOFDDLL __attribute__((visibility("default")))
#endif


typedef enum tagTOF_DEV_TYPE
{
	TOF_DEV_CHROMEBOOK		   = MAKE_UNIQUE_ID('C',  'M',  'B', 0x00),//ChromeBook
	TOF_DEV_CLEANER01A         = MAKE_UNIQUE_ID('C', 0x01,  'A', 0x00),//Cleaner01A
	TOF_DEV_CLEANER01APLUS     = MAKE_UNIQUE_ID('C', 0x01,  'A', 0x01),//Cleaner01A（Plus版）
	TOF_DEV_CLEANER01APRO      = MAKE_UNIQUE_ID('C', 0x01,  'A', 0x02),//Cleaner01A（Pro版）
	TOF_DEV_CLEANER01A_NET     = MAKE_UNIQUE_ID('C', 0x01,  'A', 0x03),//Cleaner01A（网络版）
	TOF_DEV_CLEANER01A2        = MAKE_UNIQUE_ID('C', 0x01, 0xA2, 0x00),//Cleaner01A2
	TOF_DEV_CLEANER01B         = MAKE_UNIQUE_ID('C', 0x01,  'B', 0x00),//Cleaner01B
	TOF_DEV_CLEANER01D         = MAKE_UNIQUE_ID('C', 0x01,  'D', 0x00),//Cleaner01D
	TOF_DEV_CLEANER01D_NET     = MAKE_UNIQUE_ID('C', 0x01,  'D', 0x01),//Cleaner01D（网络版）
	TOF_DEV_CLEANER01E_NET     = MAKE_UNIQUE_ID('C', 0x01,  'E', 0x01),//Cleaner01E（网络版）
	TOF_DEV_CLEANER01F         = MAKE_UNIQUE_ID('C', 0x01,  'F', 0x00),//Cleaner01F
	TOF_DEV_CLEANER01F1        = MAKE_UNIQUE_ID('C', 0x01,  'F', 0x01),//Cleaner01F1
	TOF_DEV_CLEANER01G         = MAKE_UNIQUE_ID('C', 0x01,  'G', 0x00),//Cleaner01G
	TOF_DEV_CLEANER01G1        = MAKE_UNIQUE_ID('C', 0x01,  'G', 0x01),//Cleaner01G1
	TOF_DEV_CLEANER01X         = MAKE_UNIQUE_ID('C', 0x01,  'X', 0x00),//Cleaner01X
	TOF_DEV_CLEANER02A         = MAKE_UNIQUE_ID('C', 0x02,  'A', 0x00),//Cleaner02A
	TOF_DEV_CLEANER02A_NET     = MAKE_UNIQUE_ID('C', 0x02,  'A', 0x01),//Cleaner02A（网络版）
	TOF_DEV_MARS01A            = MAKE_UNIQUE_ID('M', 0x01,  'A', 0x00),//Mars01A
	TOF_DEV_MARS01B            = MAKE_UNIQUE_ID('M', 0x01,  'B', 0x00),//Mars01B
	TOF_DEV_MARS01C            = MAKE_UNIQUE_ID('M', 0x01,  'C', 0x00),//Mars01C
	TOF_DEV_MARS01D            = MAKE_UNIQUE_ID('M', 0x01,  'D', 0x00),//Mars01D
	TOF_DEV_MARS01E            = MAKE_UNIQUE_ID('M', 0x01,  'E', 0x00),//Mars01E
	TOF_DEV_MARS04             = MAKE_UNIQUE_ID('M', 0x04, 0x00, 0x00),//Mars04
	TOF_DEV_MARS04A            = MAKE_UNIQUE_ID('M', 0x04,  'A', 0x00),//Mars04A
	TOF_DEV_MARS04B            = MAKE_UNIQUE_ID('M', 0x04,  'B', 0x00),//Mars04B
	TOF_DEV_MARS05             = MAKE_UNIQUE_ID('M', 0x05, 0x00, 0x00),//Mars05
	TOF_DEV_MARS05A            = MAKE_UNIQUE_ID('M', 0x05,  'A', 0x00),//Mars05A
	TOF_DEV_MARS05B            = MAKE_UNIQUE_ID('M', 0x05,  'B', 0x00),//Mars05B
	TOF_DEV_MARS05B_BCTC       = MAKE_UNIQUE_ID('M', 0x05,  'B', 0x01),//Mars05B(BCTC版本)
	TOF_DEV_MARS05B_BCTC_SUNNY = MAKE_UNIQUE_ID('M', 0x05,  'B', 0x02),//Mars05B(BCTC版本_sunny)
	TOF_DEV_USBTOF_HI          = MAKE_UNIQUE_ID('U',  'T',  'H', 0x00),//UsbTof-Hi
	TOF_DEV_DREAM              = MAKE_UNIQUE_ID('D',  'R',  'M', 0x00),//DREAM
	TOF_DEV_HOT002             = MAKE_UNIQUE_ID('H',  'O',  'T', 0x02),//HOT002
	TOF_DEV_HOT002A            = MAKE_UNIQUE_ID('H',  'O',  'T', 0x2a),//HOT002A
	TOF_DEV_HSP008             = MAKE_UNIQUE_ID('H',  'S',  'P', 0x08),//HSP008
	TOF_DEV_HSR003             = MAKE_UNIQUE_ID('H',  'S',  'R', 0x03),//HSR003
	TOF_DEV_HSR003A            = MAKE_UNIQUE_ID('H',  'S',  'R', 0x3A),//HSR003A
	TOF_DEV_HSR004             = MAKE_UNIQUE_ID('H',  'S',  'R', 0x04),//HSR004
	TOF_DEV_HST003             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x03),//HST003
	TOF_DEV_HST006             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x06),//HST006
	TOF_DEV_HST007             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x07),//HST007
	TOF_DEV_HST008             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x08),//HST008
	TOF_DEV_HST009             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x09),//HST009
	TOF_DEV_HST010             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x10),//HST010
	TOF_DEV_HST013             = MAKE_UNIQUE_ID('H',  'S',  'T', 0x13),//HST013
	TOF_DEV_SEEKER07C	       = MAKE_UNIQUE_ID('S',  'E',  'K', 0x7C),//seeker07c
	TOF_DEV_SEEKER08A          = MAKE_UNIQUE_ID('S',  'E',  'K', 0x8A),//seeker08A
	TOF_DEV_LOGITECH_C525      = MAKE_UNIQUE_ID('L',  'G', 0xC5, 0x25),//Logitech C525


	//这部分为demo模块
	TOF_DEV_DEMO_3DCP_NET      = MAKE_UNIQUE_ID(0xde, 0x3d, 'C', 0x00),//demo版3DCP（网络版）
	TOF_DEV_DEMO_3DCP          = MAKE_UNIQUE_ID(0xde, 0x3d, 'C', 0x01),//demo版3DCP
	TOF_DEV_DEMO_C00P01A_NET   = MAKE_UNIQUE_ID(0xde, 0xC0, 'P', 0x1A),//demo版C00P01A的RGBD模块（网络版）
	TOF_DEV_DEMO_UPG           = MAKE_UNIQUE_ID(0xde,  'U', 'P',  'G'),//demo版UPG
	TOF_DEV_DEMO_GENERAL_UVC   = TOF_DEV_DEMO_UPG,//demo版GeneralUvc

}TOF_DEV_TYPE;


typedef struct tagTofFrameData
{
	UINT64 frameID;//帧ID
	UINT64 timeStamp;//时间戳
	UINT32 frameWidth;//输出的数据的宽
	UINT32 frameHeight;//输出的数据的高

	//
	FLOAT32* pDepthData;//射线距离（滤波前）
	FLOAT32* pDepthDataFilter;//射线距离（滤波后）
	//
	PointData* pPointData;//点云数据
	PointData* pPointDataUnfilter;//点云数据（滤波前）
	//
	UINT8* pGrayData;//灰度数据
	UINT8* pConfidence;//置信度数据
	UINT8* pIntensity;//环境光数据

	RgbDData* pRgbD;//RgbD数据

	PixelCoordData* pRgb2TofPixelCoord;//RGB坐标与TOF坐标的映射表（可能为空）

	UINT8* pMaxValidPixelFlag;//最大有效像素标记

	void*  pRawData;//raw数据（支持raw数据的板子才可以）
	UINT32 nRawDataLen;//pRawData内raw数据长度，字节数

	SBOOL bMultiDevInterference;//是否出现了多机干扰
	SBOOL bHighReflectDetected;//是否检测到高反物体

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void*  pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}TofFrameData;

typedef enum tagANALOG_GAIN_MODE
{
	ANALOG_GAIN_MODE_MANUAL = 0x00000001,//手动模拟增益
	ANALOG_GAIN_MODE_AUTO   = 0x00000002,//自动模拟增益
}ANALOG_GAIN_MODE;

typedef enum tagDIGITAL_GAIN_MODE
{
	DIGITAL_GAIN_MODE_MANUAL = 0x00000001,//手动数字增益
	DIGITAL_GAIN_MODE_AUTO   = 0x00000002,//自动数字增益
}DIGITAL_GAIN_MODE;


typedef struct tagRgbFrameData
{
	UINT64 timeStamp;//时间戳
	UINT32 frameWidth;//输出的数据的宽
	UINT32 frameHeight;//输出的数据的高

	COLOR_FORMAT formatType;//指明pFrameData内数据帧的格式
	COLOR_FORMAT formatTypeOrg;//指明pFrameData内数据帧的格式(编码压缩之前的格式)
	UINT32 nFrameLen;
	UINT8* pFrameData;

	//扩展数据(一般针对客户特殊需求)，不同设备/不同客户均不同，可能为空；
	void*  pExtData;//扩展数据
	UINT32 nExtDataLen;//pExtData内扩展数据长度，字节数

}RgbFrameData;

typedef struct tagImuFrameData
{
	UINT64 timeStamp;//时间戳

	FLOAT32 accelData_x;
	FLOAT32 accelData_y;
	FLOAT32 accelData_z;

	FLOAT32 gyrData_x;
	FLOAT32 gyrData_y;
	FLOAT32 gyrData_z;

	FLOAT32 magData_x;
	FLOAT32 magData_y;
	FLOAT32 magData_z;

}ImuFrameData;


typedef struct tagTofDevInitParam
{
	SCHAR szDepthCalcCfgFileDir[256]; //深度计算所需配置文件的目录，如home/user/temp

	SBOOL bSupUsb; //是否支持USB设备

	SBOOL bSupNetWork; //是否支持网络设备
	SCHAR szHostIPAddr[32]; //本地主机上的某一个网卡的IP地址（也可不填，不填的情况下将会遍历本地所有网卡）

	SBOOL bSupSerialCOM; //是否需要支持串口（需要使用串口时才需赋值true）
	SCHAR szSerialDev[64]; //本地主机上的某一个串口设备（当bSupSerialCOM字段为true时，该字段才有效）
						   //windows环境下可以不填写，也可以填写，如COM1、COM2、...，不填写的情况下将会遍历本地所有串口
						   //linux环境下必须填写，如/dev/ttyS0、/dev/ttyUSB0、...

	SBOOL bWeakAuthority;//是否是权限较低（如非ROOT的安卓系统）,仅适用于linux系统/安卓系统

	SBOOL bDisablePixelOffset;//SDK在内部不进行地址偏移后输出TOF数据（输出给用户的TOF数据分辨率与RAW数据相同）

	UINT32 nLogLevel; //日志打印级别（暂时还未生效）
	SCHAR  szLogFile[256]; //SDK内部记录debug信息的日志文件，如home/user/temp/tof_dev_sdk_log.txt
	UINT32 nLogFileMaxSize; //日志文件允许的最大存储量（字节），赋值过大或者过小，则使用SDK内部默认大小

}TofDevInitParam;



typedef struct tagTofDeviceDescriptor
{
	void* hDevice;
	void* hDriver;

}TofDeviceDescriptor;


typedef struct tagTofDeviceDescriptorWithFd
{
	SINT32 usbDevFd; //USB设备的描述符fd
	UINT16 usbDevVID;//USB设备的VID
	UINT16 usbDevPID;//USB设备的PID

}TofDeviceDescriptorWithFd;
	
typedef struct tagTofDeviceCapability
{
	TOF_MODE tofMode;

	//TOF
	SBOOL bTofSupported;
	UINT32 tofResWidth;
	UINT32 tofResHeight;
	
	//TOF Expouse
	UINT32 supportedTofExpMode;//EXP_MODE的组合
	//TOF Analog Gain
	UINT32 supportedTofAnalogGainMode;//ANALOG_GAIN_MODE的组合
	//TOF Digital Gain
	UINT32 supportedTofDigitalGainMode;//DIGITAL_GAIN_MODE的组合

	//TOF Filter
	UINT32 supportedTOFFilter; //TOF_FILTER的组合

	//TOF HDRZ
	SBOOL bTofHDRZSupported;

	//TOF RemoveINS
	SBOOL bTofRemoveINSSupported;

	//TOF 多路径矫正
	SBOOL bTofMpiCorrectSupported;

	//TOF 多路径矫正结果和传统算法的融合
	SBOOL bTofMpiFuseSupported;

	//TOF滤波等级
	SBOOL bTofFilterLevelSupported;
	UINT8 nTofFilterLevelMin; //最小TOF滤波等级（宽松）
	UINT8 nTofFilterLevelMax; //最大TOF滤波等级（严格）

	//RGB
	SBOOL bRgbSupported;
	COLOR_FORMAT rgbColorFormat;//传出的RGB数据格式
	COLOR_FORMAT rgbColorFormatOrg;//传出的RGB数据格式(编码压缩之前的格式)
	UINT32 rgbResWidth;
	UINT32 rgbResHeight;

	//RGBD
	SBOOL bRgbDSupported;

	//IMU
	SBOOL bImuSupported;

	//远程抓图
	SBOOL bRemoteCaptureSupported;
	//固件升级
	SBOOL bUpgradeFirmwareSupported;
	//固件快速升级
	SBOOL bFastUpgradeFirmwareSupported;
	//设备重启
	SBOOL bRebootDevSupported;
	//主从机间同步时间
	SBOOL bMasterSlaveSyncTimeSupported;
	//烧写TOF模组的杂散光矫正参数
	SBOOL bBurnTofINSParamSupported;
	//烧写TOF标定文件
	SBOOL bBurnTofCalibDataSupported;
	//投射器电流设置
	SBOOL bLedCurrentSupported;
	//多设备抗干扰
	SBOOL bMultiDevAntiInterferenceSupported;
	//出流控制
	SBOOL bStreamCtrlSupported;

	//

}TofDeviceCapability;

typedef struct tagTofDeviceInfo
{
	//BASIC information
	TOF_DEV_TYPE devType;//用于区分是哪款设备
	SCHAR szDevName[32];
	SCHAR szDevId[64];//设备/模块的序列号（标识设备唯一性）
	SCHAR szFirmwareVersion[32];//固件版本信息
	UINT32 tofModuleId;//TOF模组编号

	//Capabilities
	UINT32 capCnt; //实际使用的能力个数，不超过TOF_MAX_CAPS_CNT
	TofDeviceCapability cap[TOF_MAX_CAPS_CNT];//每一种模式下的能力

}TofDeviceInfo;

typedef struct tagTofDeviceParam
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceParam;

typedef struct tagTofDeviceTemperature
{
	FLOAT32 fBoardTemp;//主板温度(需要设备支持)
	FLOAT32 fSensorTemp;//senseor温度(需要设备支持)
	FLOAT32 fImuTemp;//Imu温度(需要设备支持)
}TofDeviceTemperature;

typedef struct tagNetDevInfo
{
	SBOOL bDHCP;//是否是自动获取IP
	UINT8 byRes[3];//字节对齐，预留
	SCHAR szIPv4Address[32];//设备IP地址
	SCHAR szIPv4SubnetMask[32];//设备子网掩码
	SCHAR szIPv4Gateway[32];//设备网关
	SCHAR szMAC[32];//设备MAC地址

}NetDevInfo_t;

typedef struct tagRemoteCapture
{
	UINT8 szRes[4];//预留,4字节对齐
}RemoteCapture;


//固件升级的实时状态
typedef enum tagFIRMWARE_UPGRADE_STATUS
{
	FIRMWARE_UPGRADE_STATUS_FINISHED   = 1,//升级完成
	FIRMWARE_UPGRADE_STATUS_RUNNING    = 2,//正在升级
	FIRMWARE_UPGRADE_STATUS_FAILED     = 3,//升级失败
	FIRMWARE_UPGRADE_STATUS_UNKNOWN    = 4,//升级失败(未知错误)
	FIRMWARE_UPGRADE_STATUS_ERROR_DATA = 5,//升级失败(固件包错误)
	FIRMWARE_UPGRADE_STATUS_IO         = 6,//升级失败(IO读写失败)

}FIRMWARE_UPGRADE_STATUS;

//固件升级的实时状态信息
typedef struct tagFirmwareUpgradeStatus
{
	FIRMWARE_UPGRADE_STATUS status;//升级的状态
	UINT8 nProgress;//实时进度，取值必须处于：0-100
	UINT8 byRes[3];//字节对齐，预留
}FirmwareUpgradeStatus;

//固件升级的实时状态回调函数
typedef void (*FNFirmwareUpgradeStatus)(FirmwareUpgradeStatus *statusData, void* pUserData);

//固件升级数据
typedef struct tagFirmwareUpgradeData
{
	UINT8* pData;//指向固件数据（完整的固件数据首地址）
	UINT32 nDataLen;//pData内固件数据长度（完整的固件数据长度）

	FNFirmwareUpgradeStatus fnUpgradeStatus;//固件升级实时状态回调函数
	void* pUpgradeStatusUserData;//fnUpgradeStatus的pUserData参数
}FirmwareUpgradeData;

//设备重启
typedef struct tagRebootDev
{
	UINT8 byRes[4];//字节对齐，预留
}RebootDev;

//主从机间同步时间
typedef struct tagMasterSlaveSyncTime
{
	UINT64 hostSendTimestamp;//主机发送命令的时间（主机的本地时间）
	UINT64 slaveRecvTimestamp;//从机接收到命令的时间（从机的本地的时间）
	UINT64 slaveSendTimestamp;//从机发送命令的时间（从机的本地时间）
	UINT64 hostRecvTimestamp;//主机接收到命令的时间（主机的本地的时间）

}MasterSlaveSyncTime;

//TOF模拟增益
typedef struct tagTofAnalogGain
{
	SBOOL  bAuto;//是否自动
	UINT8  szRes[2];//4字节对齐，预留
	SBOOL  bUpdataValue;//是否更新增益值到板子（该字段仅在设置时有效）
	SINT32 lCurrent;//当前值

	SINT32 lMax;//最大值（该字段仅在获取时有效）
	SINT32 lMin;//最小值（该字段仅在获取时有效）
}TofAnalogGain;

//TOF数字增益
typedef struct tagTofDigitalGain
{
	SBOOL  bAuto;//是否自动
	UINT8  szRes[2];//4字节对齐，预留
	SBOOL  bUpdataValue;//是否更新增益值到板子（该字段仅在设置时有效）
	SINT32 lCurrent;//当前值

	SINT32 lMax;//最大值（该字段仅在获取时有效）
	SINT32 lMin;//最小值（该字段仅在获取时有效）
}TofDigitalGain;

//模组投射器电流
typedef struct tagLedCurrent
{
	UINT32 lCurrent;//当前值

	UINT32 lMax;//最大值（该字段仅在获取时有效）
	UINT32 lMin;//最小值（该字段仅在获取时有效）
}LedCurrent;

//TOF回调函数里输出的TOF数据相对于RAW数据的像素偏移个数
typedef struct tagTofFrameDataPixelOffset
{
	UINT32 nOffset;//偏移量（像素个数）

}TofFrameDataPixelOffset;


//TOF Sensor状态
typedef enum tagTofSensorStatus
{
	TofSensorStatus_StreamOff = 1,//Sensor不出流
	TofSensorStatus_StreamOn  = 2,//Sensor出流

}TofSensorStatus;

//TOF Sensor状态控制参数
typedef struct tagTofSensorStatusCtrl
{
	TofSensorStatus status;

}TofSensorStatusCtrl;

//RGB Sensor状态控制参数
typedef struct tagRgbSensorStatusCtrl
{
	UINT8 szRes[4];//预留

}RgbSensorStatusCtrl;

//Sensor状态控制
typedef struct tagSensorStatusCtrl
{
	UINT32 nIndex;//1---struTof有效, 2---struRgb有效

	union
	{
		TofSensorStatusCtrl struTof;//TOF Sensor状态控制参数

		RgbSensorStatusCtrl struRgb;//RGB Sensor状态控制参数

	}uParam;

}SensorStatusCtrl;


//快速升级固件（一般需要借助于芯片厂商的升级工具）
typedef struct tagFastUpgradeFirmware
{
	UINT8 szRes;//预留

}FastUpgradeFirmware;

//TOF模组杂散光矫正参数
typedef struct tagTofINSParam
{
	UINT8* pData;//杂散光矫正参数数据
	UINT32 nDataLen;//pData内杂散光矫正参数数据长度
}TofINSParam;

//TOF模组杂散光矫正参数的校验码
typedef struct tagTofINSCheckCode
{
	UINT16 nCRC;//CRC
}TofINSCheckCode;

//回读传输给模组的各种参数的校验码
typedef struct tagParamsCheckCode
{
	UINT16 nCRC;//CRC
}ParamsCheckCode;

//多设备抗干扰
typedef struct tagMultiDevAntiInterference
{
	SBOOL bEnable;//启用、禁用抗干扰
}MultiDevAntiInterference;

/*
*JPEG图像解码回调函数
*参数说明：
*pJpeg：待解码的jpg帧，输入参数
*nJpegLen：待解码的jpg帧的长度，输入参数
*outFormat：要求解码成的格式，输入参数
*pOutBuffer：存放解码后的数据帧，输出参数
*nOutBufLen：pOutBuffer缓冲区长度，输入参数
*pUserData：用于用户传递自定义参数，输入参数
*返回值：解码后数据的长度（也即pOutBuffer缓冲区内存放的帧的长度），不能超过nOutBufLen
*/
typedef UINT32(*FNJpegDec)(UINT8* pJpeg, const UINT32 nJpegLen, const COLOR_FORMAT outFormat, UINT8* pOutBuffer, const UINT32 nOutBufLen, void* pUserData);

//JPEG图像解码回调的参数
typedef struct tagJpegDecParam
{
	FNJpegDec fnDec;//jpeg解码回调函数
	void* pUserData;//用户自定义参数, 也即FNJpegDec里的pUserData字段
}JpegDecParam;

//TOF多路径矫正
typedef struct tagTofMpiCorrect
{
	SBOOL bEnable;//启用、禁用矫正
}TofMpiCorrect;

//TOF多路径矫正结果和传统算法的融合
typedef struct tagTofMpiFuse
{
	SBOOL bEnable;//启用、禁用融合
}TofMpiFuse;

//TOF滤波等级
typedef struct tagTofFilterLevel
{
	UINT32 nLevel;//取值范围:[TofDeviceCapability的nTofFilterLevelMin, nTofFilterLevelMax]
}TofFilterLevel;

//TOF模组信息卡数据详情
typedef struct tagTofModuleInfoCardDetail
{
	UINT8 szModulePartNO[32];//模组料号

}TofModuleInfoCardDetail;

typedef enum tagTOF_DEV_PARAM_TYPE
{
	TOF_DEV_PARAM_Temperature            = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x00),//温度信息
	TOF_DEV_PARAM_TofLensParameter       = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x01),//TOF模组内参和畸变（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
	TOF_DEV_PARAM_TofCalibData           = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x02),//TOF模组标定数据
	TOF_DEV_PARAM_netdevinfo             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x03),//网络接入设备信息
	TOF_DEV_PARAM_ReplaceTofCalibData    = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x04),//替换SDK里TOF模组标定数据（仅仅是替换SDK里标定数据，并非烧写到模组）
	TOF_DEV_PARAM_RemoteCapture          = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x05), //远程抓图：控制模块抓取数据并保存在模块内部；
	TOF_DEV_PARAM_ExportRaw              = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x06), //导出一帧RAW数据：实时的从模块里导出一帧RAW数据（适用于RAW数据和深度数据异步传输的情况）；
	TOF_DEV_PARAM_RgbLensParameter       = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x07),//RGB模组内参和畸变（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
	TOF_DEV_PARAM_UpgradeFirmware        = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x08),//升级固件
	TOF_DEV_PARAM_RebootDev              = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x09),//设备重启
	TOF_DEV_PARAM_StereoLensParameter    = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0a),//双目相机参数
	TOF_DEV_PARAM_GetMasterSlaveSyncTime = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0b),//获取主从机间同步时间
	TOF_DEV_PARAM_TofAnalogGain          = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0c),//TOF模拟增益
	TOF_DEV_PARAM_TofDigitalGain         = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0d),//TOF数字增益
	TOF_DEV_PARAM_TofLensParameterV20    = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0e),//TOF模组内参和畸变（V2.0版本）
	TOF_DEV_PARAM_TofFrameDataPixelOffset= MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x0f),//TOF回调函数里输出的TOF数据相对于RAW数据的像素偏移个数
	TOF_DEV_PARAM_DepthCalRoi            = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x10),//深度计算的区域
	TOF_DEV_PARAM_SensorStatusCtrl       = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x11),//Sensor状态控制
	TOF_DEV_PARAM_RgbLensParameterV20    = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x12),//RGB模组内参和畸变（V2.0版本）
	TOF_DEV_PARAM_RgbdCalibData          = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x13),//RGBD配准的标定数据
	TOF_DEV_PARAM_FastUpgradeFirmware    = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x14),//快速升级固件（一般需要借助于芯片厂商的升级工具）
	TOF_DEV_PARAM_TofINSParam            = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x15),//TOF模组杂散光矫正参数
	TOF_DEV_PARAM_TofINSCheckCode        = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x16),//TOF模组杂散光矫正参数的校验码
	TOF_DEV_PARAM_TofCalibCheckCode      = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x17),//TOF模组标定文件的校验码
	TOF_DEV_PARAM_LedCurrent             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x18),//模组投射器光源的电流
	TOF_DEV_PARAM_MultiDevAntiInterference = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x19),//多设备抗干扰
	TOF_DEV_PARAM_JpegDecParam           = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1a),//JPEG图像解码回调的参数
	TOF_DEV_PARAM_TofMpiCorrect          = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1b),//TOF多路径矫正
	TOF_DEV_PARAM_TofMpiFuse             = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1c),//TOF多路径矫正结果和传统算法的融合
	TOF_DEV_PARAM_TofFilterLevel         = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1d),//TOF滤波等级
	TOF_DEV_PARAM_TofModuleInfoCard      = MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1e),//TOF模组信息卡数据
	TOF_DEV_PARAM_TofModuleInfoCardDetail= MAKE_UNIQUE_ID(0x00, 0x00, 0x00, 0x1f),//TOF模组信息卡数据详情

}TOF_DEV_PARAM_TYPE;

typedef struct tagTofDeviceParamV20
{
	TOF_DEV_PARAM_TYPE type;//输入参数，只读
	
	union
	{
		TofDeviceTemperature   struTemperature;//温度信息【当type为TOF_DEV_PARAM_Temperature时有效】
		TofModuleLensParameter struTofLensParameter;//TOF模组内参和畸变【当type为TOF_DEV_PARAM_TofLensParameter时有效】（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
		TofModuleLensParameterV20 struTofLensParameterV20;//TOF模组内参和畸变【当type为TOF_DEV_PARAM_TofLensParameterV20时有效】（V2.0版本）
		RgbModuleLensParameter struRgbLensParameter;//RGB模组内参和畸变【当type为TOF_DEV_PARAM_RgbLensParameter时有效】（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
		RgbModuleLensParameterV20 struRgbLensParameterV20;//Rgb模组内参和畸变【当type为TOF_DEV_PARAM_RgbLensParameterV20时有效】（V2.0版本）
		TofCalibData           struTofCalibData;//TOF模组标定数据【当type为TOF_DEV_PARAM_TofCalibData时有效】
		NetDevInfo_t           stuNetDevData;//网络接入设备信息【当type为TOF_DEV_PARAM_netdevinfo时有效】
		TofCalibData           struReplaceTofCalibData;//替换SDK里TOF模组标定数据【当type为TOF_DEV_PARAM_ReplaceTofCalibData时有效】
		RemoteCapture          struRemoteCapture;//远程抓图：控制模块抓取数据并保存在模块内部；【当type为TOF_DEV_PARAM_RemoteCapture时有效】
		TofRawData             struExportRaw;//导出一帧RAW数据：实时的从模块里导出一帧RAW数据（适用于RAW数据和深度数据异步传输的情况）；【当type为TOF_DEV_PARAM_ExportRaw时有效】
		FirmwareUpgradeData    struFirmware;//固件升级数据【当type为TOF_DEV_PARAM_UpgradeFirmware时有效】
		RebootDev              struRebootDev;//设备重启【当type为TOF_DEV_PARAM_RebootDev时有效】
		StereoLensParameter    struStereoLensParameter;//双目相机参数【当type为TOF_DEV_PARAM_StereoLensParameter时有效】
		MasterSlaveSyncTime    struMasterSlaveSyncTime;//主从机间同步时间【当type为TOF_DEV_PARAM_GetMasterSlaveSyncTime时有效】
		TofAnalogGain          struTofAnalogGain;//TOF模拟增益【当type为TOF_DEV_PARAM_TofAnalogGain时有效】
		TofDigitalGain         struTofDigitalGain;//TOF数字增益【当type为TOF_DEV_PARAM_TofDigitalGain时有效】
		TofFrameDataPixelOffset struPixelOffset;//TOF回调函数里输出的TOF数据相对于RAW数据的像素偏移个数【当type为TOF_DEV_PARAM_TofFrameDataPixelOffset时有效】
		DepthCalRoi            struDepthCalRoi;//深度计算的区域【当type为TOF_DEV_PARAM_DepthCalRoi时有效】
		SensorStatusCtrl       struSensorStatusCtrl;//Sensor状态控制【当type为TOF_DEV_PARAM_SensorStatusCtrl时有效】
		RgbdRegistrationCalibData   struRgbdCalibData;//RGBD配准的标定数据【当type为TOF_DEV_PARAM_RgbdCalibData时有效】
		FastUpgradeFirmware    struFastUpgrade;//快速升级固件（一般需要借助于芯片厂商的升级工具）【当type为TOF_DEV_PARAM_FastUpgradeFirmware时有效】
		TofINSParam            struTofINSParam;//TOF模组杂散光矫正参数【当type为TOF_DEV_PARAM_TofINSParam时有效】
		TofINSCheckCode        struTofINSCheckCode;//TOF模组杂散光矫正参数【当type为TOF_DEV_PARAM_TofINSCheckCode时有效】
		ParamsCheckCode        struTofCalibCheckCode;//TOF模组标定文件的校验码【当type为TOF_DEV_PARAM_TofCalibCheckCode时有效】
		LedCurrent             struLedCurrent;//模组投射器电流【当type为TOF_DEV_PARAM_LedCurrent时有效】
		MultiDevAntiInterference struMultiDevAntiInterference;//多设备抗干扰【当type为TOF_DEV_PARAM_MultiDevAntiInterference时有效】
		JpegDecParam           struJpegDecParam;//JPEG图像解码回调的参数【当type为TOF_DEV_PARAM_JpegDecParam时有效】
		TofMpiCorrect          struTofMpiCorrect;//TOF多路径矫正【当type为TOF_DEV_PARAM_TofMpiCorrect时有效】
		TofMpiFuse             struTofMpiFuse;//TOF多路径矫正结果和传统算法的融合【当type为TOF_DEV_PARAM_TofMpiFuse时有效】
		TofFilterLevel         struTofFilterLevel;//TOF滤波等级【当type为TOF_DEV_PARAM_TofFilterLevel时有效】
		GeneralBlockData       struTofModuleInfoCard;//TOF模组信息卡数据【当type为TOF_DEV_PARAM_TofModuleInfoCard时有效】
		TofModuleInfoCardDetail   struTofModuleInfoCardDetail;//TOF模组信息卡数据详情【当type为TOF_DEV_PARAM_TofModuleInfoCardDetail时有效】
	}uParam;
}TofDeviceParamV20;



typedef enum tagTOFDEV_STATUS
{
	TOFDEV_STATUS_UNUSED                 = MAKE_UNIQUE_ID('U', 'U', 'S', 'E'),//（该值未使用）

	TOFDEV_STATUS_DEV_BROKEN             = MAKE_UNIQUE_ID('D', 'E', 'V', 'B'),//设备异常断开
	//
	TOFDEV_STATUS_READ_CALIB_DATA_SUC    = MAKE_UNIQUE_ID('R', 'C', 'D', 'S'),//读取标定数据成功
	TOFDEV_STATUS_READ_CALIB_DATA_FAILED = MAKE_UNIQUE_ID('R', 'C', 'D', 'F'),//读取标定数据失败
	//
	TOFDEV_STATUS_TOF_STREAM_FAILED      = MAKE_UNIQUE_ID('T', 'S', 'F', 0x00),//取TOF流失败

}TOFDEV_STATUS;

typedef void* HTOFD;


typedef void (*FNTofStream)(TofFrameData *tofFrameData, void* pUserData);
typedef void (*FNTofDeviceStatus)(TOFDEV_STATUS tofDevStatus, void* pUserData);
typedef void (*FNRgbStream)(RgbFrameData *rgbFrameData, void* pUserData);
typedef void (*FNImuStream)(ImuFrameData *imuFrameData, void* pUserData);

#ifdef __cplusplus
extern "C" {
#endif

//初始化/反初始化SDK（其他任何接口的使用都必须介于这两个接口之间，只需要调用一次）
TOFDDLL TOFRET TOFD_Init(TofDevInitParam* pInitParam);
TOFDDLL TOFRET TOFD_Uninit(void);

//获取SDK版本号（返回值为字符串型版本号）
TOFDDLL SCHAR* TOFD_GetSDKVersion(void);

//搜索到系统里SDK支持的所有设备
TOFDDLL TOFRET TOFD_SearchDevice(TofDeviceDescriptor **ppDevsDesc, UINT32* pDevNum);

//打开/关闭设备
TOFDDLL HTOFD  TOFD_OpenDevice(TofDeviceDescriptor *pDevDesc, FNTofDeviceStatus fnTofDevStatus, void* pUserData);
TOFDDLL HTOFD  TOFD_OpenDevice_WithFd(TofDeviceDescriptorWithFd *pDevDesc, FNTofDeviceStatus fnTofDevStatus, void* pUserData);
TOFDDLL TOFRET TOFD_CloseDevice(HTOFD hTofDev);

//获取设备能力
TOFDDLL TOFRET TOFD_GetDeviceInfo(HTOFD hTofDev, TofDeviceInfo *pTofDeviceInfo);

//绑定TOF 模式（关系到后续的出流）
TOFDDLL TOFRET TOFD_SetTofMode(HTOFD hTofDev, const TOF_MODE tofMode);

//获取/设置设备参数（已逐步废弃）
TOFDDLL TOFRET TOFD_GetDeviceParam(HTOFD hTofDev, TofDeviceParam *pTofDeviceParam);
TOFDDLL TOFRET TOFD_SetDeviceParam(HTOFD hTofDev, TofDeviceParam *pTofDeviceParam);

//获取/设置设备参数（V2.0版本）
TOFDDLL TOFRET TOFD_GetDeviceParamV20(HTOFD hTofDev, TofDeviceParamV20 *pTofDeviceParam);
TOFDDLL TOFRET TOFD_SetDeviceParamV20(HTOFD hTofDev, TofDeviceParamV20 *pTofDeviceParam);

//启用/禁用自动设置TOF模组曝光
TOFDDLL TOFRET TOFD_SetTofAE(HTOFD hTofDev, const SBOOL bEnable);

//获取/设置TOF模组曝光
TOFDDLL TOFRET TOFD_SetTofExpTime(HTOFD hTofDev, const UINT32 expTime);
TOFDDLL TOFRET TOFD_GetTofExpTime(HTOFD hTofDev, TofExpouse *pExp);

//获取/启用/禁用某种滤波算法
TOFDDLL TOFRET TOFD_SetTofFilter(HTOFD hTofDev, const TOF_FILTER type, const SBOOL bEnable);
TOFDDLL TOFRET TOFD_GetTofFilter(HTOFD hTofDev, const TOF_FILTER type, SBOOL* pbEnable);

//启用/禁用HDRZ算法
TOFDDLL TOFRET TOFD_SetTofHDRZ(HTOFD hTofDev, const SBOOL bEnable);

//启用/禁用RemoveINS算法
TOFDDLL TOFRET TOFD_SetTofRemoveINS(HTOFD hTofDev, const SBOOL bEnable);

//启动/关闭TOF取流
TOFDDLL TOFRET TOFD_StartTofStream(HTOFD hTofDev, FNTofStream fnTofStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopTofStream(HTOFD hTofDev);

//启动/关闭RGB取流
TOFDDLL TOFRET TOFD_StartRgbStream(HTOFD hTofDev, FNRgbStream fnRgbStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopRgbStream(HTOFD hTofDev);

//启动/关闭IMU取流
TOFDDLL TOFRET TOFD_StartImuStream(HTOFD hTofDev, FNImuStream fnImuStream, void* pUserData);
TOFDDLL TOFRET TOFD_StopImuStream(HTOFD hTofDev);

#ifdef __cplusplus
}
#endif

#endif


