#ifndef __TOF_SDK_TYPEDEF_H__
#define __TOF_SDK_TYPEDEF_H__


typedef unsigned int       UINT32;
typedef unsigned short     UINT16;
typedef unsigned char	   UINT8;
typedef unsigned long long UINT64;

typedef signed int    	   SINT32;
typedef signed short  	   SINT16;
typedef signed char		   SINT8;
typedef signed long long   SINT64;

typedef float              FLOAT32;
typedef double 		       FLOAT64;
typedef bool			   SBOOL;
typedef char			   SCHAR;	


#ifndef TRUE
    #define TRUE 1
#endif

#ifndef FALSE
    #define FALSE 0
#endif

#ifndef NULL
    #define NULL 0
#endif


#define MAKE_UNIQUE_ID(major, sub, a, b) ((major<<24) | (sub<<16) | (a<<8) | (b))

#define TOF_MAX_CAPS_CNT (6) //最大能力个数

#define PROJECT_INFO_WIDELY_RECOMMENDED "FF"

typedef enum tagTOFRET
{
	/** Success (no error) */
	TOFRET_SUCCESS = 0x00000000,
	/** Success (no error, and start to read calib data) */
	TOFRET_SUCCESS_READING_CALIB = 0x00000001,



	/** Input/output error */
	TOFRET_ERROR_IO = 0x80000001,
	/** Invalid parameter */
	TOFRET_ERROR_INVALID_PARAM = 0x80000002,
	/** Access denied (insufficient permissions) */
	TOFRET_ERROR_ACCESS = 0x80000003,
	/** No such device (it may have been disconnected) */
	TOFRET_ERROR_NO_DEVICE = 0x80000004,
	/** Operation timed out */
	TOFRET_ERROR_TIMEOUT = 0x80000005,
	/** Overflow */
	TOFRET_ERROR_OVERFLOW = 0x80000006,
	/** Insufficient memory */
	TOFRET_ERROR_NO_MEM = 0x80000007,
	/** Wrong status */
	TOFRET_ERROR_WRONG_STATUS = 0x80000008,
	/** Operation not supported */
	TOFRET_ERROR_NOT_SUPPORTED = 0x80000009,
	/** Device is in use now */
	TOFRET_ERROR_ALREADY_IN_USE = 0x8000000A,
	/** Error Data */
	TOFRET_ERROR_DATA = 0x8000000B,
	/** Cfg file not found */
	TOFRET_ERROR_CFG_FILE_NOT_FOUND = 0x8000000C,
	/** Read Calib failed */
	TOFRET_ERROR_READ_CALIB_FAILED = 0x8000000D,
	/** Operation need to wait for some time */
	TOFRET_ERROR_NEED_WAIT = 0x8000000E,
	/** Operation failed */
	TOFRET_ERROR_FAILED = 0x8000000F,
	/** Decompress Data failed */
	TOFRET_ERROR_DECOMPRESS_DATA = 0x80000010,
	/** Check crc failed */
	TOFRET_ERROR_CRC_CHECK = 0x80000011,

	/** USB write error */
	TOFRET_ERROR_USB_WRITE = 0x80010001,
	/** USB read error */
	TOFRET_ERROR_USB_READ = 0x80010002,
	/** USB disconnect */
	TOFRET_ERROR_USB_DISCONNECT = 0x80010003,

	/* invalid iic param */
	TOFRET_ERROR_INVALID_IIC_PARAM = 0x80020001,
	/* iic read failed */
	TOFRET_ERROR_IIC_READ_FAILED = 0x80020002,
	/* iic write failed */
	TOFRET_ERROR_IIC_WRITE_FAILED = 0x80020003,

	/** Other error */
	TOFRET_ERROR_OTHER = 0x8FFFFFFF,
}TOFRET;


typedef enum tagTOF_MODE
{
	//双频
	TOF_MODE_STERO_5FPS  = 0x00000001,
	TOF_MODE_STERO_10FPS = 0x00000002,
	TOF_MODE_STERO_15FPS = 0x00000004,
	TOF_MODE_STERO_30FPS = 0x00000008,
	TOF_MODE_STERO_45FPS = 0x00000010,
	TOF_MODE_STERO_60FPS = 0x00000020,

	//单频
	TOF_MODE_MONO_5FPS   = 0x00000040,
	TOF_MODE_MONO_10FPS  = 0x00000080,
	TOF_MODE_MONO_15FPS  = 0x00000100,
	TOF_MODE_MONO_30FPS  = 0x00000200,
	TOF_MODE_MONO_45FPS  = 0x00000400,
	TOF_MODE_MONO_60FPS  = 0x00000800,

	//HDRZ：这几个模式代表具有raw数据的HDRZ融合的
	TOF_MODE_HDRZ_5FPS   = 0x00001000,
	TOF_MODE_HDRZ_10FPS  = 0x00002000,
	TOF_MODE_HDRZ_15FPS  = 0x00004000,
	TOF_MODE_HDRZ_30FPS  = 0x00008000,
	TOF_MODE_HDRZ_45FPS  = 0x00010000,
	TOF_MODE_HDRZ_60FPS  = 0x00020000,

	//帧率不同
	TOF_MODE_5FPS        = 0x00040000,
	TOF_MODE_10FPS       = 0x00080000,
	TOF_MODE_20FPS       = 0x00100000,
	TOF_MODE_30FPS       = 0x00200000,
	TOF_MODE_45FPS       = 0x00400000,
	TOF_MODE_60FPS       = 0x00800000,

	//ADI特定
	TOF_MODE_ADI_1M5     = 0x01000000,
	TOF_MODE_ADI_5M      = 0x02000000,

	//自定义
	TOF_MODE_CUSTOM_1    = 0x04000000,
	TOF_MODE_CUSTOM_2    = 0x08000000,
	TOF_MODE_CUSTOM_3    = 0x10000000,
	TOF_MODE_CUSTOM_4    = 0x20000000,
	TOF_MODE_CUSTOM_5    = 0x40000000,

	//DEBUG模式
	TOF_MODE_DEBUG       = 0x80000000,


}TOF_MODE;


typedef enum tagTOF_FILTER
{
	TOF_FILTER_RemoveFlyingPixel   = 0x00000001,
	TOF_FILTER_AdaptiveNoiseFilter = 0x00000002,
	TOF_FILTER_InterFrameFilter    = 0x00000004,
	TOF_FILTER_PointCloudFilter    = 0x00000008,
	TOF_FILTER_StraylightFilter    = 0x00000010,
	TOF_FILTER_CalcIntensities     = 0x00000020,
	TOF_FILTER_MPIFlagAverage      = 0x00000040,
	TOF_FILTER_MPIFlagAmplitude    = 0x00000080,
	TOF_FILTER_MPIFlagDistance     = 0x00000100,
	TOF_FILTER_ValidateImage       = 0x00000200,
	TOF_FILTER_SparsePointCloud    = 0x00000400,
	TOF_FILTER_Average             = 0x00000800,
	TOF_FILTER_Median              = 0x00001000,
	TOF_FILTER_Confidence          = 0x00002000,
	TOF_FILTER_MPIFilter           = 0x00004000,
	TOF_FILTER_PointCloudCorrect   = 0x00008000,
	TOF_FILTER_LineRecognition     = 0x00010000,
	TOF_FILTER_RadialFusion        = 0x00020000,
	TOF_FILTER_RangeLimited        = 0x00040000,
	TOF_FILTER_Saturation          = 0x00080000,
	TOF_FILTER_StrayLightCorr      = 0x00100000,
	TOF_FILTER_Gauss               = 0x00200000,


}TOF_FILTER;


typedef enum tagEXP_MODE
{
	EXP_MODE_MANUAL = 0x00000001,//手动曝光
	EXP_MODE_AUTO   = 0x00000002,//自动曝光(AE)
}EXP_MODE;


typedef struct tagPointData
{
	FLOAT32 x;
	FLOAT32 y;
	FLOAT32 z;
}PointData;

typedef struct tagRgbDData
{
	UINT8 b;
	UINT8 g;
	UINT8 r;
}RgbDData;

//坐标
typedef struct tagPixelCoordData
{
	UINT16 x;
	UINT16 y;
}PixelCoordData;

typedef enum tagCOLOR_FORMAT
{
	//MJPEG格式
	COLOR_FORMAT_MJPEG    = MAKE_UNIQUE_ID('M', 'J', 'P', 'G'),

	//H264格式
	COLOR_FORMAT_H264     = MAKE_UNIQUE_ID('H', '2', '6', '4'),

	//YUV格式
	COLOR_FORMAT_YUV422   = MAKE_UNIQUE_ID('Y', 'U', 'V', 0x22),
	COLOR_FORMAT_YUYV     = MAKE_UNIQUE_ID('Y', 'U', 'Y', 'V'),
	COLOR_FORMAT_I420     = MAKE_UNIQUE_ID('I', '4', '2', '0'),
	COLOR_FORMAT_YV12     = MAKE_UNIQUE_ID('Y', 'V', '1', '2'),
	COLOR_FORMAT_NV12     = MAKE_UNIQUE_ID('N', 'V', '1', '2'),
	COLOR_FORMAT_NV21     = MAKE_UNIQUE_ID('N', 'V', '2', '1'),

	//RGB格式
	COLOR_FORMAT_BGR      = MAKE_UNIQUE_ID('B', 'G', 'R', 0x00), //RGB24（每个像素占3个字节，按照B、G、R的顺序存放）
	COLOR_FORMAT_RGB      = MAKE_UNIQUE_ID('R', 'G', 'B', 0x00), //RGB24（每个像素占3个字节，按照R、G、B的顺序存放）
	COLOR_FORMAT_BGRA     = MAKE_UNIQUE_ID('B', 'G', 'R', 'A'), //RGB32（每个像素占4个字节，按照B、G、R、A的顺序存放）
	COLOR_FORMAT_RGBA     = MAKE_UNIQUE_ID('R', 'G', 'B', 'A'), //RGB32（每个像素占4个字节，按照R、G、B、A的顺序存放）

}COLOR_FORMAT;


typedef struct tagRgbData
{
	UINT8 r;
	UINT8 g;
	UINT8 b;
}RgbData;


//RGB模组内参和畸变（通用模型）
typedef struct tagRgbModuleLensGeneral
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
}RgbModuleLensGeneral;

//RGB模组内参和畸变（鱼眼模型）
typedef struct tagRgbModuleLensFishEye
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 k3;
	FLOAT32 k4;
}RgbModuleLensFishEye;

//RGB模组内参和畸变（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
typedef struct tagRgbModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}RgbModuleLensParameter;

//RGB模组内参和畸变（V2.0版本）
typedef struct tagRgbModuleLensParameterV20
{
	UINT32 nIndex;//1---general有效, 2---fishEye有效

	union
	{
		//[第1种]: 普通模型
		RgbModuleLensGeneral general;//普通模型

		//[第2种]: 鱼眼模型
		RgbModuleLensFishEye fishEye;//鱼眼模型
	}uParam;

}RgbModuleLensParameterV20;

//双目相机参数
typedef struct tagStereoLensParameter
{
	FLOAT32 szRotationMatrix[3][3];//双目旋转矩阵
	FLOAT32 szTranslationMatrix[3];//双目平移矩阵

}StereoLensParameter;


//TOF Expouse
typedef struct tagTofExpouse
{
	UINT32  	nCurrent;//当前值，可读写
	UINT32  	nMax;//最大值，只读
	UINT32  	nMin;//最小值，只读
}TofExpouse;

//TOF Expouse Current Group1
typedef struct tagTofExpouseCurrentGroup1
{
	UINT32 exp;//曝光值
}TofExpouseCurrentGroup1;


//TOF Expouse Current Group2
typedef struct tagTofExpouseCurrentGroup2
{
	UINT32 exp_AEF;//自动曝光帧曝光值
	UINT32 exp_FEF;//固定曝光帧曝光值
}TofExpouseCurrentGroup2;


//TOF Expouse Current Group3
typedef struct tagTofExpouseCurrentGroup3
{
	UINT32 exp_AEF;//自动曝光帧曝光值
	UINT32 exp_FEF;//固定曝光帧曝光值
	UINT32 exp_Gray;//灰度曝光帧曝光值
}TofExpouseCurrentGroup3;

//TOF Expouse Current Items
typedef struct tagTofExpouseCurrentItems
{
	UINT32 nIndex;//1---g1有效, 2---g2有效, 3---g3有效

	union
	{
		//[第1种]: 仅适用于只有单频或者双频raw数据的时候
		TofExpouseCurrentGroup1 g1;//曝光值

		//[第2种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时）
		TofExpouseCurrentGroup2 g2;//曝光值

		//[第3种]: 仅适用于具有自动曝光帧和固定曝光帧的raw数据的时候（帧内HDRZ融合时），并且还可以配置灰度曝光帧的曝光
		TofExpouseCurrentGroup3 g3;//曝光值
	}uParam;

}TofExpouseCurrentItems;


//TOF Expouse Range Items
typedef struct tagTofExpouseRangeItems
{
	UINT32 min_AEF;//自动曝光帧曝光值(最小)
	UINT32 max_AEF;//自动曝光帧曝光值(最大)

	UINT32 min_FEF;//固定曝光帧曝光值(最小)
	UINT32 max_FEF;//固定曝光帧曝光值(最大)

	UINT32 min_Gray;//灰度曝光帧曝光值(最小)
	UINT32 max_Gray;//灰度曝光帧曝光值(最大)

}TofExpouseRangeItems;

//模组SDK的客户识别号
typedef enum tagMODULE_GUEST_ID
{
	MODULE_GUEST_ID_DEF = 0x00,//默认客户
	MODULE_GUEST_ID_01 = 0x01,//客户01
	MODULE_GUEST_ID_02 = 0x02,//客户02
	MODULE_GUEST_ID_03 = 0x03,//客户03
	MODULE_GUEST_ID_04 = 0x04,//客户04
	MODULE_GUEST_ID_05 = 0x05,//客户05
	MODULE_GUEST_ID_06 = 0x06,//客户06
	MODULE_GUEST_ID_07 = 0x07,//客户07
	MODULE_GUEST_ID_08 = 0x08,//客户08
	MODULE_GUEST_ID_09 = 0x09,//客户09
	MODULE_GUEST_ID_0A = 0x0A,//客户0A
	MODULE_GUEST_ID_0B = 0x0B,//客户0B
	MODULE_GUEST_ID_0C = 0x0C,//客户0C
	MODULE_GUEST_ID_0D = 0x0D,//客户0D
	MODULE_GUEST_ID_0E = 0x0E,//客户0E
	MODULE_GUEST_ID_0F = 0x0F,//客户0F

	MODULE_GUEST_ID_10 = 0x10,//客户10
	MODULE_GUEST_ID_11 = 0x11,//客户11
	MODULE_GUEST_ID_12 = 0x12,//客户12
	MODULE_GUEST_ID_13 = 0x13,//客户13
	MODULE_GUEST_ID_14 = 0x14,//客户14
	MODULE_GUEST_ID_15 = 0x15,//客户15
	MODULE_GUEST_ID_16 = 0x16,//客户16
	MODULE_GUEST_ID_17 = 0x17,//客户17
	MODULE_GUEST_ID_18 = 0x18,//客户18
	MODULE_GUEST_ID_19 = 0x19,//客户19
	MODULE_GUEST_ID_1A = 0x1A,//客户1A
	MODULE_GUEST_ID_1B = 0x1B,//客户1B
	MODULE_GUEST_ID_1C = 0x1C,//客户1C
	MODULE_GUEST_ID_1D = 0x1D,//客户1D
	MODULE_GUEST_ID_1E = 0x1E,//客户1E
	MODULE_GUEST_ID_1F = 0x1F,//客户1F

	MODULE_GUEST_ID_20 = 0x20,//客户20
	MODULE_GUEST_ID_21 = 0x21,//客户21
	MODULE_GUEST_ID_22 = 0x22,//客户22
	MODULE_GUEST_ID_23 = 0x23,//客户23
	MODULE_GUEST_ID_24 = 0x24,//客户24
	MODULE_GUEST_ID_25 = 0x25,//客户25
	MODULE_GUEST_ID_26 = 0x26,//客户26
	MODULE_GUEST_ID_27 = 0x27,//客户27
	MODULE_GUEST_ID_28 = 0x28,//客户28
	MODULE_GUEST_ID_29 = 0x29,//客户29
	MODULE_GUEST_ID_2A = 0x2A,//客户2A
	MODULE_GUEST_ID_2B = 0x2B,//客户2B
	MODULE_GUEST_ID_2C = 0x2C,//客户2C
	MODULE_GUEST_ID_2D = 0x2D,//客户2D
	MODULE_GUEST_ID_2E = 0x2E,//客户2E
	MODULE_GUEST_ID_2F = 0x2F,//客户2F

	MODULE_GUEST_ID_WIDELY_RECOMMENDED = 0xFF,

	MODULE_GUEST_ID_MAX,//

}MODULE_GUEST_ID;

typedef struct tagTofModuleDescriptor
{
	MODULE_GUEST_ID guestID;//客户代号
	SCHAR szProjectInfo[32];//项目代号

	UINT8* pModInfoCardData;//模组信息卡数据
	UINT32 nModInfoCardDataLen;//pModInfoCardData模组信息卡数据，字节数
	SCHAR szExtProperty1[32];//模组信息扩展属性1（可为空）
	SCHAR szExtProperty2[32];//模组信息扩展属性2（可为空）

}TofModuleDescriptor;


typedef struct tagRoiItem
{
	UINT32  left;//起始列，从0开始;
	UINT32  top;//起始行，从0开始;
	UINT32  right;//终止列，不超过图像宽;
	UINT32  bottom;//终止行，不超过图像高;

}RoiItem;


typedef struct tagDepthCalRoi
{
	RoiItem struMax;//最大值，只读
	RoiItem struDefault;//默认值，只读

	RoiItem struCurrent;//当前值，可读写

}DepthCalRoi;


//TOF模组内参和畸变（通用模型）
typedef struct tagTofModuleLensGeneral
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
}TofModuleLensGeneral;

//TOF模组内参和畸变（鱼眼模型）
typedef struct tagTofModuleLensFishEye
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 k3;
	FLOAT32 k4;
}TofModuleLensFishEye;

//TOF模组内参和畸变（V1.0版本，建议不要再用，因为不能适用于鱼眼模型）
typedef struct tagTofModuleLensParameter
{
	FLOAT32 fx;
	FLOAT32 fy;
	FLOAT32 cx;
	FLOAT32 cy;
	FLOAT32 k1;
	FLOAT32 k2;
	FLOAT32 p1;
	FLOAT32 p2;
	FLOAT32 k3;
	//FLOAT32 k4;
}TofModuleLensParameter;

//TOF模组内参和畸变（V2.0版本）
typedef struct tagTofModuleLensParameterV20
{
	UINT32 nIndex;//1---general有效, 2---fishEye有效

	union
	{
		//[第1种]: 普通模型
		TofModuleLensGeneral general;//普通模型

		//[第2种]: 鱼眼模型
		TofModuleLensFishEye fishEye;//鱼眼模型
	}uParam;

}TofModuleLensParameterV20;

//TOF模组标定数据
typedef struct tagTofCalibData
{
	UINT8* pData;//指向标定数据
	UINT32 nDataLen;//pData内标定数据长度
}TofCalibData;

//RGBD配准的标定数据
typedef struct tagRgbdRegistrationCalibData
{
	UINT8* pData;//指向标定数据
	UINT32 nDataLen;//pData内标定数据长度
}RgbdRegistrationCalibData;

//通用数据块
typedef struct tagGeneralBlockData
{
	UINT8* pData;//指向数据
	UINT32 nDataLen;//pData内数据长度
}GeneralBlockData;

typedef struct tagTofRawData
{
	//RAW数据
	UINT8* pRaw;//一帧RAW数据
	UINT32 nRawLen;//RAW数据长度（字节数）

	//RAW数据其他属性参数
	FLOAT32 fTemperature;//出RAW数据时模组温度（注意：部分型号模组不需要该字段、部分模组RAW数据自带该数据，那么可以输入0值）

}TofRawData;


typedef struct tagExterntionHooks
{
	void* pUserData;//用户自定义数据

	/**********用于提前送出计算出来的曝光值**********/
	//@    pExp:  计算出的曝光值信息;
	//@    user_data:  用户自定义数据，与pUserData属于同一个;
	//@    【特别注意】:  对于在该回调函数内调用TOFM_XXX接口时，只允许调用软件算法部分接口，否则会死锁！！！！！
	void(*RecvTofExpTime)(TofExpouseCurrentItems* pExp, void*user_data);//根据模组实际情况选择是否实现


}ExterntionHooks;

//多机干扰检测结果
typedef enum tagMULTI_DEV_INTERFERENCE
{
	MULTI_DEV_INTERFERENCE_NONE = 0, //无干扰
	MULTI_DEV_INTERFERENCE_NOT_DELAY, //有干扰，但是无需调整sensor出流
	MULTI_DEV_INTERFERENCE_NEED_DELAY, //有干扰，需要调整sensor出流
	MULTI_DEV_INTERFERENCE_NEED_DELAY_TIPS, //有干扰，需要调整sensor出流（仅做提示用）

}MULTI_DEV_INTERFERENCE;



#endif //__TYPEDEF_H__


