#ifndef __BERXEL_HAWK_DEFINES_H__
#define __BERXEL_HAWK_DEFINES_H__
//
#include <stdint.h>
//#include "BerxelHawkFrame.h"
//
namespace berxel
{

class BerxelHawkFrame;
#define MAX_SN_SIZE 32

typedef enum {
	BERXEL_HAWK_PIXEL_TYPE_IMAGE_RGB24                = 0x00,
	BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_12I_4D           = 0x01, 
	BERXEL_HAWK_PIXEL_TYPE_DEP_16BIT_13I_3D           = 0x02, 
	BERXEL_HAWK_PIXEL_TYPE_IR_16BIT                   = 0x03,
	BERXEL_HAWK_PIXEL_INVALID_TYPE					  = 0xff,
}BerxelHawkPixelType ;



typedef enum {
	BERXEL_HAWK_COLOR_STREAM							= 0x01,
	BERXEL_HAWK_DEPTH_STREAM							= 0x02,
	BERXEL_HAWK_IR_STREAM								= 0x04,
	BERXEL_HAWK_LIGHT_IR_STREAM							= 0x20,
	BERXEL_HAWK_INVALID_STREAM							= 0xff,
}BerxelHawkStreamType;



typedef enum 
{
	BERXEL_HAWK_SINGULAR_STREAM_FLAG_MODE				= 0x01,	
	BERXEL_HAWK_MIX_STREAM_FLAG_MODE				    = 0x02,
	BERXEL_HAWK_MIX_HD_STREAM_FLAG_MODE				    = 0x03,
	BERXEL_HAWK_MIX_QVGA_STREAM_FLAG_MODE				= 0x04,	

}BerxelHawkStreamFlagMode;


typedef enum {
	BERXEL_HAWK_DEVICE_CONNECT 	  = 0x00,
	BERXEL_HAWK_DEVICE_DISCONNECT = 0x01
} BerxelHawkDeviceStatus;



typedef enum {
	BERXEL_HAWK_UPGRADE_SATRT					  = 0x00,
	BERXEL_HAWK_UPGRAD_ENTNER_DFU_MODE_FAILED     = 0x01,
	BERXEL_HAWK_UPGRAD_ENTNER_DFU_SUCCESS		  = 0x02,
	BERXEL_HAWK_UPGRAD_DOWNLOAD_FILE_SUCCEED	  = 0x03,
	BERXEL_HAWK_UPGRAD_DOWNLOAD_FILE_FAILED		  = 0x04,
	BERXEL_HAWK_UPGRAD_PROCESSING		          = 0x05,
	BERXEL_HAWK_UPGRADE_SUCCESS				      = 0x06,
	BERXEL_HAWK_UPGRADE_FAILED				      = 0x07,

}BERXEL_HAWK_UPGRADE_STATUS;

typedef enum
{
	BERXEL_HAWK_DEVICE_UNKNOW = 0x00,
	BERXEL_HAWK_DEVICE_UVC = 0x01,
	BERXEL_HAWK_DEVICE_NET = 0x02,
	BERXEL_HAWK_DEVICE_SONIX = 0x03,
}BERXEL_DEVICE_TYPE;


typedef struct _BerxelHawkDeviceInfo {
	uint16_t vendorId;
	uint16_t productId;
	uint32_t deviceNum;
	uint32_t deviceType;
	uint32_t devBus;
	uint32_t devPort;
	char     serialNumber[MAX_SN_SIZE];
	char     deviceAddress[255];
} BerxelHawkDeviceInfo;


typedef struct _BerxelFrameMode {
	BerxelHawkPixelType pixelType;
	int16_t resolutionX;
	int16_t resolutionY;
	int8_t  framerate;
} BerxelHawkStreamFrameMode;


#pragma pack (push, 1)

typedef struct _BerxelSdkVersion {
	uint16_t major;
	uint16_t minor;
	uint16_t revision;
} BerxelHawkSdkVersion;

typedef struct _BerxelFwVersion {
	uint16_t major;
	uint16_t minor;
	uint16_t revision;
	char chipVersion[64];
} BerxelHawkFwVersion;



typedef struct _BerxelHwVersion {
	uint16_t major;
	uint16_t minor;
	uint16_t revision;
} BerxelHawkHwVersion;

typedef struct {
	BerxelHawkSdkVersion sdkVersion;
	BerxelHawkFwVersion  fwVersion;
	BerxelHawkHwVersion  hwVersion;
} BerxelHawkVersions;


typedef struct _BerxelHawkPoint3D
{
	float x;
	float y;
	float z;
}BerxelHawkPoint3D;


typedef struct _BerxelHawkPoint2D
{
	uint32_t x;
	uint32_t y;

}BerxelHawkPoint2D;


typedef struct _BerxelHawkCameraIntrinsic
{
	float fxParam;  
	float fyParam;  
	float cxParam;  
	float cyParam;  
	float k1Param;  
	float k2Param; 
	float p1Param; 
	float p2Param; 
	float k3Param;  
}BerxelHawkCameraIntrinsic;


typedef struct _BerxelHawkIntrinsicInfo
{
	int8_t   colorIntrinsicParams[36];       //36 bytes 9个float
	int8_t   irIntrinsicParams[36];          //36 bytes 9个float
	int8_t   liteIrIntrinsicParams[36];      //36 bytes 9个float
	int8_t   rotateIntrinsicParams[36];      //36 bytes 9个float
	int8_t   translationIntrinsicParams[12]; //12 bytes 3个float
	
}BerxelHawkDeviceIntrinsicParams;

typedef struct _BerxelHawkNetParams
{
	uint8_t  static_ip;    //0 dhcp client, 1 static ip
	uint32_t ip_addr;
	uint32_t net_mask;
	uint32_t gw_addr;
	uint32_t dns_addr;
	uint8_t  dhcps_enable; //dhcp server, 0 disable 1 enable
	uint32_t dhcps_saddr;
	uint32_t dhcps_eaddr;
	uint8_t  reserved[230];
}BerxelHawkNetParams;

#pragma pack(pop)

#if defined(_WIN32)
typedef void (_stdcall * BerxelHawkNewFrameCallBack) (BerxelHawkStreamType streamType, BerxelHawkFrame* pFrame, void* pUserData);
typedef void (_stdcall * BerxelHawkDeviceStatusChangeCallback) (const char* deviceAddress, const char* deviceSerialNumber, BerxelHawkDeviceStatus deviceState, void* pUserData);
typedef void (_stdcall * BerxelHawkUpgradeProcessCallBack) (BERXEL_HAWK_UPGRADE_STATUS statusID, float progress,   void* pUserData);
#else
typedef void (* BerxelHawkNewFrameCallBack) (BerxelHawkStreamType streamType,BerxelHawkFrame* pFrame,  void* pUserData);
typedef void (* BerxelHawkDeviceStatusChangeCallback) (const char* deviceAddress, const char* deviceSerialNumber, BerxelHawkDeviceStatus deviceState, void* pUserData);
typedef void (* BerxelHawkUpgradeProcessCallBack) (BERXEL_HAWK_UPGRADE_STATUS statusID, float progress, void* pUserData);
#endif

}
//
#endif
