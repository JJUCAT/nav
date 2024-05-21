#ifdef WIN32

	#include <Windows.h>
	#include <direct.h>
	#include <psapi.h>
	#include <io.h>

	#define R_OK 4
	#define W_OK 2
	#define X_OK 1
	#define F_OK 0

#elif defined LINUX 

	#include <unistd.h>
	#include <sys/stat.h>
	#include <sys/time.h>
	#include <sys/sysinfo.h>
	#include <fcntl.h>

#endif
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <memory>
#include <iostream>
#include <thread>
#include <list>
#include <mutex>
#include <vector>
#include <chrono>
#include <algorithm>
#include "tof_sdk_typedef.h"
#include "tof_dev_sdk.h"

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>


ros::Publisher pub_info;
ros::Publisher pub_pointCloud;
ros::Publisher pub_depthz;
ros::Publisher pub_gray;
ros::Publisher pub_rgbd;
ros::Publisher pub_rgb;

sensor_msgs::CameraInfoPtr camInfoMsg;
sensor_msgs::PointCloud2 cloudMsg;
sensor_msgs::ImagePtr depthzMsg;
sensor_msgs::ImagePtr grayMsg;
sensor_msgs::ImagePtr rgbdMsg;
sensor_msgs::ImagePtr rgbMsg;

pcl::PointCloud<pcl::PointXYZ> pclCloud;

static bool g_signalCtrlC = false;//ctrl-c信号

//
typedef struct tagStreamCallBackParam
{
	TofDeviceInfo struCaps;

	UINT32 tof_frame_count;
	UINT32 rgb_frame_count;
	
}StreamCallBackParam;



#ifdef WIN32
static int gettimeofday(struct timeval *tp, void *tzp)
{
	time_t clock;
	struct tm tm;
	SYSTEMTIME wtm;
	GetLocalTime(&wtm);
	tm.tm_year = wtm.wYear - 1900;
	tm.tm_mon = wtm.wMonth - 1;
	tm.tm_mday = wtm.wDay;
	tm.tm_hour = wtm.wHour;
	tm.tm_min = wtm.wMinute;
	tm.tm_sec = wtm.wSecond;
	tm.tm_isdst = -1;
	clock = mktime(&tm);
	tp->tv_sec = (long)clock;
	tp->tv_usec = wtm.wMilliseconds * 1000;
	return (0);
}
#endif

static unsigned long long Utils_GetTickCount(void)
{
	unsigned long long tick = 0;

#ifdef WIN32
	//tick = GetTickCount();//实际精度只有15ms左右; 返回的是一个32位的无符号整数，Windows连续运行49.710天后，它将再次从零开始计时; 
	//tick = GetTickCount64();//返回一个64位的无符号整数。Windows连续运行5.8亿年后，其计时才会归零; 
	//tick = clock();//该程序从启动到函数调用占用CPU的时间, 是C/C++中的计时函数

	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec / 1000);

	auto timePoint = std::chrono::steady_clock::now(); // std::chrono::time_point
	tick = std::chrono::duration_cast<std::chrono::milliseconds>(timePoint.time_since_epoch()).count();

#elif defined LINUX
	//struct timeval tv;
	//gettimeofday(&tv, 0);
	//tick = (tv.tv_sec * 1000 + tv.tv_usec/1000);

	struct timespec tv;
	clock_gettime(CLOCK_MONOTONIC, &tv);
	tick = (tv.tv_sec * 1000 + tv.tv_nsec / 1000000);

#else
	printf("unknown platform in getting tick cnt, error!\n");
#endif // WIN32

	return tick;
}


static float CalCenterPointDataZAvg(PointData *pPointData, const UINT32 width, const UINT32 height)
{
	if (NULL == pPointData)
	{
		return 0;
	}

	const int start_h = (10<height) ? ((height / 2) - 5) : 0;
	const int end_h = (10<height) ? ((height / 2) + 5) : (height);
	const int start_w = (10<width) ? ((width / 2) - 5) : 0;
	const int end_w = (10<width) ? ((width / 2) + 5) : (width);


	float sum = 0.0;
	int cnt = 0;
	for (int h = start_h; h < end_h; h++)
	{
		PointData *pTmp = pPointData + h*width;
		for (int w = start_w; w < end_w; w++)
		{
			if (0.00001 < pTmp[w].z)
			{
				sum += pTmp[w].z;
				cnt++;
			}
		}
	}

	return ((0 < cnt) ? (sum / cnt) : 0);
}

static const char* TofMode2Str(const TOF_MODE mode)
{
	const char* pStr = "Unknown";

	switch (mode)
	{
	case TOF_MODE_STERO_5FPS: pStr = "STERO_5FPS"; break;
	case TOF_MODE_STERO_10FPS: pStr = "STERO_10FPS"; break;
	case TOF_MODE_STERO_15FPS: pStr = "STERO_15FPS"; break;
	case TOF_MODE_STERO_30FPS: pStr = "STERO_30FPS"; break;
	case TOF_MODE_STERO_45FPS: pStr = "STERO_45FPS"; break;
	case TOF_MODE_STERO_60FPS: pStr = "STERO_60FPS"; break;

	case TOF_MODE_MONO_5FPS: pStr = "MONO_5FPS"; break;
	case TOF_MODE_MONO_10FPS: pStr = "MONO_10FPS"; break;
	case TOF_MODE_MONO_15FPS: pStr = "MONO_15FPS"; break;
	case TOF_MODE_MONO_30FPS: pStr = "MONO_30FPS"; break;
	case TOF_MODE_MONO_45FPS: pStr = "MONO_45FPS"; break;
	case TOF_MODE_MONO_60FPS: pStr = "MONO_60FPS"; break;

	case TOF_MODE_HDRZ_5FPS: pStr = "HDRZ_5FPS"; break;
	case TOF_MODE_HDRZ_10FPS: pStr = "HDRZ_10FPS"; break;
	case TOF_MODE_HDRZ_15FPS: pStr = "HDRZ_15FPS"; break;
	case TOF_MODE_HDRZ_30FPS: pStr = "HDRZ_30FPS"; break;
	case TOF_MODE_HDRZ_45FPS: pStr = "HDRZ_45FPS"; break;
	case TOF_MODE_HDRZ_60FPS: pStr = "HDRZ_60FPS"; break;

	case TOF_MODE_5FPS: pStr = "5FPS"; break;
	case TOF_MODE_10FPS: pStr = "10FPS"; break;
	case TOF_MODE_20FPS: pStr = "20FPS"; break;
	case TOF_MODE_30FPS: pStr = "30FPS"; break;
	case TOF_MODE_45FPS: pStr = "45FPS"; break;
	case TOF_MODE_60FPS: pStr = "60FPS"; break;

	case TOF_MODE_ADI_1M5: pStr = "ADI_1M5"; break;
	case TOF_MODE_ADI_5M: pStr = "ADI_5M"; break;

	case TOF_MODE_CUSTOM_1: pStr = "CUSTOM_1"; break;
	case TOF_MODE_CUSTOM_2: pStr = "CUSTOM_2"; break;
	case TOF_MODE_CUSTOM_3: pStr = "CUSTOM_3"; break;
	case TOF_MODE_CUSTOM_4: pStr = "CUSTOM_4"; break;
	case TOF_MODE_CUSTOM_5: pStr = "CUSTOM_5"; break;

	case TOF_MODE_DEBUG: pStr = "DEBUG"; break;


	default: break;
	}

	return pStr;
}

static SBOOL g_isConnectionBroken = false;//是否有设备断线

static void CallBackTofDeviceStatus(TOFDEV_STATUS tofDevStatus, void *pUserData)
{
	printf("device status: 0x08%x.\n", tofDevStatus);

	if (TOFDEV_STATUS_DEV_BROKEN == tofDevStatus)
	{
		printf("a device connection is broken!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
		g_isConnectionBroken = true;
	}

}

static void publishPointCloud(TofFrameData *tofFrameData, StreamCallBackParam *cbData)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	pclCloud.points.clear();
	pcl::PointXYZ singlePoint;

	for (int i = 0; i < width * height; ++i)
	{
		singlePoint.x = tofFrameData->pPointData[i].x;
		singlePoint.y = tofFrameData->pPointData[i].y;
		singlePoint.z = tofFrameData->pPointData[i].z;

		pclCloud.points.push_back(singlePoint);
	}

	pcl::toROSMsg(pclCloud, cloudMsg);
	std::string frameId = "camera_point_cloud_frame";
	cloudMsg.header.frame_id = frameId.c_str();
	cloudMsg.header.stamp = ros::Time::now();
	pub_pointCloud.publish(cloudMsg);
}

static void publishDepthzImage(TofFrameData *tofFrameData, StreamCallBackParam *cbData)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	cv::Mat depthzMat = cv::Mat(height, width, CV_16UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			depthzMat.at<UINT16>(i, j) = static_cast<unsigned short>(tofFrameData->pPointData[i * width + j].z * 1000);
		}
	}

	depthzMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, depthzMat).toImageMsg();
	std::string frameId = "camera_depthz_frame";
	depthzMsg->header.frame_id = frameId.c_str();
	depthzMsg->header.stamp = ros::Time::now();
	pub_depthz.publish(depthzMsg);
}

static void publishGrayImage(TofFrameData *tofFrameData, StreamCallBackParam *cbData)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;
	UINT8* pGray = tofFrameData->pGrayData;

	cv::Mat grayMat = cv::Mat(height, width, CV_8UC1);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			grayMat.at<UINT8>(i, j) = static_cast<unsigned char>(pGray[i * width + j]);
		}
	}

	grayMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, grayMat).toImageMsg();
	std::string frameId = "camera_gray_frame";
	grayMsg->header.frame_id = frameId.c_str();
	grayMsg->header.stamp = ros::Time::now();
	pub_gray.publish(grayMsg);
}

static void publishRgbdImage(TofFrameData *tofFrameData, StreamCallBackParam *cbData)
{
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	for (UINT32 i = 0; i < height; ++i)
	{
		for (UINT32 j = 0; j < width; ++j)
		{
			rgbMat.at<cv::Vec3b>(i, j)[0] = tofFrameData->pRgbD[i * width + j].b; //B
			rgbMat.at<cv::Vec3b>(i, j)[1] = tofFrameData->pRgbD[i * width + j].g; //G
			rgbMat.at<cv::Vec3b>(i, j)[2] = tofFrameData->pRgbD[i * width + j].r; //R
		}
	}

	rgbdMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, rgbMat).toImageMsg();
	std::string frameId = "camera_rgbd_frame";
	rgbdMsg->header.frame_id = frameId.c_str();
	rgbdMsg->header.stamp = ros::Time::now();
	pub_rgbd.publish(rgbdMsg);
}


static void publishRgbImage(RgbFrameData *rgbFrameData, StreamCallBackParam *cbData)
{
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;
	const UINT32 pixel_cnt = width * height;

	cv::Mat rgbMat = cv::Mat(height, width, CV_8UC3);
	if(COLOR_FORMAT_RGB == rgbFrameData->formatType)
	{
		for (int i = 0; i < pixel_cnt * 3; i += 3)
		{
			rgbMat.data[i + 0] = rgbFrameData->pFrameData[i + 2];//B
			rgbMat.data[i + 1] = rgbFrameData->pFrameData[i + 1];//G
			rgbMat.data[i + 2] = rgbFrameData->pFrameData[i + 0];//R
		}
	}
	else if(COLOR_FORMAT_BGR == rgbFrameData->formatType)
	{
		for (int i = 0; i < pixel_cnt * 3; i += 3)
		{
			rgbMat.data[i + 0] = rgbFrameData->pFrameData[i + 0];//B
			rgbMat.data[i + 1] = rgbFrameData->pFrameData[i + 1];//G
			rgbMat.data[i + 2] = rgbFrameData->pFrameData[i + 2];//R
		}
	}
	else
	{
		printf("rgb formatType=%d, it is need to write publishing code by yourself.\n", rgbFrameData->formatType);
		return;
	}

	rgbMsg = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, rgbMat).toImageMsg();
	std::string frameId = "camera_rgb_frame";
	rgbMsg->header.frame_id = frameId.c_str();
	rgbMsg->header.stamp = ros::Time::now();
	pub_rgb.publish(rgbMsg);
}

static void PublishTofFrame(TofFrameData *tofFrameData, StreamCallBackParam *cbData)
{
	if (NULL != tofFrameData->pPointData)
	{
		publishPointCloud(tofFrameData, cbData);
		publishDepthzImage(tofFrameData, cbData);
	}

	if (NULL != tofFrameData->pGrayData)
	{
		publishGrayImage(tofFrameData, cbData);
	}

	if (NULL != tofFrameData->pRgbD)
	{
		publishRgbdImage(tofFrameData, cbData);
	}
}
static void PublishRgbFrame(RgbFrameData *rgbFrameData, StreamCallBackParam *cbData)
{
	if (NULL != rgbFrameData->pFrameData)
	{
		publishRgbImage(rgbFrameData, cbData);
	}
}

static void fnTofStream(TofFrameData *tofFrameData, void* pUserData)
{
	//特别注意：因为有些模块的tofFrameData里的各个指针仅在回调函数内有效，
	//所以，如果要将tofFrameData数据放到其他线程里去处理，必须将tofFrameData里各个指针对应的内存内容复制一份！！！！！！

	StreamCallBackParam* pStreamParam = (StreamCallBackParam*)pUserData;
	(pStreamParam->tof_frame_count)++;

	//printf("szDevId=%s.\n", pStreamParam->struCaps.szDevId);//szDevId可用于区分是哪个模块

	//
	const UINT32 width = tofFrameData->frameWidth;
	const UINT32 height = tofFrameData->frameHeight;

	const UINT32 frame_count = pStreamParam->tof_frame_count;
	const UINT64 host_ts = Utils_GetTickCount();//主机时间
	const UINT64 dev_ts = tofFrameData->timeStamp;//设备时间

	//
	const float depthZ = CalCenterPointDataZAvg(tofFrameData->pPointData, tofFrameData->frameWidth, tofFrameData->frameHeight);
	printf("[%u], TOF frame, host_ts=%llu, dev_ts=%llu, depthZ=%0.3fm.\n", frame_count, host_ts, dev_ts, depthZ);

	PublishTofFrame(tofFrameData, pStreamParam);
}

static void fnRgbStream(RgbFrameData *rgbFrameData, void* pUserData)
{
	//特别注意：因为有些模块的rgbFrameData里的各个指针仅在回调函数内有效，
	//所以，如果要将rgbFrameData数据放到其他线程里去处理，必须将rgbFrameData里各个指针对应的内存内容复制一份！！！！！！

	StreamCallBackParam* pStreamParam = (StreamCallBackParam*)pUserData;
	(pStreamParam->rgb_frame_count)++;

	//printf("szDevId=%s.\n", pStreamParam->struCaps.szDevId);//szDevId可用于区分是哪个模块

	//
	const UINT32 width = rgbFrameData->frameWidth;
	const UINT32 height = rgbFrameData->frameHeight;

	const UINT32 frame_count = pStreamParam->rgb_frame_count;
	const UINT64 host_ts = Utils_GetTickCount();//主机时间
	const UINT64 dev_ts = rgbFrameData->timeStamp;//设备时间

	//
	printf("[%u], RGB frame, host_ts=%llu, dev_ts=%llu, formatType=0x%08x, 0x%08x.\n", frame_count, host_ts, dev_ts, rgbFrameData->formatType, rgbFrameData->formatTypeOrg);

	PublishRgbFrame(rgbFrameData, pStreamParam);
}

static void PrintDevInfo(TofDeviceInfo *pTofDeviceInfo)
{
	printf("Dev Info:==================================\n");
	printf(">>  szDevName=%s.\n", pTofDeviceInfo->szDevName);
	printf(">>  szDevId=%s.\n", pTofDeviceInfo->szDevId);
	printf(">>  szFirmwareVersion=%s.\n", pTofDeviceInfo->szFirmwareVersion);
	printf("Dev Info==================================\n\n");
}


static TOF_MODE ChoseTofMode(TofDeviceInfo& struCaps)
{
	//打印出所有支持的TOF模式，供用户选择
	printf("chose tof mode from list: \n");
	printf(">>  number: mode.\n");
	for (UINT32 i = 0; i < struCaps.capCnt; i++)
	{
		printf(">>  %u: %s.\n", i, TofMode2Str(struCaps.cap[i].tofMode));
	}

	if (1 == struCaps.capCnt)//只有一种模式，直接返回，省去输入的过程
	{
		return struCaps.cap[0].tofMode;
	}

#if 0
	//这里简单处理，选择第一种支持的模式
	return struCaps.cap[0].tofMode;
#else
	//用于选择哪一种tof模式
	while (1)
	{
		printf("input mode (number) >>");

		std::string strInput;
		std::cin >> strInput;

		const UINT32 i = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if (struCaps.capCnt > i)
		{
			return struCaps.cap[i].tofMode;
		}
		else
		{
			printf("the number is invalid.\n");
		}
	}
#endif

}


static bool OpenStream(HTOFD hTofD, TofDeviceCapability* pCaps, StreamCallBackParam* pStreamParam)
{
	if (pCaps->bTofSupported)//支持TOF的情况下
	{
		pStreamParam->tof_frame_count = 0;
		TOFRET retVal = TOFD_StartTofStream(hTofD, fnTofStream, pStreamParam);
		if ((TOFRET_SUCCESS != retVal) && (TOFRET_SUCCESS_READING_CALIB != retVal))
		{
			printf("start TOF stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		pStreamParam->rgb_frame_count = 0;
		TOFRET retVal = TOFD_StartRgbStream(hTofD, fnRgbStream, pStreamParam);
		if (TOFRET_SUCCESS != retVal)
		{
			printf("start RGB stream, [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", retVal);
			return false;
		}
	}

	return true;

}

static void CloseStream(HTOFD hTofD, TofDeviceCapability* pCaps)
{
	if (pCaps->bTofSupported)//支持TOF的情况下
	{
		TOFD_StopTofStream(hTofD);
	}

	if (pCaps->bRgbSupported)//支持RGB的情况下
	{
		TOFD_StopRgbStream(hTofD);
	}
}

static void ThreadPublishCameraInfo(HTOFD hTofD, TofDeviceCapability *pCaps, bool* bPublish)
{
	TOFRET retVal = TOFRET_ERROR_OTHER;

	TofDeviceParamV20 struParam;
	memset(&struParam, 0, sizeof(struParam));
	struParam.type = TOF_DEV_PARAM_TofLensParameterV20;
	if (TOFRET_SUCCESS != (retVal = TOFD_GetDeviceParamV20(hTofD, &struParam)))
	{
		printf("TOFD_GetDeviceParamV20(TOF_DEV_PARAM_TofLensParameterV20) failed, retVal=0x%08x.\n", retVal);
		return;
	}

	TofModuleLensParameterV20* pTofLens = &(struParam.uParam.struTofLensParameterV20);
	const UINT32 nIndex = pTofLens->nIndex;


	camInfoMsg->header.frame_id = "camera_info_frame";
	camInfoMsg->width = pCaps->tofResWidth;
	camInfoMsg->height = pCaps->tofResHeight;
	camInfoMsg->distortion_model = "plumb_bob";//指定了相机畸变模型，对于大多数相机,"plumb_bob"简单的径向和切向畸变模型就足够了


	if (1 == nIndex)
	{
		TofModuleLensGeneral* pTmp = &(pTofLens->uParam.general);

		camInfoMsg->D = std::vector<double>{pTmp->k1, pTmp->k2, pTmp->p1, pTmp->p2, pTmp->k3};
		camInfoMsg->K = boost::array<double, 9ul>{pTmp->fx, 0, pTmp->cx, 0, pTmp->fy, pTmp->cy, 0, 0, 1};
		camInfoMsg->R = boost::array<double, 9ul>{1, 0, 0, 0, 1, 0, 0, 0, 1};
		camInfoMsg->P = boost::array<double, 12ul>{pTmp->fx, 0, pTmp->cx, 0, 0, pTmp->fy, pTmp->cy, 0, 0, 0, 1, 0};
	}
	else if (2 == nIndex)
	{
		TofModuleLensFishEye* pTmp = &(pTofLens->uParam.fishEye);

		camInfoMsg->D = std::vector<double>{pTmp->k1, pTmp->k2, pTmp->k3, pTmp->k4, 0};
		camInfoMsg->K = boost::array<double, 9ul>{pTmp->fx, 0, pTmp->cx, 0, pTmp->fy, pTmp->cy, 0, 0, 1};
		camInfoMsg->R = boost::array<double, 9ul>{1, 0, 0, 0, 1, 0, 0, 0, 1};
		camInfoMsg->P = boost::array<double, 12ul>{pTmp->fx, 0, pTmp->cx, 0, 0, pTmp->fy, pTmp->cy, 0, 0, 0, 1, 0};
	}
	else
	{
		printf("Lens Paramter (index=%u):...............................\n", nIndex);
		printf(">>   unknown, not supported.\n");
		return;
	}

	while (*bPublish)
	{
		camInfoMsg->header.stamp = ros::Time::now();
		pub_info.publish(camInfoMsg);

		std::this_thread::sleep_for(std::chrono::milliseconds(100)); //单位是毫秒
	}

}

static TofDeviceCapability* GetTofModeCaps(TofDeviceInfo& struCaps, const TOF_MODE tofMode)
{
	for (UINT32 i = 0; ((i <struCaps.capCnt) && (i<TOF_MAX_CAPS_CNT)); i++)
	{
		if (tofMode == struCaps.cap[i].tofMode)
		{
			return (&(struCaps.cap[i]));
		}
	}

	return NULL;
}

static void ThreadTestDemo(HTOFD hTofD, std::string strSaveDir)
{
	TofDeviceInfo struCaps;
	memset(&struCaps, 0, sizeof(struCaps));
	TOFD_GetDeviceInfo(hTofD, &struCaps);
	PrintDevInfo(&struCaps);

	const TOF_MODE tofMode = ChoseTofMode(struCaps);//选择其中一种TOF模式出TOF数据

	TOFRET retVal = TOFD_SetTofMode(hTofD, tofMode);
	if (TOFRET_SUCCESS != retVal)
	{
		printf("set tof mode (0x%08x), [ FAILED ], retVal=0x%08x!!!!!!!!!!!!!!!!!!!!!\n\n", tofMode, retVal);
		return;
	}

	TofDeviceCapability* pCaps = GetTofModeCaps(struCaps, tofMode);//获取tof mode对应的能力

	//
	StreamCallBackParam streamParam;
	memset(&streamParam, 0, sizeof(streamParam));
	memcpy(&streamParam.struCaps, &struCaps, sizeof(struCaps));
	const bool bSuc = OpenStream(hTofD, pCaps, &streamParam);
	if (bSuc)
	{
	}

	//线程里定时发布camerainfo,需要在开流之后才行
	bool bPublishCameraInfo = true;
	std::thread thread_publish_camera_info = std::thread(ThreadPublishCameraInfo, hTofD, pCaps, &bPublishCameraInfo);


	//等待释放该设备资源的信号
	while ((!g_signalCtrlC) && ros::ok() && (!g_isConnectionBroken))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(50)); //单位是毫秒
	}

	bPublishCameraInfo = false;
	thread_publish_camera_info.join();

	//等待线程退出
	CloseStream(hTofD, pCaps);

}


static void DoTestDemo(TofDeviceDescriptor* pDevsDesc, std::string& strSaveDir)
{
	HTOFD hTofD = TOFD_OpenDevice(pDevsDesc, CallBackTofDeviceStatus, NULL);
	if (NULL == hTofD)
	{
		printf("Open Tof Device failed.\n");
		return;
	}

	std::thread thread_test = std::thread(ThreadTestDemo, hTofD, strSaveDir);
	thread_test.join();

	TOFD_CloseDevice(hTofD);

}

static UINT32 ChoseDev(const UINT32  dev_cnt)
{
	const UINT32 min_index = 1;
	const UINT32 max_index = dev_cnt;

	if (1 == max_index)//只有一个设备的话就不用选择了
	{
		return max_index;
	}

	UINT32 dev_index = 1;
	while (1)
	{
		printf("please chose a dev (min=%d, max:%d):\n", min_index, max_index);
		printf(">>");

		std::string strInput;
		std::cin >> strInput;

		dev_index = (UINT32)strtol(strInput.c_str(), NULL, 10);
		if ((min_index <= dev_index) && (max_index >= dev_index))
		{
			break;
		}
		printf("invalid dev index:%d.\n", dev_index);
	}

	return dev_index;
}

static void HandleSignal(int sig)
{
	printf("recieve signal:%d.\n", sig);

	switch (sig)
	{
	case SIGINT:
		g_signalCtrlC = true;
		printf("  ctrl+c signal ...\n");
		break;
	}
}

/*
用例功能简述：选择某一个设备的某一种模式后，进行取流测试，设备断开后会自动重连，适用linux下的ROS系统。
*/
int main(int argc, char **argv)
{
	printf("*********************start test*************************\n");

	g_signalCtrlC = false;
	signal(SIGINT, HandleSignal);

	//初始化节点
	ros::init(argc, argv, "publisher_node");

	ros::NodeHandle node_handle;

	//考虑到兼容所有类型模块，以及为了与重连的demo代码尽可能保持一致，因此默认创建所有发布话题，话题可按需屏蔽
	//init camera info publisher
	pub_info = node_handle.advertise<sensor_msgs::CameraInfo>("sunny_topic/camera_info", 5);
	camInfoMsg.reset(new sensor_msgs::CameraInfo);

	//init point clouds publisher
	pub_pointCloud = node_handle.advertise<sensor_msgs::PointCloud2>("sunny_topic/tof_frame/pointcloud", 5);
	
	//init depthz publisher
	pub_depthz = node_handle.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/depthz", 5);
	depthzMsg.reset(new sensor_msgs::Image);
	
	//init gray publisher
	pub_gray = node_handle.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/gray", 5);
	grayMsg.reset(new sensor_msgs::Image);

	//init rgbd publisher
	pub_rgbd = node_handle.advertise<sensor_msgs::Image>("sunny_topic/tof_frame/rgbd", 5);
	rgbdMsg.reset(new sensor_msgs::Image);

	//init rgb publisher
	pub_rgb = node_handle.advertise<sensor_msgs::Image>("sunny_topic/rgb_frame/rgb", 5);
	rgbMsg.reset(new sensor_msgs::Image);


	std::string strSaveDir = ("");//(".");//用于保存文件的目录，可以自己指定，为空则表示不保存文件

	TofDevInitParam struInitParam;
	memset(&struInitParam, 0, sizeof(struInitParam));
	strncpy(struInitParam.szDepthCalcCfgFileDir, "/home/sunny/catkin_ws/devel/lib/tof_dev_sdk_demo/parameter", sizeof(struInitParam.szDepthCalcCfgFileDir) - 1);
	struInitParam.bSupUsb = true;
	struInitParam.bSupNetWork = false;
	struInitParam.bSupSerialCOM = false;
	if (struInitParam.bSupSerialCOM)
	{
#ifdef WIN32
		//strncpy(struInitParam.szSerialDev, "COM1", sizeof(struInitParam.szSerialDev));//windows下可以不用赋值
#elif defined LINUX 
		strncpy(struInitParam.szSerialDev, "/dev/ttyUSB0", sizeof(struInitParam.szSerialDev));//linux下必须赋值一个实际使用的串口设备，这里随便写了一个
#endif
	}
	struInitParam.bWeakAuthority = false;
	struInitParam.bDisablePixelOffset = false;
	strncpy(struInitParam.szLogFile, "./tof_dev_sdk_log.txt", sizeof(struInitParam.szLogFile));//不赋值则不记录日志到文件
	struInitParam.nLogFileMaxSize = 10 * 1024 * 1024;
	TOFD_Init(&struInitParam);

	printf("SDK Version: %s.\n", TOFD_GetSDKVersion());

	while ((!g_signalCtrlC) && ros::ok())
	{
		g_isConnectionBroken = false;

		TofDeviceDescriptor* pDevsDescList = NULL;
		UINT32 dev_num = 0;
		TOFD_SearchDevice(&pDevsDescList, &dev_num);
		if (0 < dev_num)
		{
			const UINT32 dev_index = ChoseDev(dev_num) - 1;//决定测试哪一个设备
			DoTestDemo(pDevsDescList + dev_index, strSaveDir);

			printf("device connection is broken, try to reconnect after a few seconds...\n");
		}
		else
		{
			printf("can not find tof device, try again after a few seconds...\n");
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(2000)); //单位是毫秒
	}

	TOFD_Uninit();

	printf("*********************stop test*********************\n");

#ifdef WIN32 //防止控制台自动退出而看不到历史日志信息
	printf("please input anything to finish....");
	system("pause");
#endif

	return 0;
}




