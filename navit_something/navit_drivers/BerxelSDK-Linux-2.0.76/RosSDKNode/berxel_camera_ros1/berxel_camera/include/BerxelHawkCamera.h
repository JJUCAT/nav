#ifndef __BERXEL_HAWK_CAMERA_H__
#define __BERXEL_HAWK_CAMERA_H__

#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include <pthread.h>
#include <queue>

#include "BerxelHawkContext.h"
#include "BerxelHawkDevice.h"
#include "BerxelHawkFrame.h"
#include "BerxelHawkDefines.h"
#include "BerxelCommonFunc.h"

#include <image_transport/image_transport.h>
// #include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf/transform_broadcaster.h>

using namespace berxel;

class BerxelHawkCamera
{
public:
  BerxelHawkCamera(ros::NodeHandle& node);
  ~BerxelHawkCamera();

  int32_t initBerxelCamera();

  void getColorResolution(uint32_t& width, uint32_t& height);
  void getDepthResolution(uint32_t& width, uint32_t& height);
  void getIrResolution(uint32_t& width, uint32_t& height);
  uint32_t getDownsamplingRatio();

  void getRgbDistortionParams(std::vector<double>& d);
  void getIrDistortionParams(std::vector<double>& d);
  void getDepthDistortionParams(std::vector<double>& d);
  void getRgbInternalParameterMatrix(boost::array<double, 9>& k);
  void getIrInternalParameterMatrix(boost::array<double, 9>& k);
  void getDepthInternalParameterMatrix(boost::array<double, 9>& k);
  void getRotateMatrix(boost::array<double, 9>& r);
  void getRgbProjectionMatrix(boost::array<double, 12>& p);
  void getIrProjectionMatrix(boost::array<double, 12>& p);
  void getDepthProjectionMatrix(boost::array<double, 12>& p);

private:
  void advertiseROSTopics();
  int32_t checkConfigureParams();
  int32_t checkResolution(BerxelHawkStreamType type, int32_t width, int32_t height);
  void readDeviceIntriscParams(berxel::BerxelHawkDeviceIntrinsicParams* pParams);
  void destroy();
  int32_t processDeviceStatusChange(const char* deviceUri, const char* deviceSerialNumber,
                                    berxel::BerxelHawkDeviceStatus deviceState);
  void berxelFrameCallback(berxel::BerxelHawkStreamType streamType, berxel::BerxelHawkFrame* pFrame);
  void berxelColorFrameCallback(berxel::BerxelHawkFrame* pFrame);
  void berxelDepthFrameCallback(berxel::BerxelHawkFrame* pDepthFrame, berxel::BerxelHawkFrame* pColorFrame = NULL);
  void berxelIrFrameCallback(berxel::BerxelHawkFrame* pFrame);
  void berxelPublishTF();
  static void onDeviceStatusChange(const char* deviceUri, const char* deviceSerialNumber,
                                   berxel::BerxelHawkDeviceStatus deviceState, void* pUserData);
  static void onNewFrameCallback(berxel::BerxelHawkStreamType streamType, berxel::BerxelHawkFrame* pFrame,
                                 void* pUserData);
  static void* berxelColorPointCloudThread(void* obj);
  void berxelColorPointCloudHandle();

private:
  berxel::BerxelHawkDeviceIntrinsicParams m_DeviceIntrinsicParams;
  berxel::BerxelHawkCameraIntrinsic m_rgbIntrinsicParams;
  berxel::BerxelHawkCameraIntrinsic m_irIntrinsicParams;
  berxel::BerxelHawkCameraIntrinsic m_depthIntrinsicParams;
  berxel::BerxelHawkCameraIntrinsic m_rotaParams;  //

  berxel::BerxelHawkContext* m_pContext;
  berxel::BerxelHawkDevice* m_pHawkDevice;
  berxel::BerxelHawkDeviceInfo m_CurrentDeviceInfo;

  std::queue<berxel::BerxelHawkFrame*> m_queDepthFrame;
  std::queue<berxel::BerxelHawkFrame*> m_queColorFrame;
  pthread_mutex_t m_mutex;
  pthread_t m_nThreadID;

  int m_nStreamFlag;
  int m_nStreadmType;
  int m_nColorWidth;
  int m_nColorHeight;
  int m_nDepthWidth;
  int m_nDepthHeight;
  int m_nIrWidth;
  int m_nIrHeight;
  int m_nDepthFps;
  bool m_bDownsamplining;
  bool m_bRegistration;
  bool m_bPubPointCloud;
  bool m_bOrderedCloudPoint;
  bool m_bThreadStart;
  bool m_bDeviceIsHorizontal;
  bool m_bRosTimeStamp;
  bool m_bPubOriginalDepth;
  bool m_bColorCloudPoint;
  bool m_bSupportRGB;

  float m_camera_link_x;
  float m_camera_link_y;
  float m_camera_link_z;
  float m_camera_link_roll;
  float m_camera_link_pitch;
  float m_camera_link_yaw;
  float m_camera_rgb_frame_x;
  float m_camera_rgb_frame_y;
  float m_camera_rgb_frame_z;
  float m_camera_rgb_frame_roll;
  float m_camera_rgb_frame_pitch;
  float m_camera_rgb_frame_yaw;
  float m_camera_depth_frame_x;
  float m_camera_depth_frame_y;
  float m_camera_depth_frame_z;
  float m_camera_depth_frame_roll;
  float m_camera_depth_frame_pitch;
  float m_camera_depth_frame_yaw;
  float m_camera_rgb_optical_frame_x;
  float m_camera_rgb_optical_frame_y;
  float m_camera_rgb_optical_frame_z;
  float m_camera_rgb_optical_frame_roll;
  float m_camera_rgb_optical_frame_pitch;
  float m_camera_rgb_optical_frame_yaw;
  float m_camera_depth_optical_frame_x;
  float m_camera_depth_optical_frame_y;
  float m_camera_depth_optical_frame_z;
  float m_camera_depth_optical_frame_roll;
  float m_camera_depth_optical_frame_pitch;
  float m_camera_depth_optical_frame_yaw;

  std::string m_strDeviceName;
  std::string m_strSerialNumber;

  berxel::BerxelHawkPoint3D* m_pPointClouds;

  image_transport::CameraPublisher m_pubColor;
  image_transport::CameraPublisher m_pubDepth;
  image_transport::CameraPublisher m_pubIr;
  ros::Publisher m_pubCloudPoint;
  ros::NodeHandle& m_node;

  pthread_mutex_t m_mutex_cb;
  tf::TransformBroadcaster m_pubBroadcaster;
  tf::Transform m_camera_rgb_optical_frame;  // 相机坐标系Cam_rgb
  tf::Transform m_camera_rgb_frame;          // 相机坐标系Cam_rgb
  tf::Transform m_camera_link;               // 相机坐标系Cam_link
  tf::Transform m_camera_depth_frame;
  tf::Transform m_camera_depth_optical_frame;

  // sensor_msgs::PointCloud2 msg_pointcloud;

  // fan add. link names.
  std::string link_prefix_;
  std::string camera_link_;
  std::string camera_rgb_frame_;
  std::string camera_depth_frame_;
  std::string camera_rgb_optical_frame_;
  std::string camera_depth_optical_frame_;
};

#endif