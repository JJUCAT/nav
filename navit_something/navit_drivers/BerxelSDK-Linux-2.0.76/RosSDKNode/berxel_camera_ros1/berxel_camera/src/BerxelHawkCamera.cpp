#include "BerxelHawkCamera.h"
#include <unistd.h>
#define PI 3.1415926535897931
BerxelHawkCamera::BerxelHawkCamera(ros::NodeHandle& node)
  : m_node(node)
  , m_pContext(NULL)
  , m_pHawkDevice(NULL)
  , m_bThreadStart(false)
  , m_bDeviceIsHorizontal(false)
  , m_bRegistration(false)
  , m_bPubPointCloud(false)
  , m_bOrderedCloudPoint(false)
  , m_bPubOriginalDepth(false)
  , m_bColorCloudPoint(false)
  , m_bSupportRGB(true)
  , m_nDepthFps(30)
  , m_nStreamFlag(0x01)
  , m_nStreadmType(0x01)
  , m_nColorWidth(400)
  , m_nColorHeight(640)
  , m_nDepthWidth(400)
  , m_nDepthHeight(640)
  , m_nIrWidth(400)
  , m_nIrHeight(640)
{
  node.param("stream_flag", m_nStreamFlag, 0x01);
  node.param("stream_type", m_nStreadmType, 0x01);
  node.param("color_width", m_nColorWidth, 400);
  node.param("color_height", m_nColorHeight, 640);
  node.param("depth_width", m_nDepthWidth, 400);
  node.param("depth_height", m_nDepthHeight, 640);
  node.param("ir_width", m_nIrWidth, 400);
  node.param("ir_height", m_nIrHeight, 640);
  node.param("depth_fps", m_nDepthFps, 30);

  int nValue = 0;
  node.param("Registration", nValue, 0);
  m_bRegistration = (nValue == 1 ? true : false);
  node.param("publish_cloud_point", nValue, 0);
  m_bPubPointCloud = (nValue == 1 ? true : false);
  node.param("color_cloud_point", nValue, 0);
  m_bColorCloudPoint = (nValue == 1 ? true : false);
  node.param("publish_depth_original_raw", nValue, 0);
  m_bPubOriginalDepth = (nValue == 1 ? true : false);
  node.param("ordered_cloud", nValue, 0);
  m_bOrderedCloudPoint = (nValue == 1 ? true : false);
  node.param("time_stamp", nValue, 0);
  m_bRosTimeStamp = (nValue == 1 ? true : false);

  node.param<std::string>("serial_no", m_strSerialNumber, "");
  node.param<std::string>("camera", m_strDeviceName, "berxel_camera");

  // fan add.
  link_prefix_ = m_strDeviceName + "_";
  camera_link_ = link_prefix_ + "link";
  camera_rgb_frame_ = link_prefix_ + "rgb_frame";
  camera_depth_frame_ = link_prefix_ + "depth_frame";
  camera_rgb_optical_frame_ = link_prefix_ + "rgb_optical_frame";
  camera_depth_optical_frame_ = link_prefix_ + "depth_optical_frame";

  node.param("camera_link_x", m_camera_link_x, 0.0f);
  node.param("camera_link_y", m_camera_link_y, 0.0f);
  node.param("camera_link_z", m_camera_link_z, 0.0f);
  node.param("camera_link_roll", m_camera_link_roll, 0.0f);
  node.param("camera_link_pitch", m_camera_link_pitch, 0.0f);
  node.param("camera_link_yaw", m_camera_link_yaw, 0.0f);
  node.param("camera_rgb_frame_x", m_camera_rgb_frame_x, 0.0f);
  node.param("camera_rgb_frame_y", m_camera_rgb_frame_y, 0.0f);
  node.param("camera_rgb_frame_z", m_camera_rgb_frame_z, 0.0f);
  node.param("camera_rgb_frame_roll", m_camera_rgb_frame_roll, 0.0f);
  node.param("camera_rgb_frame_pitch", m_camera_rgb_frame_pitch, 0.0f);
  node.param("camera_rgb_frame_yaw", m_camera_rgb_frame_yaw, 0.0f);
  node.param("camera_depth_frame_x", m_camera_depth_frame_x, 0.0f);
  node.param("camera_depth_frame_y", m_camera_depth_frame_y, 0.0f);
  node.param("camera_depth_frame_z", m_camera_depth_frame_z, 0.0f);
  node.param("camera_depth_frame_roll", m_camera_depth_frame_roll, 0.0f);
  node.param("camera_depth_frame_pitch", m_camera_depth_frame_pitch, 0.0f);
  node.param("camera_depth_frame_yaw", m_camera_depth_frame_yaw, 0.0f);
  node.param("camera_rgb_optical_frame_x", m_camera_rgb_optical_frame_x, 0.0f);
  node.param("camera_rgb_optical_frame_y", m_camera_rgb_optical_frame_y, 0.0f);
  node.param("camera_rgb_optical_frame_z", m_camera_rgb_optical_frame_z, 0.0f);
  node.param("camera_rgb_optical_frame_roll", m_camera_rgb_optical_frame_roll, 0.0f);
  node.param("camera_rgb_optical_frame_pitch", m_camera_rgb_optical_frame_pitch, 0.0f);
  node.param("camera_rgb_optical_frame_yaw", m_camera_rgb_optical_frame_yaw, 0.0f);
  node.param("camera_depth_optical_frame_x", m_camera_depth_optical_frame_x, 0.0f);
  node.param("camera_depth_optical_frame_y", m_camera_depth_optical_frame_y, 0.0f);
  node.param("camera_depth_optical_frame_z", m_camera_depth_optical_frame_z, 0.0f);
  node.param("camera_depth_optical_frame_roll", m_camera_depth_optical_frame_roll, 0.0f);
  node.param("camera_depth_optical_frame_pitch", m_camera_depth_optical_frame_pitch, 0.0f);
  node.param("camera_depth_optical_frame_yaw", m_camera_depth_optical_frame_yaw, 0.0f);

  tf::Quaternion camera_q;
  camera_q.setRPY(m_camera_link_roll * PI, m_camera_link_pitch * PI, m_camera_link_yaw * PI);  // 设置旋转坐标
  m_camera_link.setRotation(camera_q);
  m_camera_link.setOrigin(tf::Vector3(m_camera_link_x, m_camera_link_y, m_camera_link_z));

  tf::Quaternion rgb_q;
  rgb_q.setRPY(m_camera_rgb_frame_roll * PI, m_camera_rgb_frame_pitch * PI,
               m_camera_rgb_frame_yaw * PI);  // 设置旋转坐标
  m_camera_rgb_frame.setRotation(rgb_q);
  m_camera_rgb_frame.setOrigin(tf::Vector3(m_camera_rgb_frame_x, m_camera_rgb_frame_y, m_camera_rgb_frame_z));

  tf::Quaternion rgb_optical_q;
  rgb_optical_q.setRPY(m_camera_rgb_optical_frame_roll * PI, m_camera_rgb_optical_frame_pitch * PI,
                       m_camera_rgb_optical_frame_yaw * PI);  // 设置旋转坐标
  m_camera_rgb_optical_frame.setRotation(rgb_optical_q);
  m_camera_rgb_optical_frame.setOrigin(
      tf::Vector3(m_camera_rgb_optical_frame_x, m_camera_rgb_optical_frame_y, m_camera_rgb_optical_frame_z));

  tf::Quaternion detpth_q;
  detpth_q.setRPY(m_camera_depth_frame_roll * PI, m_camera_depth_frame_pitch * PI,
                  m_camera_depth_frame_yaw * PI);  // 设置旋转坐标
  m_camera_depth_frame.setRotation(detpth_q);
  m_camera_depth_frame.setOrigin(tf::Vector3(m_camera_depth_frame_x, m_camera_depth_frame_y, m_camera_depth_frame_z));

  tf::Quaternion detpth__optical_q;
  detpth__optical_q.setRPY(m_camera_depth_optical_frame_roll * PI, m_camera_depth_optical_frame_pitch * PI,
                           m_camera_depth_optical_frame_yaw * PI);  // 设置旋转坐标
  m_camera_depth_optical_frame.setRotation(detpth__optical_q);
  m_camera_depth_optical_frame.setOrigin(
      tf::Vector3(m_camera_depth_optical_frame_x, m_camera_depth_optical_frame_y, m_camera_depth_optical_frame_z));

  advertiseROSTopics();

  pthread_mutex_init(&m_mutex_cb, NULL);
  m_pPointClouds =
      (berxel::BerxelHawkPoint3D*)malloc(m_nDepthWidth * m_nDepthHeight * sizeof(berxel::BerxelHawkPoint3D));
}

BerxelHawkCamera::~BerxelHawkCamera()
{
  pthread_join(m_nThreadID, NULL);
  pthread_mutex_destroy(&m_mutex_cb);

  destroy();

  if (m_pPointClouds)
  {
    free(m_pPointClouds);
    m_pPointClouds = NULL;
  }
}

int32_t BerxelHawkCamera::checkConfigureParams()
{
  if (m_bSupportRGB == false)
  {
    if (m_nStreamFlag != 1 || (BerxelHawkStreamType)m_nStreadmType != BERXEL_HAWK_DEPTH_STREAM)
    {
      ROS_ERROR("Current Device Only Support Depth!!!");
      return -1;
    }
  }

  if (m_nStreamFlag != BERXEL_HAWK_SINGULAR_STREAM_FLAG_MODE && m_nStreamFlag != BERXEL_HAWK_MIX_STREAM_FLAG_MODE &&
      m_nStreamFlag != BERXEL_HAWK_MIX_HD_STREAM_FLAG_MODE && m_nStreamFlag != BERXEL_HAWK_MIX_QVGA_STREAM_FLAG_MODE)
  {
    ROS_ERROR("Set stream flag error, please check flag : %d", m_nStreamFlag);
  }

  if (m_nStreamFlag == 1)
  {
    if ((BerxelHawkStreamType)m_nStreadmType != BERXEL_HAWK_IR_STREAM &&
        (BerxelHawkStreamType)m_nStreadmType != BERXEL_HAWK_COLOR_STREAM &&
        (BerxelHawkStreamType)m_nStreadmType != BERXEL_HAWK_DEPTH_STREAM)
    {
      ROS_ERROR("Set stream_type failed , Singular mode not support stream : %d", m_nStreadmType);
      return -1;
    }

    if ((BerxelHawkStreamType)m_nStreadmType == BERXEL_HAWK_IR_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_IR_STREAM, m_nIrWidth, m_nIrHeight) != 0)
      {
        return -1;
      }
    }

    if ((BerxelHawkStreamType)m_nStreadmType == BERXEL_HAWK_COLOR_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_COLOR_STREAM, m_nColorWidth, m_nColorHeight) != 0)
      {
        return -1;
      }
    }

    if ((BerxelHawkStreamType)m_nStreadmType == BERXEL_HAWK_DEPTH_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_DEPTH_STREAM, m_nDepthWidth, m_nDepthHeight) != 0)
      {
        return -1;
      }
    }
  }
  else if (m_nStreamFlag == 2 || m_nStreamFlag == 3 || m_nStreamFlag == 4)
  {
    if (m_nStreadmType & BERXEL_HAWK_IR_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_IR_STREAM, m_nIrWidth, m_nIrHeight) != 0)
      {
        return -1;
      }
    }

    if (m_nStreadmType & BERXEL_HAWK_COLOR_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_COLOR_STREAM, m_nColorWidth, m_nColorHeight) != 0)
      {
        return -1;
      }
    }

    if (m_nStreadmType & BERXEL_HAWK_DEPTH_STREAM)
    {
      if (checkResolution(BERXEL_HAWK_DEPTH_STREAM, m_nDepthWidth, m_nDepthHeight) != 0)
      {
        return -1;
      }
    }
  }
  else
  {
    ROS_ERROR("Set stream_flag error, not support : %d", m_nStreamFlag);
    return -1;
  }

  return 0;
}

int32_t BerxelHawkCamera::checkResolution(BerxelHawkStreamType type, int32_t width, int32_t height)
{
  const BerxelHawkStreamFrameMode* pDepthModeList = NULL;
  uint32_t nDepthModeListLen = 0;
  m_pHawkDevice->getSupportFrameModes(type, &pDepthModeList, &nDepthModeListLen);
  if (pDepthModeList[0].resolutionX > pDepthModeList[0].resolutionY)
  {
    m_bDeviceIsHorizontal = true;
  }
  else
  {
    m_bDeviceIsHorizontal = false;
  }

  if (m_bDeviceIsHorizontal)
  {
    if (height >= width)
    {
      ROS_ERROR("Current Device is horizontal, please set correct resolution!!!");
      return -1;
    }
  }
  else
  {
    if (height <= width)
    {
      ROS_ERROR("Current Device is vertical, please set correct resolution!!!");
      return -1;
    }
  }

  bool bFound = false;
  for (int i = 0; i < nDepthModeListLen; i++)
  {
    if (width == pDepthModeList[i].resolutionX && height == pDepthModeList[i].resolutionY)
    {
      bFound = true;
    }
  }

  if (!bFound)
  {
    ROS_ERROR("The resolution not support!");
    return -1;
  }

  BerxelHawkStreamFrameMode frameMode;
  m_pHawkDevice->getCurrentFrameMode(type, &frameMode);

  frameMode.resolutionX = width;
  frameMode.resolutionY = height;
  if ((type == BERXEL_HAWK_DEPTH_STREAM) || (m_nStreamFlag == 2) || (m_nStreamFlag == 4))
  {
    frameMode.framerate = m_nDepthFps;
  }

  m_pHawkDevice->setFrameMode(type, &frameMode);
  return 0;
}

int32_t BerxelHawkCamera::initBerxelCamera()
{
  // 获取context
  m_pContext = BerxelHawkContext::getBerxelContext();
  m_pContext->setDeviceStateCallback(onDeviceStatusChange, this);
  // 打开设备
  BerxelHawkDeviceInfo* pDeviceInfo = NULL;
  uint32_t deviceCount = 0;
  m_pContext->getDeviceList(&pDeviceInfo, &deviceCount);
  if ((deviceCount <= 0) || (NULL == pDeviceInfo))
  {
    ROS_ERROR("Get No Connected BerxelDevice");
    return -1;
  }

  if (m_strSerialNumber.empty() && (deviceCount == 1))
  {
    m_CurrentDeviceInfo = pDeviceInfo[0];
  }
  else
  {
    for (int i = 0; i < deviceCount; i++)
    {
      if (strcmp(pDeviceInfo[i].serialNumber, m_strSerialNumber.c_str()) == 0)
      {
        ROS_INFO("Device Serial NUmber : %s, File Serial NUmber : %s", pDeviceInfo[i].serialNumber,
                 m_strSerialNumber.c_str());
        m_CurrentDeviceInfo = pDeviceInfo[i];
        break;
      }
    }
  }

  m_pHawkDevice = m_pContext->openDevice(m_CurrentDeviceInfo);
  if (NULL == m_pHawkDevice)
  {
    ROS_ERROR("Open BerxelDevice Failed");
    return -1;
  }

  if ((m_CurrentDeviceInfo.productId == 0x0004 && m_CurrentDeviceInfo.vendorId == 0x0603) ||
      (m_CurrentDeviceInfo.productId == 0x0004 && m_CurrentDeviceInfo.vendorId == 0x0c45) ||
      (m_CurrentDeviceInfo.productId == 0x000B && m_CurrentDeviceInfo.vendorId == 0x0603))
  {
    m_bSupportRGB = false;
  }

  memset((uint8_t*)&m_DeviceIntrinsicParams, 0x00, sizeof(berxel::BerxelHawkDeviceIntrinsicParams));
  m_pHawkDevice->getDeviceIntriscParams(&m_DeviceIntrinsicParams);
  memcpy((uint8_t*)&m_rgbIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.colorIntrinsicParams,
         sizeof(BerxelHawkCameraIntrinsic));
  memcpy((uint8_t*)&m_irIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.irIntrinsicParams,
         sizeof(BerxelHawkCameraIntrinsic));
  memcpy((uint8_t*)&m_depthIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.liteIrIntrinsicParams,
         sizeof(BerxelHawkCameraIntrinsic));
  memcpy((uint8_t*)&m_rotaParams, (uint8_t*)&m_DeviceIntrinsicParams.rotateIntrinsicParams,
         sizeof(BerxelHawkCameraIntrinsic));
  // 同步当前系统时钟到设备中
  m_pHawkDevice->setSystemClock();

  // 设置流模式
  m_pHawkDevice->setStreamFlagMode((BerxelHawkStreamFlagMode)m_nStreamFlag);

  if (checkConfigureParams() != 0)
  {
    ROS_ERROR("The configure params error, please check!!!");
    return -1;
  }

  if (m_bColorCloudPoint)
  {
    int ret = pthread_create(&m_nThreadID, NULL, berxelColorPointCloudThread, this);
    if (ret != 0)
    {
      ROS_ERROR("thread create failed.ret = %d", ret);
      return -1;
    }
  }

  int ret = m_pHawkDevice->startStreams(m_nStreadmType, onNewFrameCallback, this);
  if (ret != 0)
  {
    ROS_ERROR("Open Berxel Stream Failed");
    return -1;
  }

  if (m_bRegistration)
  {
    ROS_INFO("Open Berxel set Registration Enable");
    m_pHawkDevice->setRegistrationEnable(true);
  }

  return 0;
}

void BerxelHawkCamera::berxelPublishTF()
{
  ros::Time time_now = ros::Time::now();
  // m_pubBroadcaster.sendTransform(tf::StampedTransform(m_camera_link,time_now,"Cam_base_link","Cam_link"));
  if (m_bSupportRGB)
    m_pubBroadcaster.sendTransform(tf::StampedTransform(m_camera_rgb_frame, time_now, camera_link_, camera_rgb_frame_));
  m_pubBroadcaster.sendTransform(
      tf::StampedTransform(m_camera_depth_frame, time_now, camera_link_, camera_depth_frame_));
  if (m_bSupportRGB)
    m_pubBroadcaster.sendTransform(
        tf::StampedTransform(m_camera_rgb_optical_frame, time_now, camera_rgb_frame_, camera_rgb_optical_frame_));
  m_pubBroadcaster.sendTransform(
      tf::StampedTransform(m_camera_depth_optical_frame, time_now, camera_depth_frame_, camera_depth_optical_frame_));
}

void BerxelHawkCamera::getColorResolution(uint32_t& width, uint32_t& height)
{
  width = m_nColorWidth;
  height = m_nColorHeight;
}

void BerxelHawkCamera::getDepthResolution(uint32_t& width, uint32_t& height)
{
  width = m_nDepthWidth;
  height = m_nDepthHeight;
}

void BerxelHawkCamera::getIrResolution(uint32_t& width, uint32_t& height)
{
  width = m_nIrWidth;
  height = m_nIrHeight;
}

uint32_t BerxelHawkCamera::getDownsamplingRatio()
{
  return 1;
}

void BerxelHawkCamera::destroy()
{
  if (m_pHawkDevice)
  {
    while (!m_queDepthFrame.empty())
    {
      berxel::BerxelHawkFrame* pFrame = m_queDepthFrame.front();
      m_pHawkDevice->releaseFrame(pFrame);
      m_queDepthFrame.pop();
    }

    while (!m_queColorFrame.empty())
    {
      berxel::BerxelHawkFrame* pFrame = m_queColorFrame.front();
      m_pHawkDevice->releaseFrame(pFrame);
      m_queColorFrame.pop();
    }

    m_pHawkDevice->stopStreams(m_nStreadmType);
  }

  if (m_pContext)
  {
    m_pContext->closeDevice(m_pHawkDevice);
    berxel::BerxelHawkContext::destroyBerxelContext(m_pContext);
    m_pContext = NULL;
    m_pHawkDevice = NULL;
  }
}

void BerxelHawkCamera::advertiseROSTopics()
{
  ros::NodeHandle color_node(m_node, m_strDeviceName);
  image_transport::ImageTransport color_it(color_node);
  ros::NodeHandle ir_node(m_node, m_strDeviceName);
  image_transport::ImageTransport ir_it(ir_node);
  ros::NodeHandle depth_node(m_node, m_strDeviceName);
  image_transport::ImageTransport depth_it(depth_node);

  // color
  if (m_nStreadmType & 0x01)
  {
    m_pubColor = color_it.advertiseCamera("rgb/rgb_raw", 1);
  }

  // depth
  if (m_nStreadmType & 0x02)
  {
    m_pubDepth = depth_it.advertiseCamera("depth/depth_raw", 1);
    if (m_bPubPointCloud)
    {
      m_pubCloudPoint = depth_node.advertise<sensor_msgs::PointCloud2>("berxel_cloudpoint", 1);
    }
  }

  // ir
  if (m_nStreadmType & 0x04)
  {
    m_pubIr = ir_it.advertiseCamera("ir/ir_raw", 1);
  }
}

void BerxelHawkCamera::onNewFrameCallback(berxel::BerxelHawkStreamType streamType, berxel::BerxelHawkFrame* pFrame,
                                          void* pUserData)
{
  if (NULL != pUserData)
  {
    BerxelHawkCamera* pBerxelCamera = static_cast<BerxelHawkCamera*>(pUserData);
    if (NULL != pBerxelCamera)
    {
      pBerxelCamera->berxelFrameCallback(streamType, pFrame);
    }
  }
}

void BerxelHawkCamera::berxelFrameCallback(berxel::BerxelHawkStreamType streamType, berxel::BerxelHawkFrame* pFrame)
{
  pthread_mutex_lock(&m_mutex_cb);
  if (m_bColorCloudPoint && (m_nStreadmType & BERXEL_HAWK_COLOR_STREAM) && (m_nStreadmType & BERXEL_HAWK_DEPTH_STREAM))
  {
    if (streamType == berxel::BERXEL_HAWK_COLOR_STREAM)
    {
      if (m_queColorFrame.size() >= 3)
      {
        berxel::BerxelHawkFrame* pFrame = m_queColorFrame.front();
        m_pHawkDevice->releaseFrame(pFrame);
        m_queColorFrame.pop();
      }

      m_queColorFrame.push(pFrame);
    }
    else if (streamType == berxel::BERXEL_HAWK_DEPTH_STREAM)
    {
      if (m_queDepthFrame.size() >= 3)
      {
        berxel::BerxelHawkFrame* pFrame = m_queDepthFrame.front();
        m_pHawkDevice->releaseFrame(pFrame);
        m_queDepthFrame.pop();
      }

      m_queDepthFrame.push(pFrame);
    }
    else
    {
      ROS_ERROR("Color Point Cloud Need Color And Depth, Please Check!!!");
    }
  }
  else
  {
    switch (streamType)
    {
      case berxel::BERXEL_HAWK_COLOR_STREAM: {
        berxelColorFrameCallback(pFrame);
      }
      break;
      case berxel::BERXEL_HAWK_DEPTH_STREAM: {
        berxelDepthFrameCallback(pFrame);
      }
      break;
      case berxel::BERXEL_HAWK_IR_STREAM: {
        berxelIrFrameCallback(pFrame);
      }
      break;
      default:
        break;
    }

    m_pHawkDevice->releaseFrame(pFrame);
  }

  berxelPublishTF();

  pthread_mutex_unlock(&m_mutex_cb);
}

void BerxelHawkCamera::berxelColorFrameCallback(berxel::BerxelHawkFrame* pFrame)
{
  // ROS_INFO("berxelColorFrameCallback");
  if (m_pHawkDevice && pFrame != NULL)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    ros::Time ros_now = ros::Time::now();
    if (m_bRosTimeStamp)
    {
      image->header.stamp = ros_now;
    }
    else
    {
      image->header.stamp.sec = pFrame->getTimeStamp() / 1000000.0;
      image->header.stamp.nsec = pFrame->getTimeStamp() & 0xffff;
    }

    image->width = pFrame->getWidth();
    image->height = pFrame->getHeight();
    std::size_t data_size = pFrame->getDataSize();
    image->data.resize(data_size);
    memcpy(&image->data[0], pFrame->getData(), data_size);
    image->is_bigendian = 0;
    image->encoding = sensor_msgs::image_encodings::RGB8;
    image->step = sizeof(unsigned char) * 3 * image->width;
    image->header.frame_id = camera_rgb_optical_frame_;

    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->width = image->width;
    info->height = image->height;
    info->header.frame_id = camera_rgb_optical_frame_;
    info->header.stamp = image->header.stamp;
    info->distortion_model = "plumb_bob";
    info->binning_x = info->binning_y = 1;
    getRgbDistortionParams(info->D);
    getRgbInternalParameterMatrix(info->K);
    getRgbProjectionMatrix(info->P);

    m_pubColor.publish(image, info);
  }
}

void BerxelHawkCamera::berxelDepthFrameCallback(berxel::BerxelHawkFrame* pDepthFrame,
                                                berxel::BerxelHawkFrame* pColorFrame)
{
  if (m_pHawkDevice && pDepthFrame != NULL)
  {
    // ROS_INFO("berxelDepthFrameCallback");
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    ros::Time ros_now = ros::Time::now();
    if (m_bRosTimeStamp)
    {
      image->header.stamp = ros_now;
    }
    else
    {
      image->header.stamp.sec = pDepthFrame->getTimeStamp() / 1000000.0;
      image->header.stamp.nsec = pDepthFrame->getTimeStamp() & 0xffff;
    }

    image->width = pDepthFrame->getWidth();
    image->height = pDepthFrame->getHeight();
    std::size_t data_size = pDepthFrame->getDataSize();
    image->data.resize(data_size);
    if (m_bPubOriginalDepth)
    {
      memcpy(&image->data[0], pDepthFrame->getData(), data_size);
    }
    else
    {
      BerxelCommonFunc::getInstance()->convertDepthToCv16UC1((uint16_t*)pDepthFrame->getData(),
                                                             (uint16_t*)&image->data[0], pDepthFrame->getWidth(),
                                                             pDepthFrame->getHeight(), pDepthFrame->getPixelType());
    }
    image->is_bigendian = 0;
    image->encoding = sensor_msgs::image_encodings::MONO16;
    image->step = sizeof(unsigned char) * 2 * image->width;
    image->header.frame_id = camera_depth_optical_frame_;

    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->width = image->width;
    info->height = image->height;
    info->header.frame_id = camera_depth_optical_frame_;
    info->header.stamp = image->header.stamp;
    info->distortion_model = "plumb_bob";
    info->binning_x = info->binning_y = 1;
    if (m_bRegistration)
    {
      getRgbDistortionParams(info->D);
      getRgbInternalParameterMatrix(info->K);
      // getRotateMatrix(info->R);
      getRgbProjectionMatrix(info->P);
    }
    else
    {
      getDepthDistortionParams(info->D);
      getDepthInternalParameterMatrix(info->K);
      getDepthProjectionMatrix(info->P);
      getRotateMatrix(info->R);
    }

    if (m_bPubPointCloud)
    {
      uint32_t valid_count = 0;
      sensor_msgs::PointCloud2::Ptr msg_pointcloud_ptr(new sensor_msgs::PointCloud2);
      sensor_msgs::PointCloud2& msg_pointcloud = *msg_pointcloud_ptr;
      msg_pointcloud.header.stamp = image->header.stamp;
      msg_pointcloud.header.frame_id = camera_depth_optical_frame_;
      if (m_bOrderedCloudPoint)
      {
        msg_pointcloud.width = pDepthFrame->getWidth();
        msg_pointcloud.height = pDepthFrame->getHeight();
        msg_pointcloud.is_dense = false;
      }
      sensor_msgs::PointCloud2Modifier modifier(msg_pointcloud);

      if (m_bColorCloudPoint && pColorFrame != NULL)
      {
        RGB888* pColorData = (RGB888*)pColorFrame->getData();
        // uint32_t valid_count = 0;
        // uint32_t nSize = pDepthFrame->getWidth() * pDepthFrame->getHeight();
        // sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
        // modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        // modifier.resize(nSize);
        // if (m_bOrderedCloudPoint)
        // {
        // 	_msg_pointcloud.width = pDepthFrame->getWidth();
        // 	_msg_pointcloud.height = pDepthFrame->getHeight();
        // 	_msg_pointcloud.is_dense = false;
        // }
        // _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
        // _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);
        // sensor_msgs::PointCloud2Iterator<float> iter_x(_msg_pointcloud, "x");
        // sensor_msgs::PointCloud2Iterator<float> iter_y(_msg_pointcloud, "y");
        // sensor_msgs::PointCloud2Iterator<float> iter_z(_msg_pointcloud, "z");
        // sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(_msg_pointcloud, "r");
        // sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(_msg_pointcloud, "g");
        // sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(_msg_pointcloud, "b");
        // m_pHawkDevice->convertDepthToPointCloud(pDepthFrame, 1000.0, m_pPointClouds);
        // for (int nIndex = 0; nIndex < nSize; nIndex++)
        // {
        // 	bool valid_pixel(m_pPointClouds[nIndex].z > 0);
        // 	if (valid_pixel)
        // 	{
        // 		*iter_x = m_pPointClouds[nIndex].x;
        // 		*iter_y = m_pPointClouds[nIndex].y;
        // 		*iter_z = m_pPointClouds[nIndex].z;
        // 		*iter_r = pColorData[nIndex].r;
        // 		*iter_g = pColorData[nIndex].g;
        // 		*iter_b = pColorData[nIndex].b;
        // 		valid_count++;
        // 	}
        // 	++iter_x; ++iter_y; ++iter_z;
        // 	++iter_r; ++iter_g; ++iter_b;
        // }
        // _msg_pointcloud.header.stamp = image->header.stamp;
        // _msg_pointcloud.header.frame_id = camera_depth_optical_frame_;
        // if (!m_bOrderedCloudPoint)
        // {
        // 	_msg_pointcloud.width = valid_count;
        // 	_msg_pointcloud.height = 1;
        // 	_msg_pointcloud.is_dense = true;
        // 	modifier.resize(valid_count);
        // }
        // // ROS_INFO("color point");
        // m_pubCloudPoint.publish(_msg_pointcloud);

        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32, "rgb",
                                      1, sensor_msgs::PointField::FLOAT32);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg_pointcloud, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg_pointcloud, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg_pointcloud, "b");
        m_pHawkDevice->convertDepthToPointCloud(pDepthFrame, 1000.0, m_pPointClouds);
        for (int nIndex = 0; nIndex < pDepthFrame->getWidth() * pDepthFrame->getHeight(); nIndex++)
        {
          bool valid_pixel(m_pPointClouds[nIndex].z > 0);
          if (valid_pixel || m_bOrderedCloudPoint)
          {
            *iter_x = m_pPointClouds[nIndex].x;
            *iter_y = m_pPointClouds[nIndex].y;
            *iter_z = m_pPointClouds[nIndex].z;
            *iter_r = pColorData[nIndex].r;
            *iter_g = pColorData[nIndex].g;
            *iter_b = pColorData[nIndex].b;
            valid_count++;
          }

          ++iter_x;
          ++iter_y;
          ++iter_z;
          ++iter_r;
          ++iter_g;
          ++iter_b;
        }

        if (!m_bOrderedCloudPoint)
        {
          msg_pointcloud.width = valid_count;
          msg_pointcloud.height = 1;
          msg_pointcloud.is_dense = true;
        }

        m_pubCloudPoint.publish(msg_pointcloud_ptr);
      }
      else
      {
        modifier.setPointCloud2Fields(3, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1,
                                      sensor_msgs::PointField::FLOAT32, "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg_pointcloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg_pointcloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg_pointcloud, "z");
        m_pHawkDevice->convertDepthToPointCloud(pDepthFrame, 1000.0, m_pPointClouds);
        for (int nIndex = 0; nIndex < pDepthFrame->getWidth() * pDepthFrame->getHeight(); nIndex++)
        {
          bool valid_pixel(m_pPointClouds[nIndex].z > 0);
          if (valid_pixel || m_bOrderedCloudPoint)
          {
            *iter_x = m_pPointClouds[nIndex].x;
            *iter_y = m_pPointClouds[nIndex].y;
            *iter_z = m_pPointClouds[nIndex].z;
            valid_count++;
          }

          ++iter_x;
          ++iter_y;
          ++iter_z;
        }

        if (!m_bOrderedCloudPoint)
        {
          msg_pointcloud.width = valid_count;
          msg_pointcloud.height = 1;
          msg_pointcloud.is_dense = true;
        }

        m_pubCloudPoint.publish(msg_pointcloud_ptr);

        /*				uint32_t nSize = pDepthFrame->getWidth() * pDepthFrame->getHeight();
                sensor_msgs::PointCloud2Modifier modifier(_msg_pointcloud);
                modifier.setPointCloud2FieldsByString(1, "xyz");
                modifier.resize(nSize);

                if (m_bOrderedCloudPoint)
                {
                  _msg_pointcloud.width = pDepthFrame->getWidth();
                  _msg_pointcloud.height = pDepthFrame->getHeight();
                  _msg_pointcloud.is_dense = false;
                }

                _msg_pointcloud.row_step = _msg_pointcloud.width * _msg_pointcloud.point_step;
                    _msg_pointcloud.data.resize(_msg_pointcloud.height * _msg_pointcloud.row_step);
                sensor_msgs::PointCloud2Iterator<float> iter_x(_msg_pointcloud, "x");
                sensor_msgs::PointCloud2Iterator<float> iter_y(_msg_pointcloud, "y");
                sensor_msgs::PointCloud2Iterator<float> iter_z(_msg_pointcloud, "z");
                m_pHawkDevice->convertDepthToPointCloud(pDepthFrame, 1000.0, m_pPointClouds);
                for (int nIndex = 0; nIndex < nSize; nIndex++)
                {
                  bool valid_pixel(m_pPointClouds[nIndex].z > 0);
                  if (valid_pixel || m_bOrderedCloudPoint)
                  {
                    *iter_x = m_pPointClouds[nIndex].x;
                    *iter_y = m_pPointClouds[nIndex].y;
                    *iter_z = m_pPointClouds[nIndex].z;

                    ++iter_x; ++iter_y; ++iter_z;
                    ++valid_count;
                  }
                }

                _msg_pointcloud.header.stamp = image->header.stamp;
                _msg_pointcloud.header.frame_id = camera_depth_optical_frame_;

                if (!m_bOrderedCloudPoint)
                {
                  _msg_pointcloud.width = valid_count;
                  _msg_pointcloud.height = 1;
                  _msg_pointcloud.is_dense = true;
                  modifier.resize(valid_count);
                }

                m_pubCloudPoint.publish(_msg_pointcloud);
                */
      }
    }
    m_pubDepth.publish(image, info);
  }
}

void BerxelHawkCamera::berxelIrFrameCallback(berxel::BerxelHawkFrame* pFrame)
{
  if (m_pHawkDevice && pFrame != NULL)
  {
    sensor_msgs::ImagePtr image(new sensor_msgs::Image);
    ros::Time ros_now = ros::Time::now();
    if (m_bRosTimeStamp)
    {
      image->header.stamp = ros_now;
    }
    else
    {
      image->header.stamp.sec = pFrame->getTimeStamp() / 1000000.0;
      image->header.stamp.nsec = pFrame->getTimeStamp() & 0xffff;
    }

    image->width = pFrame->getWidth();
    image->height = pFrame->getHeight();
    std::size_t data_size = pFrame->getDataSize();
    image->data.resize(data_size);
    memcpy(&image->data[0], pFrame->getData(), data_size);
    image->is_bigendian = 0;
    image->encoding = sensor_msgs::image_encodings::MONO16;
    image->step = sizeof(unsigned char) * 2 * image->width;
    image->header.frame_id = "ir_optical_frame";

    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->width = image->width;
    info->height = image->height;
    info->header.frame_id = "ir_optical_frame";
    info->header.stamp = image->header.stamp;
    info->distortion_model = "plumb_bob";
    info->binning_x = info->binning_y = 1;
    getIrDistortionParams(info->D);
    getIrInternalParameterMatrix(info->K);
    getRotateMatrix(info->R);
    getIrProjectionMatrix(info->P);
    m_pubIr.publish(image, info);
  }
}

void BerxelHawkCamera::readDeviceIntriscParams(berxel::BerxelHawkDeviceIntrinsicParams* pParams)
{
  if (m_pHawkDevice != NULL)
  {
    m_pHawkDevice->getDeviceIntriscParams(pParams);
  }
}

void BerxelHawkCamera::onDeviceStatusChange(const char* deviceUri, const char* deviceSerialNumber,
                                            berxel::BerxelHawkDeviceStatus deviceState, void* pUserData)
{
  if (NULL != pUserData)
  {
    BerxelHawkCamera* pBerxelCamera = static_cast<BerxelHawkCamera*>(pUserData);
    if (NULL != pBerxelCamera)
    {
      pBerxelCamera->processDeviceStatusChange(deviceUri, deviceSerialNumber, deviceState);
    }
  }
}

int32_t BerxelHawkCamera::processDeviceStatusChange(const char* deviceUri, const char* deviceSerialNumber,
                                                    berxel::BerxelHawkDeviceStatus deviceState)
{
  switch (deviceState)
  {
    case berxel::BERXEL_HAWK_DEVICE_DISCONNECT: {
      if (strcmp(m_CurrentDeviceInfo.serialNumber, deviceSerialNumber) == 0)
      {
        ROS_INFO("Device Disconnect!");
        if (m_pHawkDevice != NULL)
        {
          pthread_mutex_lock(&m_mutex_cb);
          while (!m_queDepthFrame.empty())
          {
            berxel::BerxelHawkFrame* pFrame = m_queDepthFrame.front();
            m_pHawkDevice->releaseFrame(pFrame);
            m_queDepthFrame.pop();
          }

          while (!m_queColorFrame.empty())
          {
            berxel::BerxelHawkFrame* pFrame = m_queColorFrame.front();
            m_pHawkDevice->releaseFrame(pFrame);
            m_queColorFrame.pop();
          }
          pthread_mutex_unlock(&m_mutex_cb);
          m_pHawkDevice->stopStreams(m_nStreadmType);

          m_pContext->closeDevice(m_pHawkDevice);
          m_pHawkDevice = NULL;
        }
      }
    }
    break;
    case berxel::BERXEL_HAWK_DEVICE_CONNECT: {
      if (strcmp(m_CurrentDeviceInfo.serialNumber, deviceSerialNumber) == 0)
      {
        ROS_INFO("Device Connected!");
        BerxelHawkDeviceInfo* pDeviceInfo = NULL;
        uint32_t deviceCount = 0;
        m_pContext->getDeviceList(&pDeviceInfo, &deviceCount);
        if ((deviceCount <= 0) || (NULL == pDeviceInfo))
        {
          ROS_ERROR("Get No Connected BerxelDevice");
          return -1;
        }

        bool bFound = false;
        for (int i = 0; i < deviceCount; i++)
        {
          if (strcmp(pDeviceInfo[i].serialNumber, deviceSerialNumber) == 0)
          {
            bFound = true;
            m_CurrentDeviceInfo = pDeviceInfo[i];
            break;
          }
        }

        if (bFound == false || m_pHawkDevice != NULL)
        {
          ROS_ERROR("Device(%s) no exist", deviceUri);
          return -1;
        }

        ROS_INFO("m_CurrentDeviceInfo -- > SN : %s", m_CurrentDeviceInfo.serialNumber);
        ROS_INFO("m_CurrentDeviceInfo -- > SN : %s", m_CurrentDeviceInfo.serialNumber);
        ROS_INFO("m_CurrentDeviceInfo -- > SN : %s", m_CurrentDeviceInfo.serialNumber);
        m_pHawkDevice = m_pContext->openDevice(m_CurrentDeviceInfo);
        if (NULL == m_pHawkDevice)
        {
          ROS_ERROR("Open BerxelDevice Failed");
          return -1;
        }

        memset((uint8_t*)&m_DeviceIntrinsicParams, 0x00, sizeof(berxel::BerxelHawkDeviceIntrinsicParams));
        m_pHawkDevice->getDeviceIntriscParams(&m_DeviceIntrinsicParams);
        memcpy((uint8_t*)&m_rgbIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.colorIntrinsicParams,
               sizeof(BerxelHawkCameraIntrinsic));
        memcpy((uint8_t*)&m_irIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.irIntrinsicParams,
               sizeof(BerxelHawkCameraIntrinsic));
        memcpy((uint8_t*)&m_depthIntrinsicParams, (uint8_t*)&m_DeviceIntrinsicParams.liteIrIntrinsicParams,
               sizeof(BerxelHawkCameraIntrinsic));
        memcpy((uint8_t*)&m_rotaParams, (uint8_t*)&m_DeviceIntrinsicParams.rotateIntrinsicParams,
               sizeof(BerxelHawkCameraIntrinsic));
        // 同步当前系统时钟到设备中
        //  m_pHawkDevice->setSystemClock();

        // 设置流模式
        m_pHawkDevice->setStreamFlagMode((BerxelHawkStreamFlagMode)m_nStreamFlag);
        int ret = m_pHawkDevice->startStreams(m_nStreadmType, onNewFrameCallback, this);
        if (ret != 0)
        {
          ROS_ERROR("Open Berxel Stream Failed");
          return -1;
        }

        if (m_bRegistration)
        {
          ROS_INFO("Open Berxel set Registration Enable");
          m_pHawkDevice->setRegistrationEnable(true);
        }
      }
    }
    break;
    default:
      break;
  }

  return 0;
}

void BerxelHawkCamera::getRgbDistortionParams(std::vector<double>& d)
{
  if (m_pHawkDevice != NULL)
  {
    std::vector<double> d_rgb = { m_rgbIntrinsicParams.k1Param, m_rgbIntrinsicParams.k2Param,
                                  m_rgbIntrinsicParams.p1Param, m_rgbIntrinsicParams.p2Param,
                                  m_rgbIntrinsicParams.k3Param };
    d.swap(d_rgb);
  }
}

void BerxelHawkCamera::getIrDistortionParams(std::vector<double>& d)
{
  if (m_pHawkDevice != NULL)
  {
    std::vector<double> d_ir = { m_irIntrinsicParams.k1Param, m_irIntrinsicParams.k2Param, m_irIntrinsicParams.p1Param,
                                 m_irIntrinsicParams.p2Param, m_irIntrinsicParams.k3Param };
    d.swap(d_ir);
  }
}

void BerxelHawkCamera::getDepthDistortionParams(std::vector<double>& d)
{
  if (m_pHawkDevice != NULL)
  {
    std::vector<double> d_depth = { m_depthIntrinsicParams.k1Param, m_depthIntrinsicParams.k2Param,
                                    m_depthIntrinsicParams.p1Param, m_depthIntrinsicParams.p2Param,
                                    m_depthIntrinsicParams.k3Param };
    d.swap(d_depth);
  }
}

void BerxelHawkCamera::getRgbInternalParameterMatrix(boost::array<double, 9>& k)
{
  if (m_nColorWidth > 640 || m_nColorHeight > 640)
  {
    boost::array<double, 9> k_rgb = { m_rgbIntrinsicParams.fxParam,
                                      0.0,
                                      m_rgbIntrinsicParams.cxParam,
                                      0.0,
                                      m_rgbIntrinsicParams.fyParam,
                                      m_rgbIntrinsicParams.cyParam,
                                      0.0,
                                      0.0,
                                      1.0 };
    k.swap(k_rgb);
  }
  else if (m_nColorWidth > 320 || m_nColorHeight > 320)
  {
    boost::array<double, 9> k_rgb = { m_rgbIntrinsicParams.fxParam / 2.0,
                                      0.0,
                                      m_rgbIntrinsicParams.cxParam / 2.0,
                                      0.0,
                                      m_rgbIntrinsicParams.fyParam / 2.0,
                                      m_rgbIntrinsicParams.cyParam / 2.0,
                                      0.0,
                                      0.0,
                                      1.0 };
    k.swap(k_rgb);
  }
  else
  {
    boost::array<double, 9> k_rgb = { m_rgbIntrinsicParams.fxParam / 4.0,
                                      0.0,
                                      m_rgbIntrinsicParams.cxParam / 4.0,
                                      0.0,
                                      m_rgbIntrinsicParams.fyParam / 4.0,
                                      m_rgbIntrinsicParams.cyParam / 4.0,
                                      0.0,
                                      0.0,
                                      1.0 };
    k.swap(k_rgb);
  }
}

void BerxelHawkCamera::getIrInternalParameterMatrix(boost::array<double, 9>& k)
{
  if (m_nIrHeight > 640 || m_nIrWidth > 640)
  {
    boost::array<double, 9> k_ir = { m_irIntrinsicParams.fxParam,
                                     0.0,
                                     m_irIntrinsicParams.cxParam,
                                     0.0,
                                     m_irIntrinsicParams.fyParam,
                                     m_irIntrinsicParams.cyParam,
                                     0.0,
                                     0.0,
                                     1.0 };
    k.swap(k_ir);
  }
  else if (m_nIrHeight > 320 || m_nIrWidth > 320)
  {
    boost::array<double, 9> k_ir = { m_irIntrinsicParams.fxParam / 2.0,
                                     0.0,
                                     m_irIntrinsicParams.cxParam / 2.0,
                                     0.0,
                                     m_irIntrinsicParams.fyParam / 2.0,
                                     m_irIntrinsicParams.cyParam / 2.0,
                                     0.0,
                                     0.0,
                                     1.0 };
    k.swap(k_ir);
  }
  else
  {
    boost::array<double, 9> k_ir = { m_irIntrinsicParams.fxParam / 4.0,
                                     0.0,
                                     m_irIntrinsicParams.cxParam / 4.0,
                                     0.0,
                                     m_irIntrinsicParams.fyParam / 4.0,
                                     m_irIntrinsicParams.cyParam / 4.0,
                                     0.0,
                                     0.0,
                                     1.0 };
    k.swap(k_ir);
  }
}

void BerxelHawkCamera::getDepthInternalParameterMatrix(boost::array<double, 9>& k)
{
  if (m_nDepthHeight > 640 || m_nDepthWidth > 640)
  {
    boost::array<double, 9> k_depth = { m_depthIntrinsicParams.fxParam,
                                        0.0,
                                        m_depthIntrinsicParams.cxParam,
                                        0.0,
                                        m_depthIntrinsicParams.fyParam,
                                        m_depthIntrinsicParams.cyParam,
                                        0.0,
                                        0.0,
                                        1.0 };
    k.swap(k_depth);
  }
  else if (m_nDepthHeight > 320 || m_nDepthWidth > 320)
  {
    boost::array<double, 9> k_depth = { m_depthIntrinsicParams.fxParam / 2.0,
                                        0.0,
                                        m_depthIntrinsicParams.cxParam / 2.0,
                                        0.0,
                                        m_depthIntrinsicParams.fyParam / 2.0,
                                        m_depthIntrinsicParams.cyParam / 2.0,
                                        0.0,
                                        0.0,
                                        1.0 };
    k.swap(k_depth);
  }
  else
  {
    boost::array<double, 9> k_depth = { m_depthIntrinsicParams.fxParam / 4.0,
                                        0.0,
                                        m_depthIntrinsicParams.cxParam / 4.0,
                                        0.0,
                                        m_depthIntrinsicParams.fyParam / 4.0,
                                        m_depthIntrinsicParams.cyParam / 4.0,
                                        0.0,
                                        0.0,
                                        1.0 };
    k.swap(k_depth);
  }
}

void BerxelHawkCamera::getRotateMatrix(boost::array<double, 9>& r)
{
  boost::array<double, 9> r_rotate = { m_rotaParams.fxParam, m_rotaParams.fyParam, m_rotaParams.cxParam,
                                       m_rotaParams.cyParam, m_rotaParams.k1Param, m_rotaParams.k2Param,
                                       m_rotaParams.p1Param, m_rotaParams.p2Param, m_rotaParams.k3Param };
  r.swap(r_rotate);
}

void BerxelHawkCamera::getRgbProjectionMatrix(boost::array<double, 12>& p)
{
  if (m_nColorWidth > 640 || m_nColorHeight > 640)
  {
    boost::array<double, 12> p_rgb = { m_rgbIntrinsicParams.fxParam,
                                       0.0,
                                       m_rgbIntrinsicParams.cxParam,
                                       0.0,
                                       0.0,
                                       m_rgbIntrinsicParams.fyParam,
                                       m_rgbIntrinsicParams.cyParam,
                                       0.0,
                                       0.0,
                                       0.0,
                                       1.0,
                                       0.0 };
    p.swap(p_rgb);
  }
  else if (m_nColorWidth > 320 || m_nColorHeight > 320)
  {
    boost::array<double, 12> p_rgb = { m_rgbIntrinsicParams.fxParam / 2.0,
                                       0.0,
                                       m_rgbIntrinsicParams.cxParam / 2.0,
                                       0.0,
                                       0.0,
                                       m_rgbIntrinsicParams.fyParam / 2.0,
                                       m_rgbIntrinsicParams.cyParam / 2.0,
                                       0.0,
                                       0.0,
                                       0.0,
                                       1.0,
                                       0.0 };
    p.swap(p_rgb);
  }
  else
  {
    boost::array<double, 12> p_rgb = { m_rgbIntrinsicParams.fxParam / 4.0,
                                       0.0,
                                       m_rgbIntrinsicParams.cxParam / 4.0,
                                       0.0,
                                       0.0,
                                       m_rgbIntrinsicParams.fyParam / 4.0,
                                       m_rgbIntrinsicParams.cyParam / 4.0,
                                       0.0,
                                       0.0,
                                       0.0,
                                       1.0,
                                       0.0 };
    p.swap(p_rgb);
  }
}

void BerxelHawkCamera::getIrProjectionMatrix(boost::array<double, 12>& p)
{
  if (m_nIrWidth > 640 || m_nIrHeight > 640)
  {
    boost::array<double, 12> p_ir = { m_irIntrinsicParams.fxParam,
                                      0.0,
                                      m_irIntrinsicParams.cxParam,
                                      0.0,
                                      0.0,
                                      m_irIntrinsicParams.fyParam,
                                      m_irIntrinsicParams.cyParam,
                                      0.0,
                                      0.0,
                                      0.0,
                                      1.0,
                                      0.0 };
    p.swap(p_ir);
  }
  else if (m_nIrWidth > 320 || m_nIrHeight > 320)
  {
    boost::array<double, 12> p_ir = { m_irIntrinsicParams.fxParam / 2.0,
                                      0.0,
                                      m_irIntrinsicParams.cxParam / 2.0,
                                      0.0,
                                      0.0,
                                      m_irIntrinsicParams.fyParam / 2.0,
                                      m_irIntrinsicParams.cyParam / 2.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      1.0,
                                      0.0 };
    p.swap(p_ir);
  }
  else
  {
    boost::array<double, 12> p_ir = { m_irIntrinsicParams.fxParam / 4.0,
                                      0.0,
                                      m_irIntrinsicParams.cxParam / 4.0,
                                      0.0,
                                      0.0,
                                      m_irIntrinsicParams.fyParam / 4.0,
                                      m_irIntrinsicParams.cyParam / 4.0,
                                      0.0,
                                      0.0,
                                      0.0,
                                      1.0,
                                      0.0 };
    p.swap(p_ir);
  }
}

void BerxelHawkCamera::getDepthProjectionMatrix(boost::array<double, 12>& p)
{
  if (m_nDepthWidth > 640 || m_nDepthHeight > 640)
  {
    boost::array<double, 12> p_depth = { m_depthIntrinsicParams.fxParam,
                                         0.0,
                                         m_depthIntrinsicParams.cxParam,
                                         0.0,
                                         0.0,
                                         m_depthIntrinsicParams.fyParam,
                                         m_depthIntrinsicParams.cyParam,
                                         0.0,
                                         0.0,
                                         0.0,
                                         1.0,
                                         0.0 };
    p.swap(p_depth);
  }
  else if (m_nDepthWidth > 320 || m_nDepthHeight > 320)
  {
    boost::array<double, 12> p_depth = { m_depthIntrinsicParams.fxParam / 2.0,
                                         0.0,
                                         m_depthIntrinsicParams.cxParam / 2.0,
                                         0.0,
                                         0.0,
                                         m_depthIntrinsicParams.fyParam / 2.0,
                                         m_depthIntrinsicParams.cyParam / 2.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         1.0,
                                         0.0 };
    p.swap(p_depth);
  }
  else
  {
    boost::array<double, 12> p_depth = { m_depthIntrinsicParams.fxParam / 4.0,
                                         0.0,
                                         m_depthIntrinsicParams.cxParam / 4.0,
                                         0.0,
                                         0.0,
                                         m_depthIntrinsicParams.fyParam / 4.0,
                                         m_depthIntrinsicParams.cyParam / 4.0,
                                         0.0,
                                         0.0,
                                         0.0,
                                         1.0,
                                         0.0 };
    p.swap(p_depth);
  }
}

void* BerxelHawkCamera::berxelColorPointCloudThread(void* obj)
{
  if (obj != NULL)
  {
    BerxelHawkCamera* pBerxelHawkCamera = static_cast<BerxelHawkCamera*>(obj);
    if (pBerxelHawkCamera != NULL)
    {
      pBerxelHawkCamera->berxelColorPointCloudHandle();
    }
  }
}

void BerxelHawkCamera::berxelColorPointCloudHandle()
{
  while (ros::ok())
  {
    pthread_mutex_lock(&m_mutex_cb);

    if (m_queDepthFrame.size() > 0 && m_queColorFrame.size() > 0)
    {
      // ROS_INFO("berxelColorPointCloudHandle");
      berxel::BerxelHawkFrame* pDepthFrame = m_queDepthFrame.front();
      berxel::BerxelHawkFrame* pColorFrame = m_queColorFrame.front();

      berxelDepthFrameCallback(pDepthFrame, pColorFrame);
      berxelColorFrameCallback(pColorFrame);

      m_queDepthFrame.pop();
      m_pHawkDevice->releaseFrame(pDepthFrame);

      m_queColorFrame.pop();
      m_pHawkDevice->releaseFrame(pColorFrame);
    }

    pthread_mutex_unlock(&m_mutex_cb);
  }
}
