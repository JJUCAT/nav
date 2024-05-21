/*
 * @Author: guo
 * @Date: 2023-02-01 19:05:51
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-21 10:29:40
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/msg/vanjee_msg/lidar_point_cloud_msg.hpp
 */
#pragma once

#include "vanjee_driver/msg/point_cloud_msg.hpp"
#ifdef POINT_TYPE_XYZIRT
typedef PointCloudT<PointXYZIRT> LidarPointCloudMsg;
#elif POINT_TYPE_XYZHSV
typedef PointCloudT<PointXYZHSV> LidarPointCloudMsg;
#else
typedef PointCloudT<PointXYZI> LidarPointCloudMsg;
#endif