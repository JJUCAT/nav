/*
 * @Author: Guo Xuemei 1242143575@qq.com
 * @Date: 2023-01-28 14:49:41
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-21 09:54:10
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/msg/pcl_point_cloud_msg.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <pcl/io/io.h>
#include <pcl/point_types.h>

typedef pcl::PointXYZI PointXYZI;
typedef pcl::PointXYZHSV PointXYZHSV;
struct PointXYZIRT
{
  PCL_ADD_POINT4D;
  float intensity;
  uint16_t ring = 0;
  double timestamp = 0;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))
template <typename T_Point>
class PointCloudT : public pcl::PointCloud<T_Point>
{
public:
  typedef T_Point PointT;
  typedef typename pcl::PointCloud<T_Point>::VectorType VectorT;
  double timestamp = 0.0; 
  uint32_t seq = 0;       
};