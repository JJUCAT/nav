/*
 * @Author: guo
 * @Date: 2023-01-28 14:36:23
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-03-21 09:55:38
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/msg/pcl_point_cloud_msg.hpp
 */
#pragma once

#include <vector>
#include <string>
struct PointXYZI
{
    float x;
    float y;
    float z;
    float intensity;
};
struct PointXYZHSV
{
    float x;
    float y;
    float z;
    float h;
    float s;
    float v;
};
struct PointXYZIRT
{
    float x;
    float y;
    float z;
    float intensity;
    uint16_t ring;
    double timestamp;
};
template <typename T_Point>
class PointCloudT
{

public:
    typedef T_Point PointT;
    typedef std::vector<PointT> VectorT;
    uint32_t height = 0;   
    uint32_t width = 0;    
    bool is_dense = false; 
    double timestamp;      
    uint32_t seq = 0;      
    VectorT points;
};
