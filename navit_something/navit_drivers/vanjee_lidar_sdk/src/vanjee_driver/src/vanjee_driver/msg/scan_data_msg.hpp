#pragma once
#include "vanjee_driver/common/super_header.hpp"
#include <array>
#include <vector>
#include <string>

namespace vanjee
{
  namespace lidar
  {
    class ScanData
    {

    public:
        uint32 seq = 0;                 // 序号
        float64 timestamp = 0.0;        // 时间戳
        float32 angle_min;              // 开始扫描的角度(rad)
        float32 angle_max;              // 结束扫描的角度(rad))
        float32 angle_increment;        // 每一次扫描增加的角度(rad)

        float32 time_increment;         // 测量的时间间隔(s)

        float32 scan_time;              // 扫描的时间间隔(s)

        float32 range_min;              // 距离最小值(m)
        float32 range_max;              // 距离最大值(m)
        
        std::vector<float32> ranges;        // 距离数组(长度360)(注意: 值 < range_min 或 > range_max 应当被丢弃)
        std::vector<float32> intensities;   // 与设备有关,强度数组(长度360)
    };
  }
}
