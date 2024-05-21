#pragma once
#include "vanjee_driver/common/super_header.hpp"
#include <array>

namespace vanjee
{
  namespace lidar
  {
    class ImuPacket
    {
      public:
        float64 timestamp = 0.0; 
        uint32 seq = 0;
        std::array<float64,4> orientation;
        std::array<float64,9> orientation_covariance;

        std::array<float64,3> angular_voc;
        std::array<float64,9> angular_voc_covariance;

        std::array<float64,3> linear_acce;
        std::array<float64,9> linear_acce_covariance;
    };
  }
}