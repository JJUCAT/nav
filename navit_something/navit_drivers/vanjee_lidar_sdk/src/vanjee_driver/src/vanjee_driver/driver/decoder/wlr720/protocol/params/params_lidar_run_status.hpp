#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_LiDARRunStatus : public ParamsAbstract
    {
      public:
        std::array<uint8,19> reserve1;
        uint16 imu_temp;
        std::array<uint8,26> reserve2;
      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
          imu_temp = ((*(buf+19) & 0xFF) << 8) + (*(buf+20) & 0xFF);
        }
    };
  }
}