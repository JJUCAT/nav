#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_LDAngle721 : public ParamsAbstract
    {
      public:
        uint8 NumOfLines;
        /// @brief 1000 times the real vertical angle
        std::array<int32,64> VerAngle;
        /// @brief 1000 times the real horizontal angle
        std::array<int32,64> HorAngle;
      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
          NumOfLines = *buf;
          int32* data = reinterpret_cast<int32*>(buf+1);
          std::copy(data,data+VerAngle.size(),std::begin(VerAngle));
          std::copy(data+VerAngle.size(),data+VerAngle.size()+HorAngle.size(),std::begin(HorAngle));
        }
    };
  }
}