#pragma once

#include "vanjee_driver/driver/difop/params_abstract.hpp"
#include "vanjee_driver/common/super_header.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Params_IMUAdd720 : public ParamsAbstract
    {
      public:
        float X_K;
        float X_B;

        float Y_K;
        float Y_B;

        float Z_K;
        float Z_B;
      public:
        virtual std::shared_ptr<std::vector<uint8>> GetBytes()
        {
          std::shared_ptr<std::vector<uint8>> buf = std::make_shared<std::vector<uint8>>();
          return nullptr;
        }

        virtual void Load(ProtocolBase& protocol)
        {
          auto buf = protocol.Content.data();
        
          float* data = reinterpret_cast<float*>(buf);
          X_B = *data;
          data = reinterpret_cast<float*>(buf+4);
          X_K = *data;

          data = reinterpret_cast<float*>(buf+8);
          Y_B = *data;
          data = reinterpret_cast<float*>(buf+12);
          Y_K = *data;

          data = reinterpret_cast<float*>(buf+16);
          Z_B = *data;
          data = reinterpret_cast<float*>(buf+20);
          Z_K = *data;
          
        }
    };
  }
}