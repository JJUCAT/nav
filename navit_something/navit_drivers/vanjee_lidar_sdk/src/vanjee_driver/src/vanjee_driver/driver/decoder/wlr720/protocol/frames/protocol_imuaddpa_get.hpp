#pragma once

#include "protocol_abstract_720.hpp"
#include "vanjee_driver/driver/decoder/wlr720/protocol/params/params_imu_add.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ImuAddGet720:public ProtocolAbstract720
    {
      public:
        Protocol_ImuAddGet720():ProtocolAbstract720(CmdRepository720::CreateInstance()->Sp_ImuAddParamGet,std::make_shared<Params_IMUAdd720>())
        {
          
        }
    };
  }
}