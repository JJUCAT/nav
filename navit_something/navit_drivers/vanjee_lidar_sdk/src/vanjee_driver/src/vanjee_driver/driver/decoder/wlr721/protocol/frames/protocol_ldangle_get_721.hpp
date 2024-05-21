#pragma once

#include "protocol_abstract_721.hpp"
#include "vanjee_driver/driver/decoder/wlr721/protocol/params/params_ldangle_721.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_LDAngleGet721:public ProtocolAbstract721
    {
      public:
        Protocol_LDAngleGet721():ProtocolAbstract721(CmdRepository721::CreateInstance()->Sp_LDAngleGet,std::make_shared<Params_LDAngle721>())
        {
          
        }
    };
  }
}