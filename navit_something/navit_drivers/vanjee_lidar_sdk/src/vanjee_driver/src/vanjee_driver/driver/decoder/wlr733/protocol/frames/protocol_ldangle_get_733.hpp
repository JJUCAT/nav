#pragma once

#include "protocol_abstract_733.hpp"
#include "vanjee_driver/driver/decoder/wlr733/protocol/params/params_ldangle_733.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_LDAngleGet733:public ProtocolAbstract733
    {
      public:
        Protocol_LDAngleGet733():ProtocolAbstract733(CmdRepository733::CreateInstance()->Sp_LDAngleGet,std::make_shared<Params_LDAngle733>())
        {
          
        }
    };
  }
}