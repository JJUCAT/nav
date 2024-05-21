#pragma once

#include "protocol_abstract_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/params/params_heart_bit_tcp.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_HeartBit719CTcp:public ProtocolAbstract719C
    {
      public:
        Protocol_HeartBit719CTcp():ProtocolAbstract719C(CmdRepository719C::CreateInstance()->Sp_HeartBit_Tcp,std::make_shared<Params_HeartBit719CTcp>())
        {
          
        }
    };
  }
}