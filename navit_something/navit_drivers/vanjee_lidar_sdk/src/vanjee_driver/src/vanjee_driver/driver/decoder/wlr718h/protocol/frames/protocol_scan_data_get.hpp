#pragma once

#include "protocol_abstract_718h.hpp"
#include "vanjee_driver/driver/decoder/wlr718h/protocol/params/params_scan_data.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ScanDataGet718H:public ProtocolAbstract718H
    {
      public:
        Protocol_ScanDataGet718H():ProtocolAbstract718H(CmdRepository718H::CreateInstance()->Sp_ScanDataGet,std::make_shared<Params_ScanData718H>())
        {
          
        }
    };
  }
}