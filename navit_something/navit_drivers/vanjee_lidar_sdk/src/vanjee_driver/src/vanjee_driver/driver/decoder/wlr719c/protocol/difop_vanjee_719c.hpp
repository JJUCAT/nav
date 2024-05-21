#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"

#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/cmd_repository_719c.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_scan_data_get.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heart_bit_tcp.hpp>
#include <vanjee_driver/driver/decoder/wlr719c/protocol/frames/protocol_heart_bit_udp.hpp>

namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee719C : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee719C::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      // GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet719C>()->GetRequest()),true);
      // (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719C::CreateInstance()->Sp_ScanDataGet->GetCmdKey(),getDifoCtrlData_ScanDataGet);

      GetDifoCtrlClass getDifoCtrlData_HeartBit_Tcp(*(std::make_shared<Protocol_HeartBit719CTcp>()->GetRequest()), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719C::CreateInstance()->Sp_HeartBit_Tcp->GetCmdKey(),getDifoCtrlData_HeartBit_Tcp);

      GetDifoCtrlClass getDifoCtrlData_HeartBit_Udp(*(std::make_shared<Protocol_HeartBit719CUdp>()->GetRequest()), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719C::CreateInstance()->Sp_HeartBit_Udp->GetCmdKey(),getDifoCtrlData_HeartBit_Udp);
    }
  }
}