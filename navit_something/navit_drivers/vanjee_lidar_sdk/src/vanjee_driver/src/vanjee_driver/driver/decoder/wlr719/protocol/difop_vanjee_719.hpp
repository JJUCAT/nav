#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"

#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/cmd_repository_719.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_heart_bit.hpp>
#include <vanjee_driver/driver/decoder/wlr719/protocol/frames/protocol_scan_data_get.hpp>

namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee719 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee719::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet719>()->GetRequest()), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719::CreateInstance()->Sp_ScanDataGet->GetCmdKey(),getDifoCtrlData_ScanDataGet);

      GetDifoCtrlClass getDifoCtrlData_HeartBit(*(std::make_shared<Protocol_HeartBit719>()->GetRequest()), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository719::CreateInstance()->Sp_HeartBit->GetCmdKey(),getDifoCtrlData_HeartBit);
    }
  }
}