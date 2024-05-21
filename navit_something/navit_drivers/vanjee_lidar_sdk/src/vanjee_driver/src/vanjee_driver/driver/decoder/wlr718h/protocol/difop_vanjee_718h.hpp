#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"

#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/cmd_repository_718h.hpp>
#include <vanjee_driver/driver/decoder/wlr718h/protocol/frames/protocol_scan_data_get.hpp>

namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee718H : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee718H::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      const uint8 arr[] = {0x01,0x00,0x00,0x00};
      std::shared_ptr<std::vector<uint8>> content = std::make_shared<std::vector<uint8>>();
      content->insert(content->end(),arr,arr+sizeof(arr)/sizeof(uint8));

      GetDifoCtrlClass getDifoCtrlData_ScanDataGet(*(std::make_shared<Protocol_ScanDataGet718H>()->GetRequest(content)), false, 3000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository718H::CreateInstance()->Sp_ScanDataGet->GetCmdKey(),getDifoCtrlData_ScanDataGet);

    }
  }
}