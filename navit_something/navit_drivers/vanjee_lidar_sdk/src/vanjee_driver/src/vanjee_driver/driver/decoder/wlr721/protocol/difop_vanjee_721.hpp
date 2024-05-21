#pragma once
#include <memory>
#include <vector>

#include "vanjee_driver/common/super_header.hpp"
#include "vanjee_driver/driver/difop/protocol_abstract.hpp"
#include "vanjee_driver/driver/difop/difop_base.hpp"


namespace vanjee
{
  namespace lidar
  {
    class DifopVanjee721 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee721::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet721>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository721::CreateInstance()->Sp_LDAngleGet->GetCmdKey(),getDifoCtrlData_LdAngleGet);
    }
  }
}