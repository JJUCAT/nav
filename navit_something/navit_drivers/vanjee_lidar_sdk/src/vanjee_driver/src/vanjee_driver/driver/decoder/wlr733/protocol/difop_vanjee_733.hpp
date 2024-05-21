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
    class DifopVanjee733 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee733::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet733>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository733::CreateInstance()->Sp_LDAngleGet->GetCmdKey(),getDifoCtrlData_LdAngleGet);
    }
  }
}