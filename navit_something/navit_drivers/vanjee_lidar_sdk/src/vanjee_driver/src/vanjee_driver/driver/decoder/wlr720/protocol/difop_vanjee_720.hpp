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
    class DifopVanjee720 : public DifopBase
    {
      public:
        virtual void initGetDifoCtrlDataMapPtr();
    };

    void DifopVanjee720::initGetDifoCtrlDataMapPtr()
    {
      getDifoCtrlData_map_ptr_ = std::make_shared<std::map<uint16,GetDifoCtrlClass>>(); 

      GetDifoCtrlClass getDifoCtrlData_LdAngleGet(*(std::make_shared<Protocol_LDAngleGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->Sp_LDAngleGet->GetCmdKey(),getDifoCtrlData_LdAngleGet);

      GetDifoCtrlClass getDifoCtrlData_ImuLineGet(*(std::make_shared<Protocol_ImuLineGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->Sp_ImuLineParamGet->GetCmdKey(),getDifoCtrlData_ImuLineGet);

      GetDifoCtrlClass getDifoCtrlData_IMUAddGet(*(std::make_shared<Protocol_ImuAddGet720>()->GetRequest()));
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->Sp_ImuAddParamGet->GetCmdKey(),getDifoCtrlData_IMUAddGet);

       GetDifoCtrlClass getDifoCtrlData_ImuTempGet(*(std::make_shared<Protocol_ImuTempGet>()->GetRequest()),false,10000);
      (*getDifoCtrlData_map_ptr_).emplace(CmdRepository720::CreateInstance()->Sp_TemperatureParamGet->GetCmdKey(),getDifoCtrlData_ImuTempGet);
    }
  }
}
