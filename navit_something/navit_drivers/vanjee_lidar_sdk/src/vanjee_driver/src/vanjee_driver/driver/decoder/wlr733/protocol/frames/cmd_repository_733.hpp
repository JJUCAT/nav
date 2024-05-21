#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository733
    {
      public:
        const std::shared_ptr<CmdClass> Sp_LDAngleGet = std::make_shared<CmdClass>(0x06,0x03);
        static CmdRepository733* CreateInstance()
        {
          if(p_CmdRepository == nullptr)
            p_CmdRepository = new CmdRepository733();

          return p_CmdRepository;
        }
      private:
        static CmdRepository733* p_CmdRepository;
        CmdRepository733(){}
        CmdRepository733(const CmdRepository733&) = delete;
        CmdRepository733& operator=(const CmdRepository733&) = delete;
    };

    CmdRepository733* CmdRepository733::p_CmdRepository = nullptr;
  }
}