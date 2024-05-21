#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository721
    {
      public:
        const std::shared_ptr<CmdClass> Sp_LDAngleGet = std::make_shared<CmdClass>(0x05,0x14);
        static CmdRepository721* CreateInstance()
        {
          if(p_cmdRepository == nullptr)
            p_cmdRepository = new CmdRepository721();

          return p_cmdRepository;
        }
      private:
        static CmdRepository721* p_cmdRepository;
        CmdRepository721(){}
        CmdRepository721(const CmdRepository721&) = delete;
        CmdRepository721& operator=(const CmdRepository721&) = delete;
    };

    CmdRepository721* CmdRepository721::p_cmdRepository = nullptr;
  }
}