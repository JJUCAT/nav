#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository719
    {
      public:
        const std::shared_ptr<CmdClass> Sp_ScanDataGet = std::make_shared<CmdClass>(0x05,0x03);
        const std::shared_ptr<CmdClass> Sp_HeartBit = std::make_shared<CmdClass>(0x04,0x04);
        static CmdRepository719* CreateInstance()
        {
          if(p_CmdRepository719 == nullptr)
            p_CmdRepository719 = new CmdRepository719();

          return p_CmdRepository719;
        }
      private:
        static CmdRepository719* p_CmdRepository719;
        CmdRepository719(){}
        CmdRepository719(const CmdRepository719&) = delete;
        CmdRepository719& operator=(const CmdRepository719&) = delete;
    };

    CmdRepository719* CmdRepository719::p_CmdRepository719 = nullptr;
  }
}