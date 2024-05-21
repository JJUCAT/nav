#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository719C
    {
      public:
        const std::shared_ptr<CmdClass> Sp_ScanDataGet = std::make_shared<CmdClass>(0x01,0x01);
        const std::shared_ptr<CmdClass> Sp_HeartBit_Tcp = std::make_shared<CmdClass>(0x04,0x04);
        const std::shared_ptr<CmdClass> Sp_HeartBit_Udp = std::make_shared<CmdClass>(0x05,0x03);
        static CmdRepository719C* CreateInstance()
        {
          if(p_CmdRepository719C == nullptr)
            p_CmdRepository719C = new CmdRepository719C();

          return p_CmdRepository719C;
        }
      private:
        static CmdRepository719C* p_CmdRepository719C;
        CmdRepository719C(){}
        CmdRepository719C(const CmdRepository719C&) = delete;
        CmdRepository719C& operator=(const CmdRepository719C&) = delete;
    };

    CmdRepository719C* CmdRepository719C::p_CmdRepository719C = nullptr;
  }
}