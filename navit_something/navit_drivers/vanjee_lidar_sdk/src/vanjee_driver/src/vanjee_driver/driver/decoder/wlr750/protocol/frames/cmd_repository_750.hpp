#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository750
    {
      public:
        static CmdRepository750* CreateInstance()
        {
          if(p_CmdRepository == nullptr)
            p_CmdRepository = new CmdRepository750();

          return p_CmdRepository;
        }
      private:
        static CmdRepository750* p_CmdRepository;
        CmdRepository750(){}
        CmdRepository750(const CmdRepository750&) = delete;
        CmdRepository750& operator=(const CmdRepository750&) = delete;
    };

    CmdRepository750* CmdRepository750::p_CmdRepository = nullptr;
  }
}