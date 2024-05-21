#pragma once

#include <vanjee_driver/driver/difop/cmd_class.hpp>

namespace vanjee
{
  namespace lidar
  {
    class CmdRepository740
    {
      public:
        static CmdRepository740* CreateInstance()
        {
          if(p_cmdRepository == nullptr)
            p_cmdRepository = new CmdRepository740();

          return p_cmdRepository;
        }
      private:
        static CmdRepository740* p_cmdRepository;
        CmdRepository740(){}
        CmdRepository740(const CmdRepository740&) = delete;
        CmdRepository740& operator=(const CmdRepository740&) = delete;
    };

    CmdRepository740* CmdRepository740::p_cmdRepository = nullptr;
  }
}