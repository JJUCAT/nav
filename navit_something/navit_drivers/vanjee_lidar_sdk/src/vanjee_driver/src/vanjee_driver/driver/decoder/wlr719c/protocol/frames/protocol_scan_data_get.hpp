#pragma once

#include "protocol_abstract_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/params/params_scan_data.hpp"
#include "vanjee_driver/driver/difop/params_abstract.hpp"

namespace vanjee
{
  namespace lidar
  {
    class Protocol_ScanDataGet719C:public ProtocolAbstract719C
    {
      public:
        Protocol_ScanDataGet719C():ProtocolAbstract719C(CmdRepository719C::CreateInstance()->Sp_ScanDataGet,std::make_shared<Params_ScanData719C>())
        {
          
        }

        std::shared_ptr<std::vector<uint8>> GetRequest(std::shared_ptr<std::vector<uint8>> content = nullptr)
        {
          if(content == nullptr)
          {
            const uint8 arr[] = {0x00,0x00,0x00,0x00};
            content = std::make_shared<std::vector<uint8>>();
            content->insert(content->end(),arr,arr+sizeof(arr)/sizeof(uint8));
          }
          
          return (std::make_shared<ProtocolBase>(ByteVector({0x00,0x00}),ByteVector({0x00,0x00,0x00,0x00}),CheckType,Type,
            DeviceType,Remain,Sp_Cmd->MainCmd,Sp_Cmd->SubCmd,ByteVector({0x00,0x01}),*content))->GetBytes();
        }
    };
  }
}