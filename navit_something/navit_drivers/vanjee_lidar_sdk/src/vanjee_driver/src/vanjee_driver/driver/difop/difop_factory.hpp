#pragma once 
#include <memory>

#include "vanjee_driver/driver/difop/difop_base.hpp"
#include "vanjee_driver/driver/difop/protocol_base.hpp"
#include "vanjee_driver/driver/decoder/wlr718h/protocol/difop_vanjee_718h.hpp"
#include "vanjee_driver/driver/decoder/wlr719/protocol/difop_vanjee_719.hpp"
#include "vanjee_driver/driver/decoder/wlr719c/protocol/difop_vanjee_719c.hpp"
#include "vanjee_driver/driver/decoder/wlr720/protocol/difop_vanjee_720.hpp"
#include "vanjee_driver/driver/decoder/wlr721/protocol/difop_vanjee_721.hpp"
#include "vanjee_driver/driver/decoder/wlr733/protocol/difop_vanjee_733.hpp"
#include "vanjee_driver/driver/decoder/wlr740/protocol/difop_vanjee_740.hpp"
#include "vanjee_driver/driver/decoder/wlr750/protocol/difop_vanjee_750.hpp"

namespace vanjee
{
  namespace lidar
  {
    class DifopFactory
    {
      public:
        static std::shared_ptr<DifopBase> createDifop(LidarType type);
    };

    std::shared_ptr<DifopBase> DifopFactory::createDifop(LidarType type)
    {
      std::shared_ptr<DifopBase> ret_ptr;
      ProtocolBase pb;
      
      switch (type)
      {
        case LidarType::vanjee_718h:
          ret_ptr = std::make_shared<DifopVanjee718H>();
          ProtocolBase::ByteOrder = 2;
          break;
        case LidarType::vanjee_719:
          ret_ptr = std::make_shared<DifopVanjee719>();
          break;
        case LidarType::vanjee_719c:
          ret_ptr = std::make_shared<DifopVanjee719C>();
          break;
        case LidarType::vanjee_720:
          ret_ptr = std::make_shared<DifopVanjee720>(); 
          break;
        case LidarType::vanjee_721:
          ret_ptr = std::make_shared<DifopVanjee721>();
          break;
        case LidarType::vanjee_733:
          ret_ptr = std::make_shared<DifopVanjee733>(); 
          pb.Idx.resize(4);
          ret_ptr->setOrgProtocolBase(pb);
          ProtocolBase::ByteOrder = 2;
          break;
        case LidarType::vanjee_740:
          ret_ptr = std::make_shared<DifopVanjee740>();
          break;
        case LidarType::vanjee_750:
          ret_ptr = std::make_shared<DifopVanjee750>();
          break;
        
        default:
          exit(-1);
      }

      return ret_ptr;
    }
  }
}