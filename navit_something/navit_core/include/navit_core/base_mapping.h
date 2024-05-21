#ifndef NAVIT_CORE_MAPPING_H
#define NAVIT_CORE_MAPPING_H

#warning navit_core/base_mapping.h has been deprecated

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
namespace navit_core {
/**
 * @class mapping
*/
class Mapping {
  public:
    using Ptr = boost::shared_ptr<Mapping>;

    virtual void initialize(const std::string& name) = 0;

    virtual bool stopMapping() = 0;

    virtual bool saveMap(const std::string& map_name) = 0;

    virtual ~Mapping(){}

    enum MappingMode : uint8_t {
        STOP_MAPPING = 1,
        SAVE_MAP = 2
    };

  protected:
    Mapping(){}
};
  
};  // namespace navit_core

#endif  // NAVIT_CORE_MAPPING_H
