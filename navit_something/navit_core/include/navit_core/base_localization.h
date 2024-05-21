#ifndef BASE_LOCALIZATION_H_
#define BASE_LOCALIZATION_H_

#include <boost/shared_ptr.hpp>

namespace navit_core {
/**
 * @class localization base class
*/
class BaseLocalization {
  public:
    using Ptr = boost::shared_ptr<BaseLocalization>;

    virtual ~BaseLocalization(){}

    virtual void initialize(const std::string& name) = 0;

    virtual void start() = 0;

    virtual void stop() = 0;

  protected:
    BaseLocalization(){}
};

}  // namespace navit_core

#endif  // BASE_LOCALIZATION_H_
