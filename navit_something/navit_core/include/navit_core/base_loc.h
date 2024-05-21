/*
* @Author: czk
* @Date:   2022-06-28 17:59:23
* @Last Modified by:   chenzongkui
* @Last Modified time: 2022-06-28 19:34:24
*/
#ifndef LOCALIZATION_H_
#define LOCALIZATION_H_

#warning navit_core/base_loc.h has been deprecated

namespace navit_core {
/**
 * @class Slam
*/
class Localization {
  public:
    using Ptr = boost::shared_ptr<Localization>;

    virtual void initialize(const std::string& name) = 0;

    virtual ~Localization(){}

  protected:
    Localization(){}
};

}  // namespace navit_core

#endif  // LOCALIZATION_H_
