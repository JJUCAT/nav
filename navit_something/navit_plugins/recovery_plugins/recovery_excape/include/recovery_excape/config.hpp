#ifndef RECOVERY_EXCAPE__CONFIGURATION_HPP_
#define RECOVERY_EXCAPE__CONFIGURATION_HPP_

#include <ros/ros.h>

namespace recovery_excape {

class Configuration
{
 public:

  Configuration(ros::NodeHandle& nh)
    : nh_(nh)
  {
    Get();
  }

  ~Configuration() {}

  explicit Configuration(const Configuration& config)
  {
    nh_ = config.nh_;
    cmd_vel_topic = config.cmd_vel_topic;
    loop = config.loop;
    wheel_base = config.wheel_base;
    head = config.head;
    base_free = config.base_free;
    head_free = config.head_free;
  }

  Configuration& operator=(const Configuration& config)
  {
    if (this != &config) {
      nh_ = config.nh_;
      cmd_vel_topic = config.cmd_vel_topic;
      loop = config.loop;
      wheel_base = config.wheel_base;
      head = config.head;
      base_free = config.base_free;
      head_free = config.head_free;
    }
    return *this;
  }

  void Get()
  {
    nh_.param("loop", loop, loop);
    nh_.param("wheel_base", wheel_base, wheel_base);
    nh_.param("head", head, head);
    nh_.param("base_free", base_free, base_free);
    nh_.param("head_free", head_free, head_free);
  }

  std::string cmd_vel_topic{"cmd_vel"};
  int loop{5};
  double wheel_base{1.2};
  double head{1.7};
  int base_free{200};  
  int head_free{130};

 private:

  ros::NodeHandle nh_;

}; // class Configuration

} // namespace recovery_excape

#endif // RECOVERY_EXCAPE__CONFIGURATION_HPP_
