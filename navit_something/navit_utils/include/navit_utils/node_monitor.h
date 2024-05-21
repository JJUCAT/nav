#ifndef SYSTEM_MONITOR_H
#define SYSTEM_MONITOR_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include "cpu_util.h"  
#include <string>
#include <vector>

//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>

//namespace py = pybind11;
namespace navit_utils {
class SystemMonitor {
public:
  SystemMonitor();
  void spin();
  void plotUsageGraph(const std::map<std::string, std::pair<float, float>>& nodeUsage);

private:
  ros::NodeHandle nh_{"~"};

  std::map<std::string, std::pair<uint64_t, uint64_t>> process_info_;
  std::map<std::string, std::pair<float, float>> node_usage_;

  std::vector<std::string> process_names_;

  ros::Publisher pub_;
  float monitor_frequency_;
  float total_cpu_, total_memory_;
  void monitorSystem();
  bool first_hit_ = true;
  uint64_t cpu_total_occupy_last_, cpu_proc_occupy_last_;
  int cpu_nums_;
};
} //namespace navit_utils
#endif  // SYSTEM_MONITOR_H