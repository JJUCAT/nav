#include "navit_utils/node_monitor.h"

namespace navit_utils {
SystemMonitor::SystemMonitor() {
  nh_.getParam("process_names", process_names_);
  nh_.param("monitor_frequency", monitor_frequency_, 1.0f);
  nh_.param("cpu_nums", cpu_nums_, 4);
  
  // pub
  pub_ = nh_.advertise<std_msgs::String>("system_monitor", 10);
}

void SystemMonitor::spin() {
  ros::Rate loop_rate(monitor_frequency_);

  for (const auto& process_name : process_names_) {
    std::cout << "We will monitor process node: " << process_name << std::endl;
    process_info_.insert({process_name, std::pair<uint64_t, uint64_t>(0, 0)});
  }

  total_memory_ = getTotalMemery();
  while (ros::ok()) {
    monitorSystem();
    plotUsageGraph(node_usage_);
    loop_rate.sleep();
  }
}

void SystemMonitor::plotUsageGraph(const std::map<std::string, std::pair<float, float>>& nodeUsage) {
    // py::scoped_interpreter guard{};
    // py::object matplotlib = py::module::import("matplotlib.pyplot");
    
    // for (const auto& node : nodeUsage) {
    //     std::string nodeName = node.first;
    //     float cpuUsage = node.second.first;
    //     float memoryUsage = node.second.second;

    //     matplotlib.attr("plot")(cpuUsage, py::arg("label") = "CPU Usage - " + nodeName);
    //     matplotlib.attr("plot")(memoryUsage, py::arg("label") = "Memory Usage - " + nodeName);
    // }
    
    // matplotlib.attr("title")("Node CPU and Memory Usage");
    // matplotlib.attr("xlabel")("Time");
    // matplotlib.attr("ylabel")("Usage (%)");
    
    // matplotlib.attr("legend")();
    // matplotlib.attr("show")();
}

void SystemMonitor::monitorSystem() {
  for (const auto& process_name : process_names_) {
    std::vector<int32_t> pids;
    std_msgs::String msg;
    if (navit_utils::getProcessPids(process_name, &pids)) {
      for (const auto& pid : pids) {
        if (first_hit_) {
          process_info_[process_name].first = navit_utils::getCpuTotalOccupy();
          process_info_[process_name].second = navit_utils::getCpuProcOccupy(pid);
          first_hit_ = false;
          continue;
        }

        uint64_t cpu_total_occupy = getCpuTotalOccupy();
        uint64_t cpu_proc_occupy = getCpuProcOccupy(pid);

        uint32_t memory = navit_utils::getProcMemory(pid);
        float cpu_occ_per = float(cpu_proc_occupy - process_info_[process_name].second)/float(cpu_total_occupy - process_info_[process_name].first)* 100.0f * cpu_nums_;
        float mem_occ_per = (memory/total_memory_* 100.0f);
        msg.data = "Process: " + process_name + ", PID: " + std::to_string(pid)
                 + ", CPU Occupy: " + std::to_string(cpu_occ_per)
                 + ", Memory: " + std::to_string(memory/total_memory_* 100.0f) + "\n";
        
        ROS_INFO("Process %s, PID is %d, cpu_occ is %f, memory occ is %f", process_name.c_str(), pid, cpu_occ_per, mem_occ_per);    
        pub_.publish(msg);
        process_info_[process_name].first = cpu_total_occupy;
        process_info_[process_name].second = cpu_proc_occupy;

        node_usage_.insert({process_name, std::pair<uint64_t, uint64_t>(cpu_occ_per, mem_occ_per)});
      }
      pub_.publish(msg);
    }
  }
}

} //namespace navit_utils

int main(int argc, char **argv) {
  ros::init(argc, argv, "navit_utils_node");
  navit_utils::SystemMonitor systemMonitor;
  systemMonitor.spin();
  return 0;
}