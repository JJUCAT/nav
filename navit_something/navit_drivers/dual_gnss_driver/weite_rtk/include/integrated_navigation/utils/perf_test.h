/************************************************
Created by mikaa-mi on 20-4-3.
**************************************************/

#ifndef PERCEPTION_PERF_TEST_H
#define PERCEPTION_PERF_TEST_H

#include <chrono>

#include "ros/ros.h"

using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

class Timer {
 public:
  Timer() = default;

  // no-thread safe.
  void Start() { start_time_ = std::chrono::high_resolution_clock::now(); };

  double End(const std::string &msg) {
    end_time_ = std::chrono::high_resolution_clock::now();
    auto dur_time = std::chrono::duration<double>(end_time_ - start_time_).count();
    ROS_INFO_STREAM(
        "[TimeDebug]" << std::fixed << std::setprecision(6) << msg << " elapsed_time: " << dur_time << " s");
    start_time_ = end_time_;
    // return elapsed_time;
    return dur_time;
  };

 private:
  TimePoint start_time_;
  TimePoint end_time_;

  // DISALLOW_COPY_AND_ASSIGN(Timer);
};

class PerfTestWrapper {
 public:
  explicit PerfTestWrapper(const std::string &msg) : msg_(msg) { timer_.Start(); }

  ~PerfTestWrapper() { timer_.End(msg_); }

 private:
  Timer timer_;
  std::string msg_;
};
#ifdef PERF_TEST
#define PERF_TEST_FUNCTION(function_name) PerfTestWrapper _timer_wrapper_(function_name)

#define PERF_TEST_START() \
  Timer _timer_;          \
  _timer_.Start()

#define PERF_TEST_END(msg) _timer_.End(msg)
#define PERF_TEST_CUR(msg) ROS_INFO_STREAM(std::fixed << std::setprecision(6) << msg << ros::Time::now().toSec());
#else
#define PERF_TEST_FUNCTION(function_name)
#define PERF_TEST_START()
#define PERF_TEST_END(msg)
#define PERF_TEST_CUR(msg)
#endif

#endif  // PERCEPTION_PERF_TEST_H
