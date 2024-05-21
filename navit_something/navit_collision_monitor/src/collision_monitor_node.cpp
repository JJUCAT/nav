// Copyright (c) 2022 Samsung R&D Institute Russia
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>

#include <ros/ros.h>

#include "navit_collision_monitor/collision_monitor.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "collision_monitor_node");
  ros::NodeHandle nh("~");
  auto node = std::make_shared<navit_collision_monitor::CollisionMonitor>(nh);
  ros::spin();
  ros::shutdown();
  return 0;
}
