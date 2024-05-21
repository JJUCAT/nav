//
// Created by fan on 23-8-14.
//

#include "ros/ros.h"
#include "navit_map_server/navit_map_server.h"
#include <filesystem>

#define USAGE                                                                                                          \
  "\nUSAGE: navit_map_server <map.json>\n"                                                                             \
  "  map.json: map json file.\n"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "navit_map_server_node");
  // ros::init(argc, argv, "navit_map_server_node", ros::init_options::AnonymousName);
  ros::NodeHandle nh("~");
  if (argc != 2)
  {
    ROS_ERROR("%s", USAGE);
    exit(-1);
  }

  std::string fname(argv[1]);
  ROS_INFO("start navit_map_server with file %s", fname.c_str());

  std::filesystem::path file_path(fname);
  if (!std::filesystem::exists(file_path))
  {
    printf("file is not exist. %s\n", file_path.c_str());
    return -1;
  }

  try
  {
    navit_map_server::NavitMapServer mapServer(fname);
    ros::spin();
  }
  catch (std::runtime_error& e)
  {
    ROS_ERROR("navit_map_server exception: %s", e.what());
    return -1;
  }

  ROS_INFO("stop navit_map_server with file %s", fname.c_str());

  return 0;
}