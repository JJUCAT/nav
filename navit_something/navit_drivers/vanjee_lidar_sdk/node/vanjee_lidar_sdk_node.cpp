/*
 * @Author: guo
 * @Date: 2023-01-17 14:29:07
 * @LastEditoWJ: Do not edit
 * @LastEditTime: 2023-03-22 09:57:51
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/node/vanjee_lidar_node.cpp
 */
#include "manager/node_manager.hpp"
#include <vanjee_driver/macro/version.hpp>
#include <signal.h>
#ifdef ROS_FOUND
#include <ros/ros.h>
#include <ros/package.h>
#elif ROS2_FOUND
#include <rclcpp/rclcpp.hpp>
#endif

using namespace vanjee::lidar;
#ifdef ROS2_FOUND
std::mutex g_mtx;
std::condition_variable g_cv;
#endif
static void sigHandler(int sig)
{
    WJ_MSG << "Vanjee-LiDAR-Driver is stopping....." << WJ_REND;
#ifdef ROS_FOUND
    ros::shutdown();
#elif ROS2_FOUND
    g_cv.notify_all();
#endif
}
int main(int argc, char **argv)
{
    signal(SIGINT, sigHandler); 

    WJ_TITLE << "********************************************************" << WJ_REND;
    WJ_TITLE << "**********                                    **********" << WJ_REND;
    WJ_TITLE << "**********  Vanjee_Lidar_SDK Version: v" << VANJEE_LIDAR_VERSION_MAJOR
             << "." << VANJEE_LIDAR_VERSION_MINOR
             << "." << VANJEE_LIDAR_VERSION_PATCH << "  **********" << WJ_REND;
    WJ_TITLE << "**********                                    **********" << WJ_REND;
    WJ_TITLE << "********************************************************" << WJ_REND;
#ifdef ROS_FOUND
    ros::init(argc, argv, "vanjee_lidar_sdk_node", ros::init_options::NoSigintHandler);
#elif ROS2_FOUND

    rclcpp::init(argc, argv);
#endif
    std::string config_path;
#ifdef RUN_IN_ROS_WORKSPACE
    config_path = ros::package::getPath("vanjee_lidar_sdk");
#else
    config_path = (std::string)PROJECT_PATH; 
#endif
    config_path += "/config/config.yaml";
#ifdef ROS_FOUND

    ros::NodeHandle priv_hh("~");
    std::string path;
    priv_hh.param("config_path", path, std::string(""));
    if (!path.empty())
    {
        config_path = path;
    }
#endif
    YAML::Node config;
    try
    {
        config = YAML::LoadFile(config_path);
    }
    catch (...)
    {
        WJ_ERROR << "The format of config file " << config_path
                 << " is wrong. Please check (e.g. indentation)." << WJ_REND;
        return -1;
    }
    std::shared_ptr<NodeManager> demo_ptr = std::make_shared<NodeManager>();
    demo_ptr->init(config);
    demo_ptr->start();
    WJ_MSG << "Vanjee-LiDAR-Driver is running....." << WJ_REND;

#ifdef ROS_FOUND
    ros::spin();
#elif ROS2_FOUND
  std::unique_lock<std::mutex> lck(g_mtx);
  g_cv.wait(lck);
#endif

    return 0;
}