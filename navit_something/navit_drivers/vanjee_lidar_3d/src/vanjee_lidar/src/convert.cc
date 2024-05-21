#include <string>
#include <fstream>
#include <iostream>
#include "vanjee_lidar/convert.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/package.h>

#define HDL_Grabber_toRadians(x) ((x)*M_PI/180.0)
using namespace std;
namespace vanjee_lidar
{
  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :  priv_nh_(private_nh)
  {
    sleep(1);
    YAML::Node config;
    std::string anglePath;
    anglePath = ros::package::getPath("vanjee_lidar");
    anglePath = anglePath.substr(0, anglePath.find_last_of("/"));
    anglePath = anglePath + "/config/config.yaml";
    config = YAML::LoadFile(anglePath);
   
    //std::cout << config["lidar"] << std::endl; //可以直接用下标访问

    YAML::Node lidar_config = lidar::yamlSubNodeAbort(config, "lidar");
    //YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
    //ROS_INFO("lidar_size : %ld",lidar_config.size());
    for(uint8_t i = 0; i < lidar_config.size(); ++i)
    {
      std::string l_lidartype = "";
      //lidar::yamlRead<std::string>(lidar_config[i]["driver"], "lidartype", l_lidartype, 0);
      l_lidartype  = lidar_config[i]["driver"]["lidartype"].as<std::string>();
      //ROS_INFO("lidartype = = = %s",l_lidartype.c_str());
      
      if(l_lidartype == "wlr720")
      {
        data_point[i].reset(new vanjee_rawdata::Lidar720());
        data_point[i]->readyamlconfig(node,private_nh,anglePath,i);

        data_point[i]->setCalibration();
        //data_point[i]->setup();

        data_point[i]->output_pointcloud = node.advertise<sensor_msgs::PointCloud2>(data_point[i]->config_lidaryaml.point_cloudtopic,10);
        data_point[i]->output_pointcloudIMU = node.advertise<sensor_msgs::Imu>(data_point[i]->config_lidaryaml.point_imutopic,10);
        
        data_point[i]->vanjee_pointscan_ = node.subscribe(data_point[i]->config_lidaryaml.point_packetscantopic, 10,
                       &vanjee_rawdata::RawData::processScantopoint, data_point[i],
                       //&Convert::processScan, (Convert *)this,
                       ros::TransportHints().tcpNoDelay(true));

        data_point[i]->vanjee_pointbut_ = node.subscribe(data_point[i]->config_lidaryaml.point_packetscantopic + "_but", 10,
                       &vanjee_rawdata::RawData::initbutt, data_point[i],
                       ros::TransportHints().tcpNoDelay(true));
        
      }
      else if(l_lidartype == "wlr721")
      {
        data_point[i].reset(new vanjee_rawdata::Lidar721());
        data_point[i]->readyamlconfig(node,private_nh,anglePath,i);

        data_point[i]->setCalibration();
        //data_point[i]->setup();

        data_point[i]->output_pointcloud = node.advertise<sensor_msgs::PointCloud2>(data_point[i]->config_lidaryaml.point_cloudtopic,10);
        //data_point[i]->output_pointcloudIMU = node.advertise<sensor_msgs::Imu>(data_point[i]->config_lidaryaml.point_imutopic,10);
        
        data_point[i]->vanjee_pointscan_ = node.subscribe(data_point[i]->config_lidaryaml.point_packetscantopic, 10,
                       &vanjee_rawdata::RawData::processScantopoint, data_point[i],
                       //&Convert::processScan, (Convert *)this,
                       ros::TransportHints().tcpNoDelay(true));

        data_point[i]->vanjee_pointbut_ = node.subscribe(data_point[i]->config_lidaryaml.point_packetscantopic + "_but", 10,
                       &vanjee_rawdata::RawData::initbutt, data_point[i],
                       ros::TransportHints().tcpNoDelay(true));

      }
      usleep(50000);
    }
    
    
  }


} // namespace vanjee_lidar
