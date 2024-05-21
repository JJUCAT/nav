/** \file
 *
 *  ROS driver implementation for the VANJEE LIDARs
 */
#include "vanjee_driver/vanjee_driver.h"
#include <vanjee_msgs/VanjeeScan.h>

namespace vanjee_driver
{

  vanjeeDriver::vanjeeDriver(ros::NodeHandle node, ros::NodeHandle private_nh)
  {
    YAML::Node config;
    std::string anglePath;
    anglePath = ros::package::getPath("vanjee_driver");
    anglePath = anglePath.substr(0, anglePath.find_last_of("/"));
    anglePath = anglePath + "/config/config.yaml";
    config = YAML::LoadFile(anglePath);
    //std::cout << config["lidar"] << std::endl; //可以直接用下标访问

    // std::string file_name = yjh_common_module::File::GetPackagePath(" vanjee_driver")+"/cfg/status.yaml";
    // yjh_common_module::ModuleStatus::Initialize(file_name);

    YAML::Node lidar_config = lidar::yamlSubNodeAbort(config, "lidar");
    //YAML::Node lidar_config = yamlSubNodeAbort(config, "lidar");
    ROS_INFO("************************* Welcome to use Vanjee Lidar *************************");
    ROS_INFO("lidar_size : %ld",lidar_config.size());
    for (uint8_t i = 0; i < lidar_config.size(); ++i)
    {
      std::string l_lidartype = "";
      std::string l_pcappath = "";
      std::string l_lidarIp = "";
      std::string l_packettopic = "";
      int l_lidarport = 0;
      //lidar::yamlRead<std::string>(lidar_config[i]["driver"], "lidartype", l_lidartype, 0);
      l_lidartype  = lidar_config[i]["driver"]["lidartype"].as<std::string>();
      ROS_INFO("lidartype : %s",l_lidartype.c_str());
      l_pcappath  = lidar_config[i]["driver"]["pcap"].as<std::string>();
      //ROS_INFO("l_pcappath : %s",l_pcappath.c_str());
      l_lidarIp  = lidar_config[i]["proto"]["lidar_ip"].as<std::string>();
      ROS_INFO("lidarIp : %s",l_lidarIp.c_str());
      l_lidarport  = lidar_config[i]["proto"]["dest_port"].as<int>();
      ROS_INFO("lidarport : %d",l_lidarport);
      l_packettopic = lidar_config[i]["ros_topic"]["ros_recv_packet_topic"].as<std::string>();
      ROS_INFO("packettopic : %s",l_packettopic.c_str());

      bool multicast = lidar_config[i]["driver"]["multicast"].as<bool>();
      std::string l_destIp = "";
      if(multicast == true)
      {
        l_destIp = lidar_config[i]["proto"]["dest_ip"].as<std::string>();
      }

      std::string l_localIp = lidar_config[i]["proto"]["local_ip"].as<std::string>();


      if(l_lidartype == "wlr720")
      {
        if(l_pcappath != "nothing")
        {
          lidarTnput[i].reset(new vanjee_driver::InputPCAP720(node, l_lidarport, 1800, l_pcappath));
          ROS_INFO("Get NO.%d Lidar,the type is %s ,pointcloud from pcap %s",i,l_lidartype.c_str(),l_pcappath.c_str());

          lidarTnput[i]->socketpublisher = node.advertise<vanjee_msgs::VanjeeScan>(l_packettopic, 10);
          lidarTnput[i]->sock_butpub = private_nh.advertise<sensor_msgs::Imu>(l_packettopic + "_but", 10);

          lidarTnput[i]->Read_configyaml(anglePath,"",i);
          lidarTnput[i]->startpublisher();
        }
        else
        {
          lidarTnput[i].reset(new vanjee_driver::InputSocket720(node, l_lidarport,anglePath,i,l_destIp,l_localIp));
          ROS_INFO("Get NO.%d Lidar,the type is %s,pointcloud from omline lidar [%s,3333]",i,l_lidartype.c_str(),l_lidarIp.c_str());

          lidarTnput[i]->socketpublisher = node.advertise<vanjee_msgs::VanjeeScan>(l_packettopic, 10);
          lidarTnput[i]->sock_butpub = node.advertise<sensor_msgs::Imu>(l_packettopic + "_but", 10);

          lidarTnput[i]->Read_configyaml(anglePath,"",i);
          lidarTnput[i]->startpublisher();
        }
      }
      else if (l_lidartype == "wlr721")
      {
        if(l_pcappath != "nothing")
        {
          lidarTnput[i].reset(new vanjee_driver::InputPCAP721(node, l_lidarport, 1800, l_pcappath));
          ROS_INFO("Get NO.%d Lidar,the type is %s ,pointcloud from pcap %s",i,l_lidartype.c_str(),l_pcappath.c_str());

          lidarTnput[i]->socketpublisher = node.advertise<vanjee_msgs::VanjeeScan>(l_packettopic, 10);
          lidarTnput[i]->sock_butpub = private_nh.advertise<sensor_msgs::Imu>(l_packettopic + "_but", 10);

          lidarTnput[i]->Read_configyaml(anglePath,"",i);

          lidarTnput[i]->startpublisher();

          //ROS_INFO("Get NO.%d Lidar,the type is %s ,pointcloud from pcap %s",i,l_lidartype.c_str(),l_pcappath.c_str());

        }
        else
        {
          lidarTnput[i].reset(new vanjee_driver::InputSocket721(node, l_lidarport,anglePath,i,l_destIp ,l_localIp));
          ROS_INFO("Get NO.%d Lidar,the type is %s,pointcloud from omline lidar [%s,3333]",i,l_lidartype.c_str(),l_lidarIp.c_str());

          lidarTnput[i]->socketpublisher = node.advertise<vanjee_msgs::VanjeeScan>(l_packettopic, 10);
          lidarTnput[i]->sock_butpub = node.advertise<sensor_msgs::Imu>(l_packettopic + "_but", 10);

          lidarTnput[i]->Read_configyaml(anglePath,"",i);
          lidarTnput[i]->startpublisher();
        }
      }
      usleep(50000);
    }


  }

  vanjeeDriver::~vanjeeDriver()
  {
    if (difop_thread_ != NULL)
    {
      difop_thread_->interrupt();
      difop_thread_->join();
    }
  }

  /** poll the device
   *
   *  @returns true unless end of file reached
   */
  bool vanjeeDriver::poll(void)
  {
    ros::Rate loop_rate(120);
    loop_rate.sleep();
    return true;
  }




}
