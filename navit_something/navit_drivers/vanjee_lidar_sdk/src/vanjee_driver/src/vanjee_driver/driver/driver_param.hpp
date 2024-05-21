/*
 * @Author: guo
 * @Date: 2023-01-19 10:59:25
 * @LastEditors: Guo Xuemei 1242143575@qq.com
 * @LastEditTime: 2023-02-27 18:56:37
 * @FilePath: /vanjee_lidar_sdk_test/src/vanjee_lidar_sdk/src/vanjee_driver/src/vanjee_driver/driver/driver_param.hpp
 */

#pragma once

#include <vanjee_driver/common/wj_log.hpp>
#include <string>
#include <map>
#include <vector>

namespace vanjee
{
namespace lidar
{
    enum LidarType
    {
      vanjee_718h,
      vanjee_719,
      vanjee_719c,
      vanjee_720,
      vanjee_721,
      vanjee_733,
      vanjee_738,
      vanjee_739,
      vanjee_740,
      vanjee_750
    };
    inline std::string lidarTypeToStr(const LidarType &type)
    {
      std::string str = "";
      switch (type)
      {
      case LidarType::vanjee_718h:
        str = "vanjee_718h";
        break;
      case LidarType::vanjee_719:
        str = "vanjee_719";
        break;
      case LidarType::vanjee_719c:
        str = "vanjee_719c";
        break;
      case LidarType::vanjee_720:
        str = "vanjee_720";
        break;
      case LidarType::vanjee_721:
        str = "vanjee_721";
        break;
      case LidarType::vanjee_733:
        str = "vanjee_733";
        break;
      case LidarType::vanjee_738:
        str = "vanjee_738";
        break;
      case LidarType::vanjee_739:
        str = "vanjee_739";
        break;
      case LidarType::vanjee_740:
        str = "vanjee_740";
        break;
      case LidarType::vanjee_750:
        str = "vanjee_750";
        break;
      
      default:
        str = "ERROR";
        WJ_ERROR << "WJ_ERROR" << WJ_REND;
        break;
      }
      return str;
    }
    inline LidarType strToLidarType(const std::string &type)
    {
      if(type == "vanjee_718h")
      {
        return LidarType::vanjee_718h;
      }
      else if(type == "vanjee_719")
      {
        return LidarType::vanjee_719;
      }
      else if(type == "vanjee_719c")
      {
        return LidarType::vanjee_719c;
      }
      else if (type == "vanjee_720")
      {
        return LidarType::vanjee_720;
      }
      else if (type == "vanjee_721")
      {
        return LidarType::vanjee_721;
      }
      else if (type == "vanjee_733")
      {
        return LidarType::vanjee_733;
      }
      else if(type =="vanjee_738")
      {
        return LidarType::vanjee_738;
      }
      else if (type == "vanjee_739")
      {
        return LidarType::vanjee_739;
      }
      else if(type == "vanjee_740")
      {
        return LidarType::vanjee_740;
      }
      else if(type == "vanjee_750")
      {
        return LidarType::vanjee_750;
      }

      else
      {
        WJ_ERROR << "Wrong lidar type: " << type << WJ_REND;
        exit(-1);
      }
    }
    enum InputType
    {
      ONLINE_LIDAR = 1,
      PCAP_FILE,
      RAW_PACKET

    };
    inline std::string inputTypeToStr(const InputType &type)
    {
      std::string str = "";
      switch (type)
      {
      case InputType::ONLINE_LIDAR:
        str = "ONLINE_LIDAR";
        break;
      case InputType::PCAP_FILE:
        str = "PCAP_FILE";
        break;

      default:
        str = "ERROR";
        WJ_ERROR << "WJ_ERROR" << WJ_REND;
        break;
      }
      return str;
    }

    struct WJTransfromParam 
    {
      float x = 0.0f;     
      float y = 0.0f;     
      float z = 0.0f;     
      float roll = 0.0f;  
      float pitch = 0.0f; 
      float yaw = 0.0f;   
      void print() const
      {
        WJ_INFO << "------------------------------------------------------" << WJ_REND;
        WJ_INFO << "          VanjeeLidar Transform Parameters            " << WJ_REND;
        WJ_INFOL << "x: " << x << WJ_REND;
        WJ_INFOL << "y: " << y << WJ_REND;
        WJ_INFOL << "z: " << z << WJ_REND;
        WJ_INFOL << "roll: " << roll << WJ_REND;
        WJ_INFOL << "pitch: " << pitch << WJ_REND;
        WJ_INFOL << "yaw: " << yaw << WJ_REND;
        WJ_INFO << "------------------------------------------------------" << WJ_REND;
      }
    };
    struct WJDecoderParam
    {
      bool config_from_file = true;                                    
      bool wait_for_difop = true;                                       
      float min_distance = 0.0f;                                        
      float max_distance = 0.0f;                                        
      float start_angle = 0.0f;                                         
      float end_angle = 360.0f;                                                                      
      bool use_lidar_clock = false;                                     
      bool dense_points = false;
      bool ts_first_point = false;  
      uint16_t publish_mode = 2; 
      uint16_t rpm = 0;
      bool AlgorithmEnb = true;
      bool BlankPointCompenEnb = true;
      bool OutlierEnb = true;
      bool CloseRangeOutliersEnb = true;
      std::string angle_path_ver = "";                                      
      std::string angle_path_hor = ""; 
      std::string imu_param_path = "";                                     
      WJTransfromParam transform_param;                                 
      void print() const
      {
        WJ_INFO << "------------------------------------------------" << WJ_REND;
        WJ_INFO << "----------VANJEE Decoder Parameters-------------" << WJ_REND;
        WJ_INFOL << "wait_for_difop: " << wait_for_difop << WJ_REND;
        WJ_INFOL << "min_distance: " << min_distance << WJ_REND;
        WJ_INFOL << "max_distance: " << max_distance << WJ_REND;
        WJ_INFOL << "start_angle: " << start_angle << WJ_REND;
        WJ_INFOL << "end_angle: " << end_angle << WJ_REND;
        WJ_INFOL << "use_lidar_clock: " << use_lidar_clock << WJ_REND;
        WJ_INFOL << "dense_point: " << dense_points << WJ_REND;
        WJ_INFOL << "config_from_file: " << config_from_file << WJ_REND;
        WJ_INFOL << "angle_path_ver: " << angle_path_ver << WJ_REND;
        WJ_INFOL << "angle_path_hor: " << angle_path_hor << WJ_REND;
        WJ_INFOL << "imu_param_path: " << imu_param_path << WJ_REND;
        WJ_INFOL << "publish_mode: " << publish_mode << WJ_REND;
        WJ_INFO << "------------------------------------------------" << WJ_REND;
        transform_param.print();
      }
    };
    struct WJInputParam 
    {
      uint16_t connect_type = 1;
      uint16_t host_msop_port = 3333;             
      uint16_t lidar_msop_port = 3001;
      uint16_t difop_port = 0;            
      std::string host_address = "0.0.0.0";  
      std::string lidar_address = "0.0.0.0"; 
      std::string group_address = "0.0.0.0"; 
      std::string pcap_path = "";            
      bool pcap_repeat = true;               
      float pcap_rate = 0.0f;                
      bool use_vlan = false;                 
      uint16_t user_layer_bytes = 0;         
      uint16_t tail_layer_bytes = 0;         
      void print() const
      {
        WJ_INFO << "-----------------------------------------------------------" << WJ_REND;
        WJ_INFO << "             VANJEE Input Parameters                     " << WJ_REND;
        WJ_INFO << "connect_type: " << (connect_type == 2 ? "tcp" : "udp") << WJ_REND;
        WJ_INFOL << "host_msop_port: " << host_msop_port << WJ_REND;
        WJ_INFOL << "lidar_msop_port: " << lidar_msop_port << WJ_REND; 
        WJ_INFOL << "host_address: " << host_address << WJ_REND;
        WJ_INFOL << "lidar_address: "<< lidar_address <<WJ_REND;
        WJ_INFOL << "group_address: " << group_address << WJ_REND;
        WJ_INFOL << "pcap_path: " << pcap_path << WJ_REND;
        WJ_INFOL << "pcap_repeat: " << pcap_repeat << WJ_REND;
        WJ_INFOL << "pcap_rate: " << pcap_rate << WJ_REND;
        WJ_INFOL << "use_vlan: " << use_vlan << WJ_REND;
        WJ_INFOL << "user_layer_bytes: " << user_layer_bytes << WJ_REND;
        WJ_INFOL << "tail_layer_bytes: " << tail_layer_bytes << WJ_REND;
        WJ_INFO << "-----------------------------------------------------------" << WJ_REND;
      }
    };
    struct WJDriverParam
    {
      LidarType lidar_type = LidarType::vanjee_721; 
      InputType input_type = InputType::PCAP_FILE;  
      WJInputParam input_param;                     
      WJDecoderParam decoder_param;                 
      void print() const
      {
        WJ_INFO << "----------------------------------------------" << WJ_REND;
        WJ_INFO << "            VANJEE Driver Parameters          " << WJ_REND;
        WJ_INFOL << "input_type: " << inputTypeToStr(input_type) << WJ_REND;
        WJ_INFOL << "lidar_type: " << lidarTypeToStr(lidar_type) << WJ_REND;
        WJ_INFO << "----------------------------------------------" << WJ_REND;
        input_param.print();
        decoder_param.print();
      }
    };
    struct WJMemsParam
    {
      float Rotate_mirror_pitch = 0.0f;       
      std::vector<float> Rotate_mirror_offset{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
      std::vector<float> View_center_yaws{0.0f, 0.0f, 0.0f, 0.0f, 0.0f};    
      float rwadata_yaw_resolution = 0.0f;    
      float rwadata_pitch_resolution = 0.0f;  
      float start_pitch = 0.0f;               
      float end_pitch = 0.0f;                 
      float start_yaw = 0.0f;                 
      float end_yaw = 0.0f;                   
      float beta = 0.0f;                      
      float gama_z = 0.0f;                    
      float gama_y = 0.0f;                    
      bool reversal_horizontzal = false;      
      bool reversal_vertical = false;         
      int scan_echo_type =1;                  
    };
    

} 

} 
