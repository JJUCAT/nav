/*
 * This file is part of wjlidar_721 driver.
 *
 * The driver is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * The driver is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with the driver.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "vanjee_driver/driver721.h"
#include <iomanip>
#include <fstream>
#include <iostream>
#include <sensor_msgs/Imu.h>

using namespace std;
// extern volatile sig_atomic_t flag;
namespace vanjee_driver
{
  ////////////////////////////////////////////////////////////////////////
  // Input base class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number.
   */

  ////////////////////////////////////////////////////////////////////////
  // InputSocket721 class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param con_path path to config.yaml
   *  @param con_part part of config.yaml
   */
  InputSocket721::InputSocket721(ros::NodeHandle private_nh, uint16_t port ,std::string con_path,int con_part,std::string l_destIp,std::string l_localIp) : Input(private_nh, port,l_destIp,l_localIp)
  {
    sockfd_ = -1;
    npackets = 120;
    mangstr = 0;
    mangend = 360;

    Write0401Flag = false;


    congig_yaml.ReadVerAngFlag = true;
    requestBaseParam(con_path,con_part);

    // boost::shared_ptr<boost::thread> ReadBothreadHandle;
    // ReadBothreadHandle.reset(new boost::thread(boost::bind(&InputSocket721::ReadBootton,this,sock_udp)));

    boost::shared_ptr<boost::thread> ReadBaseparamHandle;
    ReadBaseparamHandle.reset(new boost::thread(boost::bind(&InputSocket721::ReadBaseParam,this,sock_udp)));

    boost::shared_ptr<boost::thread> ReadVerAngHandle;
    ReadVerAngHandle.reset(new boost::thread(boost::bind(&InputSocket721::ReadVerAngleResolution,this,sock_udp)));
  }

  /** @brief destructor */
  InputSocket721::~InputSocket721(void)
  {
    (void)close(sockfd_);
  }

  int InputSocket721::ReadBaseParam(int &sock)
  {
    ros::Rate loop_rate1(10); // 10hz - 100ms
    loop_rate1.sleep();
    for (int i = 0; i < 3; i++)
    {
      // request Base
      // requestDevBaseParam(sock);
      // loop_rate1.sleep();

      // request sn
      requestSN(sock);
      loop_rate1.sleep();

      // request vertical angle resolution
      requestVerAngleResolution(sock);
      loop_rate1.sleep();
    }
  }


  /** @brief recv  Request thread. */
  int InputSocket721::requestBaseParam(std::string config_path, int part_i)
  {
    /****************************************************************************************/
    YAML::Node config;
    config = YAML::LoadFile(config_path);
    //std::cout << config["lidar"] << std::endl; //可以直接用下标访问
    YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
    int l_port  = common_config[part_i]["proto"]["dest_port"].as<int>();

    bool l_milticast = common_config[part_i]["driver"]["multicast"].as<bool>();
    
    if(l_milticast)
    {
      devip_str_= common_config[part_i]["proto"]["dest_ip"].as<std::string>();
    }
    else
    {
      devip_str_= common_config[part_i]["proto"]["lidar_ip"].as<std::string>();
    }
    ROS_INFO("enpoint: [%s : 3333]",devip_str_.c_str());

    std::string l_csvpath = common_config[part_i]["driver"]["calibration"].as<std::string>();
    l_csvpath = ros::package::getPath("vanjee_lidar") + "/data/Vanjee_lidar_64/" + l_csvpath;
    m_VerCSVfile = l_csvpath;

    /*****************************************************************************************/

    // int sockfd = socket(PF_INET, SOCK_DGRAM, 0);
    // if (sockfd == -1)
    // {
    //   perror("socket"); // TODO: ROS_ERROR errno
    //   return -1;
    // }

    // int opt = 1;
    // //必须加SO_BROADCAST，否则sendto会显示Permission denied
    // if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR /*|SO_BROADCAST*/, (const void *)&opt, sizeof(opt)))
    // {
    //   perror("setsockopt error!\n");
    // }

    // sockaddr_in my_addr;                  // my address information
    // memset(&my_addr, 0, sizeof(my_addr)); // initialize to zeros
    // my_addr.sin_family = AF_INET;         // host byte order
    // my_addr.sin_port = htons(3001);       // port in network byte order
    // my_addr.sin_addr.s_addr = INADDR_ANY; // inet_addr("192.168.2.88");
    // // my_addr.sin_addr.s_addr = inet_addr("192.168.2.88");

    // if (bind(sockfd, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
    // {
    //   perror("bind"); // TODO: ROS_ERROR errno
    //   return -1;
    // }
    // sockaddr_in remote_addr;                                     // remote address information
    // memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    // remote_addr.sin_family = AF_INET;                            // host byte order
    // remote_addr.sin_port = htons(3333);                          // port in network byte order
    // remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP

    // // create recv thread
    // gotRequest = 0;

    // ros::Rate loop_rate1(10); // 10hz - 100ms
    // loop_rate1.sleep();

    // for (int i = 0; i < 3; i++)
    // {
    //   // request sn
    //   requestDevBaseParam(sockfd);
    //   loop_rate1.sleep();

    //   // request sn
    //   requestSN(sockfd);
    //   loop_rate1.sleep();

    //   // request vertical angle resolution
    //   requestVerAngleResolution(sockfd);
    //   loop_rate1.sleep();
    // }

    // // block to wait request reply
    // ros::Rate loop_rate(100);      // 100hz - 10ms
    // for (int i = 0; i < 300; i++) // 11s
    // {
    //   if (gotRequest == 3)
    //   {
    //     break;
    //   }
    //   loop_rate.sleep();
    // }

    // ROS_INFO("got Request reply:%d", gotRequest);

    // close(sockfd);

    return 0;
  }

  

  int InputSocket721::setCaliHval(std::string calibrapath)
	{
		ifstream file;
		file.open(calibrapath.c_str());
		unsigned int i = 0;
		//fprintf(stderr, "set Calibration in pointcloud\n");
		if (!file)
		{
			ROS_WARN_STREAM("[cloud][rawdata] " << calibrapath << " does not exist");
			fprintf(stderr, "%s\n", calibrapath.c_str());
			return -1;
		}
		else
		{
			string wow, mem, key;
			unsigned int x = 0;
			while (true)
			{
				getline(file, wow);
				if (file.fail())
					break; // check for error
				while (x < wow.length())
				{
					if (wow[x] == ',')
					{
						key = mem;
						mem.clear();
						x++; // step over ','
					}
					else
						mem += wow[x++];
				}
				istringstream isAzimuth(mem);
				isAzimuth >> AzimuthHori[i];
				//fprintf(stderr,"output:%f,%f\n",VERT_ANGLE[i],AzimuthDiff[i]);
				i++;
				mem.clear(); // reset memory
				key.clear();
				x = 0; // reset index
			}
			file.close();

			return 0;
		}
	}

  int InputSocket721::ReadBootton(int &sock)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    unsigned char g_cDebug_QueryParamUFrameBuf[34] = {0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t check = 0x00;
    for (int i = 2; i < sizeof(g_cDebug_QueryParamUFrameBuf) - 4; i++)
      check ^= g_cDebug_QueryParamUFrameBuf[i];
    g_cDebug_QueryParamUFrameBuf[31] = check; // check_bit
    g_cDebug_QueryParamUFrameBuf[32] = 0xEE;  // frame_tail
    g_cDebug_QueryParamUFrameBuf[33] = 0xEE;  // frame_tail

    ros::Rate loop_rate1(1); // 10hz - 100ms
    while (1)
    {
      if (sendto(sock, g_cDebug_QueryParamUFrameBuf, sizeof(g_cDebug_QueryParamUFrameBuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
      {
        perror("sendto request vertical angle resolution");
        // fprintf(stderr,"1**************2**************3*******\n");
        close(sock);
        return -1;
      }
      else
      {
        // fprintf(stderr,"1***********************************\n");
      }
      // sendto(sock, g_cDebug_QueryParamUFrameBuf, sizeof(g_cDebug_QueryParamUFrameBuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr));
      // fprintf(stderr,"1***********************************\n");

      loop_rate1.sleep();
    }
    fprintf(stderr, "out to read botton thread ***********************************\n");
    return 0;
  }

  int InputSocket721::ReadVerAngleResolution(int &sock)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x14};
    uint8_t check = 0x00; //用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[30] = 0x00;  // check_bit
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail
   
    while (congig_yaml.ReadVerAngFlag)
    {
      if (sendto(sock, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
      {
        perror("sendto request vertical angle resolution");
        close(sock);
        return -1;
      }
      
      sleep(3);
    }

    return 0;
  }

  void InputSocket721::refreshPackets(int &packs, unsigned char *pbuf, int rpm)
  {
    if (packs != npackets)
    {
      if (pbuf[0] == 0xFF && pbuf[1] == 0xEE)
      {
        npackets = 72000 / rpm;
        packs = npackets;
        fprintf(stderr, "refreshPackets2:%d rpm:%d\n", npackets, rpm);
      }

      packs = npackets;
      fprintf(stderr, "refreshPackets:%d\n", npackets);
    }
  }

 void InputSocket721::resleepPackets(int rpm,double time)
 {

 }

  /** @brief send a Request to get device base param. */
  int InputSocket721::requestDevBaseParam(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01};
    uint8_t check = 0x00; //用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail
    if (sendto(sockfd, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
    {
      perror("sendto request vertical angle resolution");
      close(sockfd);
      return -1;
    }
    return 0;
  }

  /** @brief send a Request to get  SN num. */
  int InputSocket721::requestSN(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01};
    uint8_t check = 0x00; //用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail
    if (sendto(sockfd, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
    {
      perror("sendto request vertical angle resolution");
      close(sockfd);
      return -1;
    }
    return 0;
  }

  void InputSocket721::Write0401(std::string l_data)
  {
    if (!Write0401Flag)
    {
      const char *pbuf = l_data.c_str();
      
      if (l_data.length() != 150)
      {
        ROS_ERROR("Wrong length of SN %d",l_data.length());
        return;
      }

      char sn[20] = {0};
      memcpy(sn, &pbuf[26], 16);
      sn[16] = 0;
      ROS_INFO("[SN] %s", sn);
      Write0401Flag = true;
    }
  }

  /** @brief send a Request to get  vertical angle resolution. */
  int InputSocket721::requestVerAngleResolution(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x14};
    uint8_t check = 0x00; //用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[30] = 0x00;  // check_bit
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail

    if (sendto(sockfd, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
    {
      perror("sendto request vertical angle resolution");
      close(sockfd);
      return -1;
    }

    return 0;
  }

  void InputSocket721::startpublisher()
  {
    boost::shared_ptr<boost::thread> udpthread_ptr1;
    udpthread_ptr1.reset(new boost::thread(boost::bind(&InputSocket721::euqueuepacket, this)));

  }

  void InputSocket721::WriVerAngtoCSV(std::string l_data)
  {
    if (congig_yaml.ReadVerAngFlag == true)
    {
      
      gotRequest++;
      ROS_INFO("got vertical angle resolution");

      float recvVerAngle[64] = {0};
      for (int j = 0; j < 64; j++)
        recvVerAngle[j] = ((unsigned char)l_data.at(30 + j * 4) << 24 
                        | (unsigned char)l_data.at(29 + j * 4) << 16 
                        | (unsigned char)l_data.at(28 + j * 4) << 8 
                        | (unsigned char)l_data.at(27 + j * 4)) * 1.0 / 1000;

      // save vertical angle resolution to file
      std::string anglePath = m_VerCSVfile;

      setCaliHval(ros::package::getPath("vanjee_lidar") + "/data/Vanjee_lidar_64/Vanjee64-carside.csv"); //获取水平角度

      ofstream file;
      file.open(anglePath.c_str());
      if (!file)
      {
        ROS_WARN_STREAM("[cloud][rawdata][721]" << anglePath << " does not exist");
        fprintf(stderr, "%s\n", anglePath.c_str());
      }
      else
      {
        for (int j = 0; j < 64; j++)
          file << fixed << setprecision(3) << recvVerAngle[j] << "," << AzimuthHori[j] << std::endl;
      }
      file.close();
      congig_yaml.ReadVerAngFlag = false;
    }
  }

  void InputSocket721::euqueuepacket()
  {
    vanjee_msgs::VanjeeScanPtr scan(new vanjee_msgs::VanjeeScan);
    scan->packets.resize(400);

    vanjee_msgs::VanjeePacket *pkt;
    std::string *pbuf = NULL;
    int i = 0;
    bool l_f = false;
    bool l_s = true;
    int l_pcakblock = 0;
    bool l_x = false;
    unsigned short p_header;
    unsigned short p_ang;
    unsigned short p_angpre=18000;
    ros::Rate loop_(2000);

    // ROS_INFO("into  euqueuepacket %d",npackets);
    while (1)
    {
      //loop_.sleep();
      if (queue_.size() > 0)
      {
        {
          boost::unique_lock<boost::mutex> lock(mutex_);
          pbuf = queue_.front();
          queue_.pop();
        }

        const char *buf = pbuf->c_str();
        

        p_header = ((unsigned char)pbuf->at(0) * 256) + (unsigned char)pbuf->at(1);

        
        //ROS_INFO("queue_.push(packet)  %d",queue_.size());

        if (!setnpacketsflag && p_header != 0xFFAA)
        {
          npackets = setnpackets721((char *)buf);
          scan->packets.resize(npackets);
          setnpacketsflag = true;
          ROS_INFO("congig_yaml.start_ang = %f",congig_yaml.start_ang);
          // continue;
        }

        switch (p_header)
        {
        case 0xFFEE:
        {
          memcpy(&scan->packets[i].data[0], &buf[0], pbuf->length());
          p_ang = ((unsigned char)pbuf->at(522) * 256) + (unsigned char)pbuf->at(523);
          p_ang = p_ang % 36000;
          
        }
        break;

        case 0xFFDD:
        {
          memcpy(&scan->packets[i].data[0], &buf[0], pbuf->length());
          p_ang = ((unsigned char)pbuf->at(522) * 256) + (unsigned char)pbuf->at(523);
          p_ang = p_ang % 36000;
          // ROS_INFO("intto 0xFFDD");
        }
        break;

        case 0xFFAA:
        {
          short l_mainming = (unsigned char)pbuf->at(22)<<8 | (unsigned char)pbuf->at(23);
          switch(l_mainming)
          {
            case 0x0401:
            {
              std::string l_data = *pbuf;
              boost::shared_ptr<boost::thread> udpthread_ptr;
              udpthread_ptr.reset(new boost::thread(boost::bind(&InputSocket721::Write0401, this,l_data)));
            }
            break;

            // case 0x0402:
            // {
              
            // }
            // break;

            case 0x0514:
            {
              std::string l_data = *pbuf;
              boost::shared_ptr<boost::thread> udpthread_ptr2;
              udpthread_ptr2.reset(new boost::thread(boost::bind(&InputSocket721::WriVerAngtoCSV, this, l_data)));
            }
            break;

            default:
            {
              
            }
            break;
          }
          //break;

          

          continue;
        }
        break;

        } // switch

        //ROS_INFO("the header  %d  i = %d  p_ang = %d   %d,congig_yaml.start_ang = %d",queue_.size(),i ,p_ang,p_angpre,congig_yaml.start_ang);
        ////ROS_INFO("pang %d",p_ang);

        if (l_s)
        {
          //if (p_ang == (int)congig_yaml.start_ang)
          if ((p_ang >= (congig_yaml.start_ang)) && (p_ang < (congig_yaml.start_ang+300)))
          {
            // ROS_INFO("p_ang %d,%d",p_ang,(int)congig_yaml.start_ang);
            l_f = true;
            l_s = false;
          }
          else
          {
            continue;
          }
          
        }

        if (p_angpre != p_ang)// || (p_angpre-p_ang)==35700 || (p_angpre-p_ang)==35880) 
        {
          i++;
          // ROS_INFO("i = %d",i);
        }
        
        
        // ROS_INFO("p_ang %d,%d   %d",p_ang,p_angpre,p_angpre - p_ang);
        //i++;
        //if (i == (npackets) && l_f)
        if(p_angpre > p_ang && l_f)
        {
          // ROS_INFO("*******socketpublisher.publish(scan)****0xFFEE*****   i = %d  p_angpre = %d  p_ang = %d   %d",i,p_angpre,p_ang , scan->packets.size());
          i = 0;
          double time2 = ros::Time::now().toSec();
          scan->header.stamp = ros::Time(time2);
          scan->header.frame_id = congig_yaml.frame_id; // std::string("wlr_720");
          socketpublisher.publish(scan);
          //l_s = true;
          //l_f = false;
          //ros::Rate loop(50);
          // loop.sleep();
          
        }
        p_angpre = p_ang;

        delete pbuf;
      } // if

    } // while
    ROS_INFO("*******out  publish *********");
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP721 class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
  InputPCAP721::InputPCAP721(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                             bool read_once, bool read_fast, double repeat_delay)
      : Input(private_nh, 0,"","")
  {
    filename_ = filename;
    pcap_ = NULL;
    empty_pcap = true;
    npackets = 120;
    mangstr = 0;
    mangend = 360;

    // Open the PCAP dump file
    // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    ROS_INFO_STREAM("Opening PCAP file " << filename_);
    //fprintf(stderr, "rePCAP file:%s\n", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
    {
      ROS_FATAL("Error opening wjlidar socket dump file.");
      return;
    }

    std::stringstream filter;
    if (devip_str_ != "") // using specific IP?
    {
      filter << "src host " << devip_str_ << " && ";
    }
    filter << "udp dst port " << port;
    pcap_compile(pcap_, &pcap_packet_filter_, filter.str().c_str(), 1, PCAP_NETMASK_UNKNOWN);
  }

  /** destructor */
  InputPCAP721::~InputPCAP721(void)
  {
    pcap_close(pcap_);
  }


  void InputPCAP721::refreshPackets(int &packs, unsigned char *pbuf, int rpm)
  {
    if (packs != npackets)
    {
      if (pbuf[0] == 0xFF && pbuf[1] == 0xEE)
      {
        npackets = 72000 / rpm;
        packs = npackets;
        fprintf(stderr, "refreshPackets721:%d rpm:%d\n", npackets, rpm);
      }
    }
  }

  void InputPCAP721::resleepPackets(int rpm,double time) 
  {
    // if (npackets != 0)
    // {
    //   float l_ti = 1 / (60.0 / rpm - npackets / 2160.0) + 0.5;
    //   ros::Rate loop_rate(l_ti);
    //   loop_rate.sleep();
    // }

    double l_ti = 1.0 / (60.0 / rpm - time);
    ros::Rate loop_rate(l_ti);
    loop_rate.sleep();
  }

  void InputPCAP721::startpublisher()
  {
    boost::shared_ptr<boost::thread> udpthread_ptr1;
    udpthread_ptr1.reset(new boost::thread(boost::bind(&InputPCAP721::euqueuepacket, this)));

  }

  void InputPCAP721::euqueuepacket()
  {
    vanjee_msgs::VanjeeScanPtr scan(new vanjee_msgs::VanjeeScan);
    scan->packets.resize(200);
    static int firstpro = 0;
    double timenow;
    double timepre;
    
    ROS_INFO("setnpacketsflag %d",setnpacketsflag);
    //npackets = 100;
    while(1)
    {
      timenow = ros::Time::now().toSec();
      for (int i = 0; i < npackets; ++i)
      {
        while (true)
        {
          // keep reading until full packet received
          //ROS_INFO("start send scan 1");
          int rc = pcap_getpacket(&scan->packets[i]);
          unsigned char *pdata = static_cast<unsigned char *>(&scan->packets[i].data[0]);
          // ROS_INFO("%d ",rc);

         
          unsigned short l_ang = ((unsigned char)pdata[2] * 256) + (unsigned char)pdata[3]; // FFDD
          //ROS_INFO("%d, %f ,%f",l_ang,congig_yaml.start_ang,congig_yaml.end_ang);
          l_ang = l_ang % 36000;
          if (congig_yaml.start_ang < congig_yaml.end_ang)
          {
            if (l_ang < congig_yaml.start_ang || l_ang >= congig_yaml.end_ang)
            {
              i--;
            }
          }
          else
          {
            if (l_ang < congig_yaml.start_ang && l_ang >= congig_yaml.end_ang)
            {
              i--;
            }
            else
            {
            }
          }
          
          if (rc == 0)
          {
            if (!setnpacketsflag && pdata[1] == 0xEE)
            {
              npackets = setnpackets721((char *)pdata);
              scan->packets.resize(npackets);
              // ROS_INFO("start send scan %d *******  %d    %d", npackets,sizeof(pdata),strlen((char *)&pdata));
              setnpacketsflag = true;
            }
            break; // got a full packet?
          }

          if (rc < 0)
            continue;; // end of file reached?

          
          
        }
        //unsigned char *pdata = static_cast<unsigned char *>(&scan->packets[i].data[0]);

        // fprintf(stderr,"msop_input_->config_.npackets = %d \n",msop_input_->config_.npackets);
        //  publish packet from horizon angle 0
        
      }

      
      scan->header.stamp = ros::Time(ros::Time::now().toSec());
      scan->header.frame_id = congig_yaml.frame_id;
      socketpublisher.publish(scan);
      //ROS_INFO("char %f ",timepre - timenow);
      timepre = ros::Time::now().toSec();
      resleepPackets(congig_yaml.rmp,timepre - timenow);
      //ROS_INFO("publish");
    }
  }

}
