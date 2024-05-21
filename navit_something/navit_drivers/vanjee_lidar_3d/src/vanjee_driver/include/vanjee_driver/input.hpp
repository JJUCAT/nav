/*
 * This file is part of vanjeelidar driver.
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
 *
 *     Input -- base class used to access the data independently of
 *              its source
 *
 *     InputSocket720 -- derived class reads live data from the device
 *              via a UDP socket
 *
 *     InputPCAP720 -- derived class provides a similar interface from a
 *              PCAP dump
 */

#ifndef __VJLIDAR_INPUT_H_
#define __VJLIDAR_INPUT_H_

#include <unistd.h>
#include <stdio.h>
#include <pcap.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <vanjee_msgs/VanjeePacket.h>
#include <string>
#include <sstream>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/file.h>
#include <signal.h>
#include <sensor_msgs/TimeReference.h>
#include <boost/thread.hpp>
#include <yaml-cpp/yaml.h>
#include <dirent.h>
#include <stdio.h>
#include <ros/package.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vanjee_driver/yaml_reader.hpp>
#include <queue>
#include "boost/thread/condition.hpp"
#include "boost/thread/mutex.hpp"

// #include "common/time.h"
// #include "common_module_status/module_status.h"
// #include "common/file.h"

#define BUFF_SIZE 1600

namespace vanjee_driver
{
  static uint16_t MSOP_DATA_PORT_NUMBER = 2368;  // vanjeelidar default data port on PC
  static uint16_t DIFOP_DATA_PORT_NUMBER = 2369; // vanjeelidar default difop data port on PC
  /**
   *  从在线的网络数据或离线的网络抓包数据（pcap文件）中提取出lidar的原始数据，即packet数据包
   * @brief The Input class,
   *
   * @param private_nh  一个NodeHandled,用于通过节点传递参数
   * @param port
   * @returns 0 if successful,
   *          -1 if end of file
   *          >0 if incomplete packet (is this possible?)
   */
  class Input
  {
  public:
    Input(ros::NodeHandle private_nh, uint16_t port,std::string l_destIp,std::string l_localIP) : private_nh_(private_nh), port_(port),packet_rate_(3000)
    {
      if (port != 0)
      {
        //ROS_INFO("((((((((((<<<<<<<<<<<<<<<<<<<<<<<<<<<<<***>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>))))))))))))");
        sock_udp = socket(PF_INET, SOCK_DGRAM, 0);
        if (sock_udp == -1)
        {
          perror("socket"); // TODO: ROS_ERROR errno
          return;
        }

        sockaddr_in my_addr;                  // my address information
        memset(&my_addr, 0, sizeof(my_addr)); // initialize to zeros
        my_addr.sin_family = AF_INET;         // host byte order
        //my_addr.sin_addr.s_addr = inet_addr(get_local_ip("ens37").c_str());//htonl(INADDR_ANY); // automatically fill in my IP
        my_addr.sin_addr.s_addr =  htonl(INADDR_ANY);//inet_addr(l_localIP.c_str());//inet_addr(l_ip.c_str());
        my_addr.sin_port = htons(port);      // port in network byte order

        if (l_destIp != "")
        {
          int opt = 1;
          if (setsockopt(sock_udp, SOL_SOCKET, SO_REUSEADDR, (const void *)&opt, sizeof(opt)))
          {
            perror("setsockopt error!\n");
            return;
          }

          struct ip_mreq mreq; //组播结构体inet_addr(devip_str_.c_str());
          mreq.imr_multiaddr.s_addr = inet_addr(l_destIp.c_str());                  //组播组的ip地址
          //mreq.imr_interface.s_addr =  inet_addr(get_local_ip("ens37").c_str());//htonl(INADDR_ANY);
          mreq.imr_interface.s_addr = inet_addr(l_localIP.c_str());//htonl(INADDR_ANY);//inet_addr(l_ip.c_str());                          //加入的客户端主机的ip地址  INADDR_ANY为0.0.0.0,泛指本机,表示本机所有的ip.
          //mreq.imr_interface.s_addr = inet_addr(l_localIP.c_str());
          setsockopt(sock_udp, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)); //设置组播权限及选项
          ROS_INFO("ADD GROUP:[%s]   %s", l_destIp.c_str() , inet_ntoa(*(struct in_addr*)&mreq.imr_interface.s_addr));
        }

        if (bind(sock_udp, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1)
        {
          perror("bind"); // TODO: ROS_ERROR errno
          return;
        }
      }

      setnpacketsflag = false;
      pushImupacketflag =false;

      /**********about pcap************/
      empty_pcap = true;
      //pcap_ = NULL;
    }

    virtual ~Input()
    {
    }

    virtual void refreshPackets(int &packs, unsigned char *pbuf, int rpm) = 0;
    virtual void resleepPackets(int rpm,double time) = 0;
    virtual void euqueuepacket() = 0;
    virtual void startpublisher() = 0;

    int nrpm;
    int mangstr;
    int mangend;

    int sock_udp;

    /********about pcap********/
    std::string filename_;
    bool empty_pcap;
    ros::Rate packet_rate_;
    pcap_t *pcap_;
    bpf_program pcap_packet_filter_;
    char errbuf_[PCAP_ERRBUF_SIZE];
    int npackets;
    int imunpackets;
    /********about pcap********/

    std::queue<std::string *> queue_;
    std::queue<std::string *> queue_imu;
    mutable boost::mutex mutex_;
    mutable boost::mutex mutex_imu;
    bool setnpacketsflag;
    bool pushImupacketflag;

    struct congigyaml
    {
      std::string lidartype;
      std::string frame_id;
      float start_ang;
      float end_ang;
      bool read_fastflag;
      bool read_onceflag;
      float repeat_delay;
      std::string lidar_ipset;
      int lidar_portset;
      std::string vanjeepackettopic;
      int rmp;
      std::string pcappath;
      bool ReadVerAngFlag;
      bool multicastflag;
      std::string destIp;
      std::string local_ip;
      bool timemode;
      int pointSplitflag; // 1 spllit by angle  ;  2 split by Circle number
    };

    congigyaml congig_yaml;

    ros::Publisher socketpublisher;
    ros::Publisher sock_butpub;

  protected:
    ros::NodeHandle private_nh_;
    uint16_t port_;
    std::string devip_str_;

  public:
    void Read_configyaml(std::string path_fig, std::string part, int lidarNO)
    {
      YAML::Node config;
      config = YAML::LoadFile(path_fig);
      // std::cout << config["lidar"] << std::endl; //可以直接用下标访问

      YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
      congig_yaml.lidartype = common_config[lidarNO]["driver"]["lidartype"].as<std::string>();
      //ROS_INFO("lidartype : %s", congig_yaml.lidartype.c_str());

      congig_yaml.frame_id = common_config[lidarNO]["ros_topic"]["point_frame_id"].as<std::string>();
      //ROS_INFO("frame_id : %s", congig_yaml.frame_id.c_str());

      congig_yaml.start_ang = common_config[lidarNO]["driver"]["min_angle"].as<float>();
      //ROS_INFO("start_ang : %d", congig_yaml.start_ang);

      congig_yaml.end_ang = common_config[lidarNO]["driver"]["max_angle"].as<float>();
      //ROS_INFO("end_ang : %d", congig_yaml.end_ang);

      congig_yaml.read_fastflag = common_config[lidarNO]["driver"]["read_fast"].as<bool>();
      //ROS_INFO("read_fastflag : %d", congig_yaml.read_fastflag);

      congig_yaml.read_onceflag = common_config[lidarNO]["driver"]["read_once"].as<bool>();
      //ROS_INFO("read_onceflag : %d", congig_yaml.read_onceflag);

      congig_yaml.repeat_delay = common_config[lidarNO]["driver"]["repeat_delay"].as<float>();
      //ROS_INFO("repeat_delay : %f", congig_yaml.repeat_delay);

      congig_yaml.lidar_ipset = common_config[lidarNO]["proto"]["lidar_ip"].as<std::string>();
      //ROS_INFO("lidar_ipset : %s", congig_yaml.lidar_ipset.c_str());

      congig_yaml.local_ip = common_config[lidarNO]["proto"]["local_ip"].as<std::string>();

      congig_yaml.lidar_portset = common_config[lidarNO]["proto"]["dest_port"].as<int>();//local_ip
      //ROS_INFO("lidar_portset : %d", congig_yaml.lidar_portset);

      //congig_yaml.rmp = common_config[lidarNO]["driver"]["rmp"].as<int>();
      //ROS_INFO("rmp : %d", congig_yaml.rmp);

      congig_yaml.vanjeepackettopic = common_config[lidarNO]["ros_topic"]["ros_recv_packet_topic"].as<std::string>();
      //ROS_INFO("vanjeepackettopic : %s", congig_yaml.vanjeepackettopic.c_str());

      congig_yaml.pcappath = common_config[lidarNO]["driver"]["pcap"].as<std::string>();
      //ROS_INFO("pcappath : %s", congig_yaml.pcappath.c_str());

      congig_yaml.ReadVerAngFlag = true;

      congig_yaml.multicastflag = common_config[lidarNO]["driver"]["multicast"].as<bool>();
      congig_yaml.destIp = common_config[lidarNO]["proto"]["dest_ip"].as<std::string>();

      congig_yaml.timemode = common_config[lidarNO]["driver"]["time_mode"].as<bool>();

      congig_yaml.pointSplitflag = common_config[lidarNO]["driver"]["pointSplitflag"].as<int>();

      // if(congig_yaml.multicastflag)
      // {
      //   devip_str_ = congig_yaml.destIp;
      // }
      // else
      // {
      //   devip_str_ = common_config[lidarNO]["proto"]["lidar_ip"].as<std::string>();
      // }

      //**************************************************************************//
      if (congig_yaml.pcappath == std::string("nothing"))
      {
        boost::shared_ptr<boost::thread> udpthread_ptr;
        udpthread_ptr.reset(new boost::thread(boost::bind(&Input::UDP_recvThread, this, congig_yaml.lidar_portset)));
      }
      else
      {
        if (congig_yaml.read_onceflag)
          ROS_INFO("Read input file only once.");
        if (congig_yaml.read_fastflag)
          ROS_INFO("Read input file as quickly as possible.");
        if (congig_yaml.repeat_delay > 0.0)
          ROS_INFO("Delay %.3f seconds before repeating input file.", congig_yaml.repeat_delay);

        //boost::shared_ptr<boost::thread> udpthread_ptr;
        //udpthread_ptr.reset(new boost::thread(boost::bind(&Input::pcap_recvThread, this, congig_yaml.pcappath)));

      }
    }

    void UDP_recvThread(int port)
    {
      struct pollfd fds[1];
      fds[0].fd = sock_udp;
      fds[0].events = POLLIN;
      static const int POLL_TIMEOUT = 1000;

      char rbuf[1500] = {0};
      sockaddr_in sender_address;
      socklen_t sender_address_len = sizeof(sender_address);
      int l_titt = 0;
      while (1)
      {
        int time_out_number = 0;
        // MODULE_STATUS_SET(0,false);
        do
        {
          // if(time_out_number > 30) MODULE_STATUS_SET(0,true);
          int retval = poll(fds, 1, POLL_TIMEOUT);
          if (retval < 0) // poll() error?
          {
            if (errno != EINTR)
              ROS_ERROR("poll() error: %s", strerror(errno));
            // MODULE_STATUS_SET(0,true);
            continue;
          }
          if (retval == 0) // poll() timeout?
          {
            time_out_number ++;
            //ROS_WARN_THROTTLE(2, "wjlidar poll() timeout");
            continue;
          }
          if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL)) // device error?
          {
            ROS_ERROR("poll() reports wjlidar error");
            // MODULE_STATUS_SET(0,true);
            continue;
          }
        } while ((fds[0].revents & POLLIN) == 0);
        // MODULE_STATUS_SET(0,false);
        ssize_t nbytes = recvfrom(sock_udp, rbuf, sizeof(rbuf), 0, (sockaddr *)&sender_address, &sender_address_len);
        //ROS_INFO("%d,%d,%d,%d,%d,%d,len = %d  ",rbuf[0],rbuf[1],rbuf[22],rbuf[23],rbuf[2],rbuf[3],nbytes);
        {
          boost::unique_lock<boost::mutex> lock(mutex_);
          std::string *packet = new std::string(rbuf, nbytes);
          queue_.push(packet);

        }

        {
          if ((unsigned char)rbuf[1] != 0xAA && pushImupacketflag == true)
          {
            boost::unique_lock<boost::mutex> lock(mutex_imu);

            l_titt++;
            //ROS_INFO("recvfrom  %x   %x   %d",(unsigned char)rbuf[0],(unsigned char)rbuf[1],l_titt);
            if (l_titt == imunpackets)
            {
              std::string *packet = new std::string(rbuf, nbytes);
              queue_imu.push(packet);
              //ROS_INFO("recvfrom  %f",ros::Time::now().toSec());
              l_titt = 0;
            }
          }
        }


          //

      }
    }

    int pcap_getpacket(vanjee_msgs::VanjeePacket* pkt)
    {
      struct pcap_pkthdr *header;
      const u_char *pkt_data;
      // while (flag == 1)
      while (true)
      {
        int res;
        if ((res = pcap_next_ex(pcap_, &header, &pkt_data)) >= 0)
        {
          int ress = pcap_offline_filter(&pcap_packet_filter_, header, pkt_data);
          // fprintf(stderr,"ress:%d\n",ress);
          // Skip packets not for the correct port and from the
          // selected IP address.
          if (!congig_yaml.lidar_ipset.empty() && (0 == ress))
            continue;

          // Keep the reader from blowing through the file.
          if (congig_yaml.read_fastflag == false) //
            packet_rate_.sleep();

          // judge scan data
          if (pkt_data[42] == 0xFF && (pkt_data[43] == 0xEE || pkt_data[43] == 0xDD) && header->caplen == header->len)
          {
            int pack_len = header->caplen;
            memcpy(&pkt->data[0], pkt_data + 42, pack_len);

          }
          else
          {
            fprintf(stderr, "error pcap len%d\n", header->caplen);
            continue;
          }

          //pkt->stamp = ros::Time::now(); // time_offset not considered here, as no
                                         // synchronization required
          empty_pcap = false;
          return 0; // success
        }

        if (empty_pcap) // no data in file?
        {
          fprintf(stderr, "empty_pcap\n");
          ROS_WARN("Error %d reading wjlidar packet: %s", res, pcap_geterr(pcap_));
          sleep(5);
          return -1;
        }

        if (congig_yaml.read_onceflag)
        {
          fprintf(stderr, "read_once\n");
          ROS_INFO("end of file reached -- done reading.");
          return -1;
        }

        if (congig_yaml.repeat_delay > 0.0)
        {
          ROS_INFO("end of file reached -- delaying %.3f seconds.", congig_yaml.repeat_delay);
          usleep(rint(congig_yaml.repeat_delay * 1000000.0));
        }

        ROS_DEBUG("replaying wjlidar dump file");

        // I can't figure out how to rewind the file, because it
        // starts with some kind of header.  So, close the file
        // and reopen it with pcap.
        pcap_close(pcap_);
        pcap_ = pcap_open_offline(filename_.c_str(), errbuf_);
        empty_pcap = true; // maybe the file disappeared?
      }                // loop back and try again
    }


    int setpacketnumforlidar(int l_rmp, int l_packblock, int starang, int endang)
    {
      int npacketnum = 0;
      if (congig_yaml.lidartype == "wlr720")
      {
        float angofpack = l_packblock * l_rmp / 3000.0;

        if (starang < endang)
        {
          npacketnum = (endang - starang) / angofpack;
        }
        else if (starang > endang)
        {
          npacketnum = (360 - endang + starang) / angofpack;
        }
      }
      else if (congig_yaml.lidartype == "wlr721")
      {
      }
      return npacketnum;
    }

    int setnpackets720(char *pcaket)
    {
      unsigned short p_header;
      int l_npackets;
      float l_ibe;
      float l_ien;
      float l_val;// = congig_yaml.rmp / 3000.0;//[600r,0.2],[300r,0.1]
      //ROS_INFO("*************************************congig_yaml.start_ang : %f  %f",congig_yaml.start_ang,congig_yaml.end_ang);
      p_header = ((unsigned char)pcaket[0] * 256) + (unsigned char)pcaket[1];
      switch (p_header)
      {
      case 0xFFDD:
      {
        uint8_t l_blocknum = (unsigned char)pcaket[4];
        int l_len = ((unsigned char)pcaket[2] * 4 + 2) * ((unsigned char)pcaket[4]* (unsigned char)pcaket[3]) + 65;
        congig_yaml.rmp =  ((unsigned char)pcaket[l_len - 21] * 256 + (unsigned char)pcaket[l_len - 22]);
        ROS_INFO("congig_yaml.rmp = %d",congig_yaml.rmp);
        l_val = congig_yaml.rmp / 3000.0;

        l_val = l_val * l_blocknum;
        l_ibe = ((int)(congig_yaml.start_ang / l_val + 0.1)) * l_val;
        l_ien = ((int)(congig_yaml.end_ang / l_val + 0.5)) * l_val;
        //ROS_INFO("^^^^^^***^^^^^%f    %f   %f   %f    %f  %f",congig_yaml.rmp / 3000.0,congig_yaml.rmp / 3000.0 * l_blocknum, (congig_yaml.start_ang / l_val) , (congig_yaml.end_ang / l_val + 0.5) ,l_ibe,l_ien);

        int l_fen;
        if(l_ibe <= l_ien)
        {
          l_fen = l_ien - l_ibe;
        }
        else
        {
          l_fen = 360 - l_ibe + l_ien;
        }
        congig_yaml.start_ang = l_ibe * 100;
        congig_yaml.end_ang = l_ien * 100;

        //ROS_INFO("congig_yaml.start_ang : %f  %f",congig_yaml.start_ang,congig_yaml.end_ang);

        // l_npackets = (1080000 / congig_yaml.rmp / l_blocknum) * (congig_yaml.end_ang - congig_yaml.start_ang) / 36000.0;
        l_npackets = (1080000 / congig_yaml.rmp / l_blocknum) *  l_fen / 360.0 + 0.1;
        //ROS_INFO("l_blocknum %d   congig_yaml.rmp %d",l_blocknum,congig_yaml.rmp);
      }
      break;

      case 0xFFEE:
        l_npackets = 72000 / 600;
      break;
      }

      ROS_INFO("*After calibration,Starting angle is %.1f , End angle is %.1f , npackets is %d*", congig_yaml.start_ang / 100 , congig_yaml.end_ang / 100, l_npackets);
      return l_npackets;
    }

    int setnpackets721(char *pcaket)
    {
      unsigned short p_header;
      int l_npackets;
      float l_ibe;
      float l_ien;
      float l_val;
      p_header = ((unsigned char)pcaket[0] * 256) + (unsigned char)pcaket[1];
      switch (p_header)
      {
      case 0xFFEE:
      {
        int l_rmp = ((unsigned char)pcaket[1310] * 256 + (unsigned char)pcaket[1311]);
        congig_yaml.rmp = ((int)((l_rmp + 10) / 300.0 + 0.001)) * 300;
        l_val = congig_yaml.rmp / 1000.0 * 5; // zhuansu / 1000.0 * mei bao de shu ju kuai ge shu
        l_ibe = ((int)(congig_yaml.start_ang / l_val + 0.1)) * l_val;
        l_ien = ((int)(congig_yaml.end_ang / l_val + 0.5)) * l_val;

        int l_fen;
        if (l_ibe <= l_ien)
        {
          l_fen = l_ien - l_ibe;
        }
        else
        {
          l_fen = 360 - l_ibe + l_ien;
        }
        congig_yaml.start_ang = l_ibe * 100;
        congig_yaml.end_ang = l_ien * 100;

        l_npackets = 72000 / congig_yaml.rmp * l_fen / 360.0 + 0.1;
        ROS_INFO("congig_yaml.rmp = %d", congig_yaml.rmp);

        // ROS_INFO("l_blocknum %d   congig_yaml.rmp %d",l_blocknum,congig_yaml.rmp);
      }
      break;

      case 0xFFDD:
      {
        int l_rmp = ((unsigned char)pcaket[1050] * 256 + (unsigned char)pcaket[1051]);
        congig_yaml.rmp = ((int)((l_rmp + 10) / 300.0 + 0.001)) * 300;
        l_val = congig_yaml.rmp / 1000.0 * 2; // zhuansu / 1000.0 * mei bao de shu ju kuai ge shu
        l_ibe = ((int)(congig_yaml.start_ang / l_val + 0.1)) * l_val;
        l_ien = ((int)(congig_yaml.end_ang / l_val + 0.5)) * l_val;

        int l_fen;
        if (l_ibe <= l_ien)
        {
          l_fen = l_ien - l_ibe;
        }
        else
        {
          l_fen = 360 - l_ibe + l_ien;
        }
        congig_yaml.start_ang = l_ibe * 100;
        congig_yaml.end_ang = l_ien * 100;

        l_npackets = 180000 / congig_yaml.rmp * l_fen / 360.0 + 0.1;
        ROS_INFO("congig_yaml.rmp = %d", congig_yaml.rmp);
      }
      break;

      default:
        break;
      }
      ROS_INFO("*After calibration,Starting angle is %.1f , End angle is %.1f , npackets is %d*", congig_yaml.start_ang / 100 , congig_yaml.end_ang / 100, l_npackets);
      return l_npackets;
    }

  };//calss name


}

#endif //
