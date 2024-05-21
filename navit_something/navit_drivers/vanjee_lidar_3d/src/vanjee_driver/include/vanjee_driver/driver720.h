/*
 * This file is part of vanjeelidar_720 driver.
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

#ifndef __VJLIDAR_INPUT_720_
#define __VJLIDAR_INPUT_720_

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <ros/ros.h>
#include <vanjee_msgs/VanjeePacket.h>
#include <vanjee_msgs/VanjeeScan.h>
#include "vanjee_driver/input.hpp"
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

#define BUFF_SIZE 1600

namespace vanjee_driver
{
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
/** @brief Live vanjeelidar input from socket. */
class InputSocket720 : public Input
{
public:
  InputSocket720(ros::NodeHandle private_nh, uint16_t port,std::string con_path,int con_part,std::string l_destIp,std::string l_localIp);

  virtual ~InputSocket720();

  int requestVerAngleResolution(int &sockfd);
  int requestSN(int &sockfd);
  int requestDevBaseParam(int &sockfd);
  int requestIMULINESPEND(int &sockfd);
  int requestIMULINEADD(int &sockfd);
  
  int requestBaseParam(std::string config_path,int part_i);
  int ReadBaseParam(int &sock);
  int ReadBootton(int &sock);
  int ReadVerAngleResolution(int &sock);
  void setcalibration(std::string l_config,int part_i,std::string sn);

  virtual void refreshPackets(int &packs,unsigned char *pbuf,int rpm);
  virtual void resleepPackets(int rpm,double time); 
  virtual void euqueuepacket();
  virtual void startpublisher();
  virtual void WriVerAngtoCSV(std::string l_data);

  bool Write0401Flag;
  bool Write0612Flag;
  bool Write0614Flag;
  void Write0401(std::string l_data);
  void Write0612(std::string l_data);
  void Write0614(std::string l_data);
  void writeImuParams(std::string config_path,int part_i);

  void ReadIMUTopublish(char *pcaket,int i);
  int UpImuHzPacket(char *pcaket);
  void euqueueIMUpacket();
  mutable boost::mutex mutex_bot;

  std::string config_path;
  int part_i;
  double imu_kb[6][2];
  
  int mangstr;
  int mangend;
  float m_Buttont;
  bool imuteflag;
  std::string m_VerCSVfile;
private:
  int sockfd_;
  char m_SN[16];
  int gotRequest;

  boost::mutex readbaseparm_mtu;
  boost::condition_variable var_rebaseparm_conmtu;
};

/** @brief vanjeelidar input from PCAP dump file.
   *
   * Dump files can be grabbed by libpcap
   */
class InputPCAP720 : public Input
{
public:
  InputPCAP720(ros::NodeHandle private_nh, uint16_t port = MSOP_DATA_PORT_NUMBER, double packet_rate = 0.0,
            std::string filename = "", bool read_once = false, bool read_fast = false, double repeat_delay = 0.0);

  virtual ~InputPCAP720();

  //virtual void refreshPackets(int &packs){}
  virtual void refreshPackets(int &packs,unsigned char *pbuf,int rpm);
  virtual void resleepPackets(int rpm,double time);
  virtual void euqueuepacket();
  virtual void startpublisher();
  
  int mangstr;
  int mangend;
  float m_Buttont;
  //int nrpm;
  
};
}

#endif  // __VJLIDAR_INPUT_720_
