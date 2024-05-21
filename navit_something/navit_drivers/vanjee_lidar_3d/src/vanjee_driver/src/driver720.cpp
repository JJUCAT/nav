/*
 * This file is part of wjlidar_720 driver.
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

#include "vanjee_driver/driver720.h"
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
  // InputSocket720 class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param con_path path to config.yaml
   *  @param con_part part of config.yaml
   */
  InputSocket720::InputSocket720(ros::NodeHandle private_nh, uint16_t port, std::string con_path, int con_part, std::string l_destIp, std::string l_localIp) : Input(private_nh, port, l_destIp, l_localIp)
  {
    sockfd_ = -1;
    npackets = 120;
    m_Buttont = -10000;
    mangstr = 0;
    mangend = 360;
    imuteflag = true;
    config_path = con_path; //
    part_i = con_part;
    memset(imu_kb, 0, sizeof(imu_kb));

    Write0401Flag = false;
    Write0612Flag = false;
    Write0614Flag = false;

    boost::unique_lock<boost::mutex> guard(readbaseparm_mtu);

    congig_yaml.ReadVerAngFlag = true;
    requestBaseParam(con_path, con_part);

    boost::shared_ptr<boost::thread> ReadBaseparamHandle;
    ReadBaseparamHandle.reset(new boost::thread(boost::bind(&InputSocket720::ReadBaseParam, this, sock_udp)));

    boost::shared_ptr<boost::thread> ReadBothreadHandle;
    ReadBothreadHandle.reset(new boost::thread(boost::bind(&InputSocket720::ReadBootton, this, sock_udp)));

    boost::shared_ptr<boost::thread> ReadVerAngHandle;
    ReadVerAngHandle.reset(new boost::thread(boost::bind(&InputSocket720::ReadVerAngleResolution, this, sock_udp)));
  }

  /** @brief destructor */
  InputSocket720::~InputSocket720(void)
  {
    (void)close(sockfd_);
  }

  int InputSocket720::ReadBaseParam(int &sock)
  {
    ros::Rate loop_rate1(10); // 10hz - 100ms
    loop_rate1.sleep();
    for (int i = 0; i < 3; i++)
    {
      // request sn
      // requestDevBaseParam(sock);
      // loop_rate1.sleep();

      // request sn
      requestSN(sock);
      loop_rate1.sleep();

      // request vertical angle resolution
      requestVerAngleResolution(sock);
      loop_rate1.sleep();

      requestIMULINESPEND(sock);
      loop_rate1.sleep();

      requestIMULINEADD(sock);
      loop_rate1.sleep();
    }
    return 0;
  }

  /** @brief recv  Request thread. */
  int InputSocket720::requestBaseParam(std::string config_path, int part_i)
  {
    /****************************************************************************************/
    YAML::Node config;
    config = YAML::LoadFile(config_path);
    // std::cout << config["lidar"] << std::endl; //可以直接用下标访问
    YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
    int l_port = common_config[part_i]["proto"]["dest_port"].as<int>();
    bool l_milticast = common_config[part_i]["driver"]["multicast"].as<bool>();

    if (l_milticast)
    {
      devip_str_ = common_config[part_i]["proto"]["dest_ip"].as<std::string>();
    }
    else
    {
      devip_str_ = common_config[part_i]["proto"]["lidar_ip"].as<std::string>();
    }
    devip_str_ = common_config[part_i]["proto"]["lidar_ip"].as<std::string>();

    ROS_INFO("enpoint: [%s : 3333]", devip_str_.c_str());

    std::string l_csvpath = common_config[part_i]["driver"]["calibration"].as<std::string>();
    l_csvpath = ros::package::getPath("vanjee_lidar") + "/data/Vanjee_lidar_16/" + l_csvpath;
    m_VerCSVfile = l_csvpath;

    return 0;
  }

  void InputSocket720::setcalibration(std::string l_config, int part_i, std::string sn)
  {

    YAML::Node config;
    config = YAML::LoadFile(l_config);

    YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
    std::string l_csvpath = "Vanjee16-carside" + sn + ".csv";
    // ROS_INFO("CSV : [%s]",l_csvpath.c_str());
    common_config[part_i]["driver"]["calibration"] = l_csvpath;

    std::fstream fout(l_config.c_str(), std::ios::out);
    fout << config;
    fout.close();
  }

  int InputSocket720::ReadVerAngleResolution(int &sock)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    unsigned char l_cDebug_QueryParamUFrameBuf[34] = {0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint8_t check = 0x00;
    for (int i = 2; i < sizeof(l_cDebug_QueryParamUFrameBuf) - 4; i++)
      check ^= l_cDebug_QueryParamUFrameBuf[i];
    l_cDebug_QueryParamUFrameBuf[31] = check; // check_bit
    l_cDebug_QueryParamUFrameBuf[32] = 0xEE;  // frame_tail
    l_cDebug_QueryParamUFrameBuf[33] = 0xEE;  // frame_tail

    while (congig_yaml.ReadVerAngFlag)
    {
      if (sendto(sock, l_cDebug_QueryParamUFrameBuf, sizeof(l_cDebug_QueryParamUFrameBuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
      {
        perror("sendto request vertical angle resolution");
        close(sock);
        return -1;
      }

      sleep(3);
    }

    ROS_INFO("out over read thread");
    return 0;
  }

  int InputSocket720::ReadBootton(int &sock)
  {

    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    unsigned char g_cDebug_QueryParamUFrameBuf[34] = {0xFF, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
        close(sock);
        return -1;
      }

      sleep(5);
    }

    ROS_INFO("out ReadBootton");
    return 0;
  }

  void InputSocket720::refreshPackets(int &packs, unsigned char *pbuf, int rpm)
  {
    if (packs != npackets)
    {
      if (pbuf[0] == 0xFF && pbuf[1] == 0xDD)
      {
        npackets = rpm / 30 * (pbuf[4]);
        npackets = 36000 / npackets;
        packs = npackets;
        fprintf(stderr, "refreshPackets1:%d channel:%d npacket:%d rpm:%d\n", npackets, pbuf[3], pbuf[4], rpm);
      }
      else if (pbuf[0] == 0xFF && pbuf[1] == 0xEE)
      {
        npackets = 72000 / rpm;
        packs = npackets;
        fprintf(stderr, "refreshPackets2:%d rpm:%d\n", npackets, rpm);
      }

      packs = npackets;
      fprintf(stderr, "refreshPackets:%d\n", npackets);
    }
  }

  void InputSocket720::resleepPackets(int rpm, double time)
  {
  }

  /** @brief send a Request to get device base param. */
  int InputSocket720::requestDevBaseParam(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x01};
    uint8_t check = 0x00; // 用于保存异或结果
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
  int InputSocket720::requestSN(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x01};
    uint8_t check = 0x00; // 用于保存异或结果
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

  /** @brief send a Request to get  vertical angle resolution. */
  int InputSocket720::requestVerAngleResolution(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x14};
    uint8_t check = 0x00; // 用于保存异或结果
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

  int InputSocket720::requestIMULINESPEND(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x12};
    uint8_t check = 0x00; // 用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail
    if (sendto(sockfd, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
    {
      perror("sendto request IMU L resolution");
      close(sockfd);
      return -1;
    }
    return 0;
  }

  int InputSocket720::requestIMULINEADD(int &sockfd)
  {
    sockaddr_in remote_addr;                                     // remote address information
    memset(&remote_addr, 0, sizeof(remote_addr));                // initialize to zeros
    remote_addr.sin_family = AF_INET;                            // host byte order
    remote_addr.sin_port = htons(3333);                          // port in network byte order
    remote_addr.sin_addr.s_addr = inet_addr(devip_str_.c_str()); // automatically fill in remote IP
    uint8_t sbuf[34] =                                           //{0};
        {0xff, 0xAA, 0x00, 0x1E, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x0B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x14};
    uint8_t check = 0x00; // 用于保存异或结果
    int i;
    for (i = 2; i < sizeof(sbuf) - 4; i++)
      check ^= sbuf[i];
    sbuf[31] = check; // check_bit
    sbuf[32] = 0xEE;  // frame_tail
    sbuf[33] = 0xEE;  // frame_tail
    if (sendto(sockfd, sbuf, sizeof(sbuf), 0, (sockaddr *)&remote_addr, sizeof(remote_addr)) == -1)
    {
      perror("sendto request IMU L resolution");
      close(sockfd);
      return -1;
    }
    return 0;
  }

  void InputSocket720::startpublisher()
  {
    boost::shared_ptr<boost::thread> udpthread_ptr1;
    udpthread_ptr1.reset(new boost::thread(boost::bind(&InputSocket720::euqueuepacket, this)));

    boost::shared_ptr<boost::thread> udpthread_ptr2;
    udpthread_ptr2.reset(new boost::thread(boost::bind(&InputSocket720::euqueueIMUpacket, this)));
  }

  void InputSocket720::writeImuParams(std::string config_path, int part_i)
  {
    YAML::Node config;
    config = YAML::LoadFile(config_path);
    // std::cout << config["lidar"] << std::endl; //可以直接用下标访问
    YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
    std::string l_csvpath = common_config[part_i]["driver"]["calibration"].as<std::string>();
    l_csvpath = ros::package::getPath("vanjee_lidar") + "/data/Vanjee_lidar_16/" + l_csvpath;
    m_VerCSVfile = l_csvpath;

    std::string l_imulinepath = l_csvpath;
    std::string l_linpath = common_config[part_i]["driver"]["calibutfipath"].as<std::string>();
    l_imulinepath = ros::package::getPath("vanjee_lidar") + "/cfg/" + l_linpath;
    ROS_INFO("[imu paramter* ]%s", l_imulinepath.c_str());

    ofstream file;
    file.open(l_imulinepath.c_str());

    if (!file)
    {
      ROS_WARN_STREAM("[ imu ]" << l_imulinepath << " does not exist");
      // fprintf(stderr, "%s\n", anglePath.c_str());
    }
    else
    {
      file << fixed << setprecision(6) << "lin,X," << imu_kb[0][0] << "," << imu_kb[0][1] << std::endl;
      file << fixed << setprecision(6) << "lin,Y," << imu_kb[1][0] << "," << imu_kb[1][1] << std::endl;
      file << fixed << setprecision(6) << "lin,Z," << imu_kb[2][0] << "," << imu_kb[2][1] << std::endl;
      file << fixed << setprecision(6) << "acc,X," << imu_kb[3][0] << "," << imu_kb[3][1] << std::endl;
      file << fixed << setprecision(6) << "acc,Y," << imu_kb[4][0] << "," << imu_kb[4][1] << std::endl;
      file << fixed << setprecision(6) << "acc,Z," << imu_kb[5][0] << "," << imu_kb[5][1] << std::endl;
    }
    file.close();
  }

  void InputSocket720::Write0401(std::string l_data)
  {

    if (!Write0401Flag)
    {
      const char *pbuf = l_data.c_str();
      Write0401Flag = true;
      if (l_data.length() != 150)
      {
        ROS_ERROR("Wrong length of SN");
      }
      char sn[20] = {0};
      memcpy(sn, &pbuf[26], 16);
      sn[16] = 0;
      ROS_INFO("[SN] %s", sn);
    }
  }

  void InputSocket720::Write0612(std::string l_data)
  {
    if (l_data.length() != 54)
    {
      ROS_ERROR("Wrong length of imu lin information");
      return;
    }

    const char *pbuf = l_data.c_str();
    // memcpy(pbuf[0],l_data,l_data.length());
    // ROS_INFO("0x06,0x12");
    if (!Write0612Flag)
    {
      Write0612Flag = true;
      gotRequest++;

      float l_k;
      float l_b;

      imuteflag = pbuf[25];
      // ROS_INFO("flagg %d",pbuf[25]);
      {
        memcpy(&l_k, &pbuf[26], 4);
        memcpy(&l_b, &pbuf[30], 4);
        imu_kb[0][0] = l_k;
        imu_kb[0][1] = l_b;

        memcpy(&l_k, &pbuf[34], 4);
        memcpy(&l_b, &pbuf[38], 4);
        imu_kb[1][0] = l_k;
        imu_kb[1][1] = l_b;

        memcpy(&l_k, &pbuf[42], 4);
        memcpy(&l_b, &pbuf[46], 4);
        imu_kb[2][0] = l_k;
        imu_kb[2][1] = l_b;
      }

      // if (imu_kb[0][0] != 0 && imu_kb[1][0] != 0 && imu_kb[2][0] != 0 && imu_kb[3][0] != 0 && imu_kb[4][0] != 0 && imu_kb[5][0] != 0)
      // {
      //   writeImuParams(config_path, part_i);
      // }
    }
  }

  void InputSocket720::Write0614(std::string l_data)
  {
    if (l_data.length() != 54)
    {
      ROS_ERROR("Wrong length of imu add information");
      return;
    }
    const char *pbuf = l_data.c_str();
    // memcpy(pbuf[0],l_data,l_data.length());
    // ROS_INFO("0x06,0x14");
    if (!Write0614Flag)
    {
      Write0614Flag = true;
      gotRequest++;

      float l_k;
      float l_b;

      {
        memcpy(&l_k, &pbuf[26], 4);
        memcpy(&l_b, &pbuf[30], 4);
        imu_kb[3][0] = l_k;
        imu_kb[3][1] = l_b;

        memcpy(&l_k, &pbuf[34], 4);
        memcpy(&l_b, &pbuf[38], 4);
        imu_kb[4][0] = l_k;
        imu_kb[4][1] = l_b;

        memcpy(&l_k, &pbuf[42], 4);
        memcpy(&l_b, &pbuf[46], 4);
        imu_kb[5][0] = l_k;
        imu_kb[5][1] = l_b;
      }

      // if (imu_kb[0][0] != 0 && imu_kb[1][0] != 0 && imu_kb[2][0] != 0 && imu_kb[3][0] != 0 && imu_kb[4][0] != 0 && imu_kb[5][0] != 0)
      // {
      //   writeImuParams(config_path, part_i);
      // }
    }
  }

  void InputSocket720::WriVerAngtoCSV(std::string l_data)
  {
    // ROS_INFO("chuizhijiaodu  %x,%x length %d",l_data.at(0),l_data.at(1),l_data.length());
    if (l_data.length() != 95)
    {
      ROS_ERROR("Wrong length of vertical angle information");
    }
    else if (congig_yaml.ReadVerAngFlag)
    {
      float recvVerAngle[16] = {0};
      float verAngle[19] = {0};
      float l_stdver;

      for (int j = 0; j < 16; j++)
      {
        l_stdver = 14.0 - j * 2.0;
        recvVerAngle[j] = ((unsigned char)(l_data.at(30 + j * 4)) << 24 | ((unsigned char)l_data.at(29 + j * 4)) << 16 | ((unsigned char)l_data.at(28 + j * 4)) << 8 | ((unsigned char)l_data.at(27 + j * 4))) * 1.0 / 1000;
        // ROS_INFO("%f :[%d %d %d %d]",recvVerAngle[j],l_data.at(30 + j * 4),l_data.at(29 + j * 4),l_data.at(28 + j * 4),l_data.at(27 + j * 4));
        if ((recvVerAngle[j] > (l_stdver + 0.5)) || (recvVerAngle[j] < (l_stdver - 0.5)))
        {
          ROS_WARN("We found obviously wrong vertical angle information.the Ver Value of the line %d is %f", j + 1, recvVerAngle[j]);
          return;
        }
      }

      verAngle[0] = verAngle[5] = verAngle[9] = verAngle[14] = recvVerAngle[7];
      verAngle[1] = recvVerAngle[0];
      verAngle[2] = recvVerAngle[1];
      verAngle[3] = recvVerAngle[2];
      verAngle[4] = recvVerAngle[3];

      verAngle[6] = recvVerAngle[4];
      verAngle[7] = recvVerAngle[5];
      verAngle[8] = recvVerAngle[6];

      verAngle[10] = recvVerAngle[8];
      verAngle[11] = recvVerAngle[9];
      verAngle[12] = recvVerAngle[10];
      verAngle[13] = recvVerAngle[11];

      verAngle[15] = recvVerAngle[12];
      verAngle[16] = recvVerAngle[13];
      verAngle[17] = recvVerAngle[14];
      verAngle[18] = recvVerAngle[15];

      std::string anglePath = m_VerCSVfile;
      ofstream file;

      file.open(anglePath.c_str());
      if (!file)
      {
        ROS_WARN_STREAM("[cloud][rawdata] 19" << anglePath << " does not exist");
        fprintf(stderr, "%s\n", anglePath.c_str());
      }
      else
      {
        for (int j = 0; j < 19; j++)
          file << fixed << setprecision(3) << verAngle[j] << ",0" << std::endl;
      }
      file.close();

      congig_yaml.ReadVerAngFlag = false;
      // ROS_WARN("Found new vertical angle information, you can choose to restart.");
    } // end if
  }

  void InputSocket720::euqueuepacket()
  {
    vanjee_msgs::VanjeeScanPtr scan(new vanjee_msgs::VanjeeScan);
    scan->packets.resize(300);

    vanjee_msgs::VanjeePacket *pkt;
    std::string *pbuf = NULL;
    int i = 0;
    bool l_f = false;
    bool l_s = true;
    int l_pcakblock = 0;
    bool l_x = false;
    unsigned short p_header;
    unsigned short p_ang;
    unsigned short p_angpre = -1;

    unsigned int pkt_id = 0 ,last_pkt_id = 0;
    unsigned int pkt_id_last = 0;
    unsigned int circle_id = 0,circle_id_0 = 0;
    unsigned int circle_id_debug = 0;
    ros::Rate loop_(2000);
    int lllllllll = 0;

    ROS_INFO("into  euqueuepacket %d", npackets);
    while (1)
    {
      loop_.sleep();
      if (queue_.size() > 0)
      {
        {
          boost::unique_lock<boost::mutex> lock(mutex_);
          pbuf = queue_.front();
          queue_.pop();
        }

        const char *buf = pbuf->c_str();
        memcpy(&scan->packets[i].data[0], &buf[0], pbuf->length());
        // int l_iii = (scan->packets[i].data[1249] * 256) + scan->packets[i].data[1250];
        // printf("------------> %d   %d\n",l_iii,i);

        p_header = ((unsigned char)pbuf->at(0) * 256) + (unsigned char)pbuf->at(1);
        // ROS_INFO("the header  %x",p_header)
        if (!setnpacketsflag && p_header != 0xFFAA)
        {
          npackets = setnpackets720((char *)buf);
          imunpackets = UpImuHzPacket((char *)buf);
          scan->packets.resize(npackets+10);
          setnpacketsflag = true;

          // ROS_INFO("npackets is %d",npackets);
          //  continue;
        }

        switch (p_header)
        {
        case 0xFFDD:
        {
          p_ang = ((unsigned char)pbuf->at(6) * 256) + (unsigned char)pbuf->at(5);
          p_ang = p_ang % 36000;
          
          circle_id = ((unsigned char)pbuf->at(pbuf->length()-60+2) * 256) + (unsigned char)pbuf->at(pbuf->length()-60+3);
          //ROS_INFO("circle_id = %d , i  = %d" , circle_id , i);
          // if(i == 0)
          //   circle_id_0 = circle_id;

          unsigned int interval = (((unsigned char)pbuf->at(1249) * 256) + (unsigned char)pbuf->at(1250) - pkt_id_last + 65536)%65536;
          if(interval != 1)
          {
            //ROS_INFO("loss package... pkt_id_interval=%d,pkt_id = %d,circle_id_debug = %d",interval,pkt_id_last,circle_id_debug);
          }
          pkt_id_last = ((unsigned char)pbuf->at(1249) * 256) + (unsigned char)pbuf->at(1250);
          circle_id_debug = circle_id;

          // if(i == 0)
          // {
          //   pkt_id = ((unsigned char)pbuf->at(1249) * 256) + (unsigned char)pbuf->at(1250);
          //   if((pkt_id + 65536 - last_pkt_id)%65536 != 100)
          //   {
          //     ROS_INFO("pkt_id interval:%d",(pkt_id + 65536 - last_pkt_id)%65536);
          //   }
          //   //ROS_INFO("pkt_id interval:%d",(pkt_id + 65536 - last_pkt_id)%65536);
          // }
          // else{
          //   last_pkt_id = pkt_id;
          // }
        }
        break;

        case 0xFFEE:
        {
          p_ang = ((unsigned char)pbuf->at(3) * 256) + (unsigned char)pbuf->at(2);
          p_ang = p_ang % 36000;

          circle_id = ((unsigned char)pbuf->at(pbuf->length()-60+2) * 256) + (unsigned char)pbuf->at(pbuf->length()-60+3);
          if(i == 0)
            circle_id_0 = circle_id;
        }
        break;

        case 0xFFAA:
        {
          // ROS_INFO("pbuf.length() = %d",pbuf->length());
          short l_mainming = (unsigned char)pbuf->at(22) << 8 | (unsigned char)pbuf->at(23);
          switch (l_mainming)
          {
          case 0x0401:
          {
            std::string l_data = *pbuf;
            boost::shared_ptr<boost::thread> udpthread_ptr;
            udpthread_ptr.reset(new boost::thread(boost::bind(&InputSocket720::Write0401, this, l_data)));
          }
          break;

          case 0x0402:
          {
            boost::unique_lock<boost::mutex> lock(mutex_bot);
            float l_tbtu = 0;
            float l_timu = 0;

            short l_ctmp = ((unsigned char)pbuf->at(46) | (((unsigned char)pbuf->at(45)) << 8));
            l_timu = l_ctmp / 100.0; // imu temperature
            l_ctmp = ((unsigned char)pbuf->at(44) | (((unsigned char)pbuf->at(43)) << 8));
            l_tbtu = l_ctmp / 100.0; // Bottom temperature

            if (l_timu == 0)
            {
              m_Buttont = l_timu;
            }
            else
            {
              m_Buttont = l_tbtu;
            }
            // ROS_INFO("m_Buttont update %f" , m_Buttont);

            // sensor_msgs::Imu imu_data;
            // imu_data.header.stamp = ros::Time::now();
            // imu_data.header.frame_id = "vanjee";
            // imu_data.angular_velocity.x = m_Buttont; // m_imuXstu.ImuAgv;//gx; // 1000;
            // imu_data.angular_velocity.y = 0;      // m_imuYstu.ImuAgv;//gy; // 1000;
            // imu_data.angular_velocity.z = 0;      // m_imuZstu.ImuAgv;//gz; // 1000;
            // sock_butpub.publish(imu_data);
            // ROS_INFO("Rm_Buttont = %f",m_Buttont);
          }
          break;

            // case 0x0501:
            // {

            // }
            // break;

          case 0x0514:
          {
            std::string l_data = *pbuf;
            boost::shared_ptr<boost::thread> udpthread_ptr2;
            udpthread_ptr2.reset(new boost::thread(boost::bind(&InputSocket720::WriVerAngtoCSV, this, l_data)));
          }
          break;

          case 0x0612:
          {
            std::string l_data = *pbuf;
            boost::shared_ptr<boost::thread> udpthread_ptr3;
            udpthread_ptr3.reset(new boost::thread(boost::bind(&InputSocket720::Write0612, this, l_data)));
          }
          break;

          case 0x0614:
          {
            std::string l_data = *pbuf;
            boost::shared_ptr<boost::thread> udpthread_ptr4;
            udpthread_ptr4.reset(new boost::thread(boost::bind(&InputSocket720::Write0614, this, l_data)));
          }
          break;
          }
        }
        break;

        } // switch
        // ROS_INFO("p_ang = %d",p_ang);

        if (p_header == 0xFFDD || p_header == 0xFFEE)
        {
          
          if (l_s && congig_yaml.pointSplitflag!=2)
          {
            if (congig_yaml.rmp != 1200 && p_ang == (int)(congig_yaml.start_ang) || (congig_yaml.rmp == 1200 && p_ang == (int)(congig_yaml.start_ang + 360)))
            // if(p_angpre > p_ang)
            {
              l_f = true;
              l_s = false;
            }
            else
            {
              
              continue;
            }
          }
          // ROS_INFO("i = %d",i);
          // fprintf(stderr, "i = %d  p_ang = %d\n" , i , p_ang);circle_id = ((unsigned char)pbuf->at(pbuf->length()-60+2) * 256) + (unsigned char)pbuf->at(pbuf->length()-60+3);
         
          i++;
          // if (i >= npackets && l_f)

          bool splitflag = false;
          switch(congig_yaml.pointSplitflag)
          {
            case 1:
            splitflag = ((i >= npackets) || (p_ang == (int)(congig_yaml.end_ang - 360)) && l_f) ;
            break;

            case 2:
            //splitflag = ((circle_id != circle_id_0 || (i >= npackets) || (p_ang == (int)(congig_yaml.end_ang - 360))  ) && l_f);
            splitflag = ((circle_id != circle_id_0));
            break;
          }

          //ROS_INFO("circle_id = %d, circle_id_0 = %d , circle_id_debug = %d , i = %d   lllllllll = %d" , circle_id , circle_id_0 , circle_id_debug,i , lllllllll++);

          if (splitflag)
          //if (((i >= npackets) || (p_ang == (int)(congig_yaml.end_ang - 360)) /*|| circle_id != circle_id_0*/) && l_f)
          // if ((p_ang == (int)(congig_yaml.end_ang-360)) && l_f)
          {
            // if( i != 100) 
            //   ROS_INFO("split package... i=%d, npackets = %d, p_ang = %d",i,npackets,p_ang);
            vanjee_msgs::VanjeePacket new_pkt;
            if(circle_id != circle_id_0 && congig_yaml.pointSplitflag==2)
            {
              new_pkt = scan->packets[i-1];
              scan->packets.pop_back();
              //ROS_INFO("i = %d ", i);
              if (i > 1)
              {
                i--;
              }
              // ROS_INFO("new_pkt.p_ang = %d",((unsigned char)new_pkt.data[6]* 256) + (unsigned char)new_pkt.data[5]);
              // ROS_INFO("new_pkt.circle_id = %d",((unsigned char)new_pkt.data[1253-60+2]* 256) + (unsigned char)new_pkt.data[1253-60+3]);
            }

            if (i >= npackets * 0.5)
            {

              vanjee_msgs::VanjeeScanPtr scanptr(new vanjee_msgs::VanjeeScan);
              scanptr->packets.resize(i);
              for(int xx=0;xx < i;xx++)
              {
                memcpy(&scanptr->packets[xx].data[0], &scan->packets[xx].data[0], pbuf->length());
              }
              

              // scan->packets.resize(0);
              // scan->packets.resize(npackets);
              double time2 = ros::Time::now().toSec();
              scanptr->header.stamp = ros::Time(time2);
              scanptr->header.frame_id = congig_yaml.frame_id; // std::string("wlr_720");
              socketpublisher.publish(scanptr);

              if(i != 100)
              {
                const uint8_t *pdata = (const uint8_t *)&scanptr->packets[0].data[0];
                ROS_INFO("pdata scanptr %d  i = %d",pdata[1249]*256+pdata[1250] , i);
              }
              

              l_s = true;
              ros::Rate loop(70);
              loop.sleep();
            }

            //ROS_INFO("i = %d  npackets = %d",i , npackets);
            i = 0;
            // double time2 = ros::Time::now().toSec();
            // scan->header.stamp = ros::Time(time2);
            // scan->header.frame_id = congig_yaml.frame_id; // std::string("wlr_720");
            // socketpublisher.publish(scan);
            
            // fprintf(stderr , "publish socket %d\n" , ((unsigned char)pbuf->at(1249) * 256) + (unsigned char)pbuf->at(1250));
            // l_s = true;
            // ros::Rate loop(50);
            // loop.sleep();

            scan->packets.resize(0);
            // scan = boost::make_shared<vanjee_msgs::VanjeeScan>();
            scan->packets.resize(npackets+10);

            if(circle_id != circle_id_0 && congig_yaml.pointSplitflag==2)
            {
              //ROS_INFO("different circle_id ,force split...,circle_id = %d,circle_id_0 = %d",circle_id,circle_id_0);
              circle_id_0 = circle_id;
              memcpy(&scan->packets[i].data[0], &new_pkt.data[0], 1600);
              i++;
            }
            
            // ROS_INFO("*******socketpublisher.publish(scan)****0xFFEE***** %d  %d   %f",sizeof(scan->packets),scan->packets.size(),time2);
          }

          p_angpre = p_ang;
        }

        delete pbuf;
        circle_id_0 = circle_id;
      } // if

    } // while
    ROS_INFO("*******out  publish *********");
  }

  void InputSocket720::euqueueIMUpacket()
  {
    std::string *pbuf = NULL;
    ros::Rate loop_(2000);

    ROS_INFO("into  euqueueIMUpacket");
    while (1)
    {
      loop_.sleep();
      if (queue_imu.size() > 0)
      {
        {
          boost::unique_lock<boost::mutex> lock(mutex_imu);
          pbuf = queue_imu.front();
          queue_imu.pop();
        }

        const char *buf = pbuf->c_str();

        ReadIMUTopublish((char *)buf, 1);
        // ROS_INFO("into  euqueueIMUpacket");

        delete pbuf;

      } // if

    } // while(1)
  }

  void InputSocket720::ReadIMUTopublish(char *pcaket, int i)
  {
    static double timePre = 0;
    sensor_msgs::Imu imu_data;
    imu_data.header.frame_id = "vanjee";

    const uint8_t *pbuf = nullptr;
    const uint8_t *pdata = (const uint8_t *)&pcaket[0];
    unsigned short l_header = (pdata[0] * 256) + pdata[1];
    // fprintf(stderr,"hearer  ****   %x\n",l_header);
    switch (l_header)
    {
    case 0xFFEE:
      pbuf = (const uint8_t *)&pdata[1200];
      break;
    case 0xFFDD:
      int l_len = (pdata[3] * pdata[4]) * (pdata[2] * 4 + 2) + 5;
      pbuf = (const uint8_t *)&pdata[l_len];
      break;
    }

    if (pbuf != nullptr)
    {
      imu_data.angular_velocity.x = (short)(pbuf[14] | (((pbuf[15]) << 8) & 0xff00)); // * 8.75;
      imu_data.angular_velocity.y = (short)(pbuf[16] | (((pbuf[17]) << 8) & 0xff00)); // * 8.75;
      imu_data.angular_velocity.z = (short)(pbuf[18] | (((pbuf[19]) << 8) & 0xff00)); // * 8.75;
      imu_data.linear_acceleration.x = (short)(pbuf[20] + ((pbuf[21]) * 256));        // * 0.061;
      imu_data.linear_acceleration.y = (short)(pbuf[22] + ((pbuf[23]) * 256));        // * 0.061;
      imu_data.linear_acceleration.z = (short)(pbuf[24] + ((pbuf[25]) * 256));        // * 0.061;

      imu_data.orientation.x = m_Buttont;

      // GetImuLocalTime
      std::tm stm;
      memset(&stm, 0, sizeof(stm));

      stm.tm_year = pbuf[9] + 100;
      stm.tm_mon = pbuf[8] - 1;
      stm.tm_mday = pbuf[7];
      stm.tm_hour = pbuf[6];
      stm.tm_min = pbuf[5];
      stm.tm_sec = pbuf[4];
      int nsec = (pbuf[10] + (pbuf[11] << 8) + (pbuf[12] << 16) + ((pbuf[13] & 0x0F) << 24));
      double nnsec = std::mktime(&stm) + nsec / 100000000.0;

      double l_dertaTi = nnsec - timePre;
      // ROS_INFO("l_dertaTi = %f    nsec = %d" , l_dertaTi , nsec);

      if ((l_dertaTi > 0.015) || (l_dertaTi < 0.005))
      {
        timePre = nnsec;
        return;
      }

      timePre = nnsec;

      if (!congig_yaml.timemode)
      {
        imu_data.header.stamp = ros::Time::now();
      }
      else
      {
        imu_data.header.stamp = ros::Time().fromSec(nnsec);
      }

      imu_data.orientation.y = nnsec;
      imu_data.orientation.z = nsec;
      // ROS_INFO("nsec = %d", nsec);

      sock_butpub.publish(imu_data);
    }
  }

  int InputSocket720::UpImuHzPacket(char *pcaket)
  {
    unsigned short p_header;
    int l_npackets;
    float l_ibe;
    float l_ien;
    float l_val; // = congig_yaml.rmp / 3000.0;//[600r,0.2],[300r,0.1]
    // ROS_INFO("*************************************congig_yaml.start_ang : %f  %f",congig_yaml.start_ang,congig_yaml.end_ang);
    p_header = ((unsigned char)pcaket[0] * 256) + (unsigned char)pcaket[1];
    switch (p_header)
    {
    case 0xFFEE:
    {
      l_npackets = 12;
    }
    break;

    case 0xFFDD:
    {
      uint8_t l_blocknum = (unsigned char)pcaket[4];
      l_npackets = 180 / l_blocknum;
    }
    break;
    }

    ROS_INFO("imunpackets : %d", l_npackets);

    pushImupacketflag = true;
    return l_npackets;
  }

  ////////////////////////////////////////////////////////////////////////
  // InputPCAP720 class implementation
  ////////////////////////////////////////////////////////////////////////

  /** @brief constructor
   *
   *  @param private_nh ROS private handle for calling node.
   *  @param port UDP port number
   *  @param packet_rate expected device packet frequency (Hz)
   *  @param filename PCAP dump file name
   */
  InputPCAP720::InputPCAP720(ros::NodeHandle private_nh, uint16_t port, double packet_rate, std::string filename,
                             bool read_once, bool read_fast, double repeat_delay)
      : Input(private_nh, 0, "", "")
  {
    filename_ = filename;
    pcap_ = NULL;
    empty_pcap = true;
    npackets = 120;
    m_Buttont = -10000;
    mangstr = 0;
    mangend = 360;

    // Open the PCAP dump file
    // ROS_INFO("Opening PCAP file \"%s\"", filename_.c_str());
    ROS_INFO_STREAM("Opening PCAP file " << filename_);
    // fprintf(stderr, "rePCAP file:%s\n", filename_.c_str());
    if ((pcap_ = pcap_open_offline(filename_.c_str(), errbuf_)) == NULL)
    {
      fprintf(stderr, "Error opening wjlidar socket dump file\n");
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
  InputPCAP720::~InputPCAP720(void)
  {
    pcap_close(pcap_);
  }

  void InputPCAP720::refreshPackets(int &packs, unsigned char *pbuf, int rpm)
  {
    if (packs != npackets)
    {
      if (pbuf[0] == 0xFF && pbuf[1] == 0xDD)
      {
        npackets = rpm / 30 * (pbuf[4]);
        npackets = 36000 / npackets;
        packs = npackets;
        fprintf(stderr, "refreshPackets1:%d channel:%d npacket:%d rpm:%d\n", npackets, pbuf[3], pbuf[4], rpm);
      }
      else if (pbuf[0] == 0xFF && pbuf[1] == 0xEE)
      {
        npackets = 72000 / rpm;
        packs = npackets;
        fprintf(stderr, "refreshPackets2:%d rpm:%d\n", npackets, rpm);
      }
    }
  }

  void InputPCAP720::resleepPackets(int rpm, double time)
  {
    // if (npackets != 0)
    // {
    //   float l_ti = 1 / (60.0 / rpm - npackets / 3000.0) + 0.5;
    //   ros::Rate loop_rate(10.1);
    //   ROS_INFO("%f",l_ti);
    //   loop_rate.sleep();
    // }

    double l_ti = 1.0 / (60.0 / rpm - time);
    ros::Rate loop_rate(l_ti);
    loop_rate.sleep();
  }

  void InputPCAP720::startpublisher()
  {
    boost::shared_ptr<boost::thread> udpthread_ptr1;
    udpthread_ptr1.reset(new boost::thread(boost::bind(&InputPCAP720::euqueuepacket, this)));
  }

  void InputPCAP720::euqueuepacket()
  {
    vanjee_msgs::VanjeeScanPtr scan(new vanjee_msgs::VanjeeScan);
    scan->packets.resize(200);
    static int firstpro = 0;
    double timenow;
    double timepre;

    // npackets = 100;
    while (1)
    {
      timenow = ros::Time::now().toSec();
      for (int i = 0; i < npackets; ++i)
      {
        while (true)
        {
          // keep reading until full packet received
          int rc = pcap_getpacket(&scan->packets[i]);
          // ROS_INFO("%d ",rc);
          if (rc == 0)
          {
            unsigned char *pdata = static_cast<unsigned char *>(&scan->packets[i].data[0]);
            const char *l_pdata = (const char *)pdata;

            if (pdata[1] == 0xDD)
            {
              unsigned short l_ang = ((unsigned char)pdata[6] * 256) + (unsigned char)pdata[5]; // FFDD
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
                if (l_ang < congig_yaml.start_ang && l_ang > congig_yaml.end_ang)
                {
                  i--;
                }
                else
                {
                }
              }
            }

            if (!setnpacketsflag) // && pdata[1] ==0xDD)
            {
              std::string l_str = l_pdata; //=new std::string s();
              npackets = setnpackets720((char *)pdata);
              scan->packets.resize(npackets);
              setnpacketsflag = true;
              break;
            }
            // else if(!setnpacketsflag && pdata[1] ==0xEE)
            // {
            //   npackets = 120;
            //   scan->packets.resize(npackets);
            //   setnpacketsflag = true;
            // }
            break; // got a full packet?
          }

          if (rc < 0)
            continue;
          ; // end of file reached?
        }
        // unsigned char *pdata = static_cast<unsigned char *>(&scan->packets[i].data[0]);

        // fprintf(stderr,"msop_input_->config_.npackets = %d \n",msop_input_->config_.npackets);
        //  publish packet from horizon angle 0
      }

      scan->header.stamp = ros::Time(ros::Time::now().toSec());
      scan->header.frame_id = congig_yaml.frame_id;
      socketpublisher.publish(scan);
      // ROS_INFO("char %f ",timepre - timenow);
      timepre = ros::Time::now().toSec();
      // resleepPackets(congig_yaml.rmp,timepre - timenow);
      // ROS_INFO("congig_yaml.rmp %d",congig_yaml.rmp);
      resleepPackets(600, timepre - timenow);
      // ROS_INFO("publish");
    }
  }

}
