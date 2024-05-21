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
 */

#ifndef _VJ_LIDAR_DRIVER_H_
#define _VJ_LIDAR_DRIVER_H_

#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Int32.h>
#include <pcl/point_types.h>
#include <pcl_ros/impl/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "vanjee_driver/input.hpp"
#include "vanjee_driver/driver720.h"
#include "vanjee_driver/driver721.h"
#include <dynamic_reconfigure/server.h>
#include <vanjee_driver/VanjeeNodeConfig.h>
#include <sensor_msgs/Imu.h>
namespace vanjee_driver
{
class vanjeeDriver
{
public:
  /**
 * @brief vanjeeDriver
 * @param node          raw packet output topic
 * @param private_nh    通过这个节点传参数
 */
  vanjeeDriver(ros::NodeHandle node, ros::NodeHandle private_nh);

  ~vanjeeDriver();

  bool poll(void);

private:
  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<vanjee_driver::VanjeeNodeConfig> > srv_;
  

  boost::shared_ptr<Input> msop_input_;
  boost::shared_ptr<Input> difop_input_;
  boost::shared_ptr<Input> lidarTnput[8];
  ros::Publisher msop_output_;
  ros::Publisher difop_output_;
  ros::Publisher output_sync_;
  ros::Publisher output_but;
  // Converter convtor_
  boost::shared_ptr<boost::thread> difop_thread_;

  // add for time synchronization
  bool time_synchronization_;
    unsigned char packetTimeStamp[10];
    uint64_t pointcloudTimeStamp;
    uint64_t GPSStableTS;
    uint64_t GPSCountingTS;
    uint64_t last_FPGA_ts;
    uint64_t GPS_ts;
    struct tm cur_time;
    int cnt_gps_ts;
    ros::Time timeStamp;

};

}  // namespace vanjeelidar_driver

#endif
