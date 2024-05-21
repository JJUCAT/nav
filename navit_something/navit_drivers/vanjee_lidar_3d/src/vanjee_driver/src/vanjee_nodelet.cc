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

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "vanjee_driver/vanjee_driver.h"

volatile sig_atomic_t flag = 1;

namespace vanjee_driver
{
class DriverNodelet : public nodelet::Nodelet
{
public:
  DriverNodelet() : running_(false)
  {
  }

  ~DriverNodelet()
  {
    if (running_)
    {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

private:
  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;  ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<vanjeeDriver> dvr_;  ///< driver implementation class
};

void DriverNodelet::onInit()
{
  // start the driver
  dvr_.reset(new vanjeeDriver(getNodeHandle(), getPrivateNodeHandle()));

  // spawn device poll thread
  running_ = true;
  //deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
  // NODELET_INFO("DriverNodelet onInit");
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll()
{
  ros::Rate loop_rate(120);
  while (ros::ok() && dvr_->poll())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }
}
}

//
// parameters are: class type, base class type
PLUGINLIB_EXPORT_CLASS(vanjee_driver::DriverNodelet, nodelet::Nodelet)
