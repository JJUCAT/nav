/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */

/*
 * Author: Paul Bovbel
 */

#ifndef POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
#define POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET

#include "ros/ros.h"
#include "boost/thread/mutex.hpp"

#include "nodelet/nodelet.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "message_filters/subscriber.h"
#include "sensor_msgs/PointCloud2.h"

#include <pcl/point_types.h>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pointcloud_to_laserscan/RobotFootprintConfig.h>
#include <dynamic_reconfigure/server.h>
// ground segmentation library
#include "patchworkpp/patchworkpp.hpp"


namespace pointcloud_to_laserscan
{
  typedef tf2_ros::MessageFilter<sensor_msgs::PointCloud2> MessageFilter;
/**
* Class to process incoming pointclouds into laserscans. Some initial code was pulled from the defunct turtlebot
* pointcloud_to_laserscan implementation.
*/
  class PointCloudToLaserScanNodelet : public nodelet::Nodelet
  {

    public:
      PointCloudToLaserScanNodelet();

    private:
      virtual void onInit();

      void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
      void failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
          tf2_ros::filter_failure_reasons::FilterFailureReason reason);

      void connectCb();

      void disconnectCb();
      void setupDynamicReconfigure(ros::NodeHandle& nh);

      ros::NodeHandle nh_, private_nh_;
      ros::Publisher pub_;
      ros::Publisher pub_ground_;
      ros::Publisher pub_nonground_;
      ros::Publisher pub_no_footprint_;
      boost::mutex connect_mutex_;

      boost::shared_ptr<tf2_ros::Buffer> tf2_;
      boost::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
      message_filters::Subscriber<sensor_msgs::PointCloud2> sub_;
      boost::shared_ptr<MessageFilter> message_filter_;
      boost::shared_ptr<PatchWorkpp> PatchworkppGroundSeg_;

      template<typename T>
      sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "base_link")
      {
          sensor_msgs::PointCloud2 cloud_ROS;
          pcl::toROSMsg(cloud, cloud_ROS);
          cloud_ROS.header.stamp = stamp;
          cloud_ROS.header.frame_id = frame_id;
          return cloud_ROS;
      }

      // ROS Parameters
      unsigned int input_queue_size_;
      std::string target_frame_;
      double tolerance_;
      double min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
      bool use_inf_;
      double range_x_min_,range_x_max_,range_y_min_,range_y_max_,range_z_min_,range_z_max_;
      bool use_cliff_detect_;
      bool use_brush_range_filter_;
      double brush_x_min_,brush_x_max_,brush_y_min_,brush_y_max_,brush_z_min_,brush_z_max_;
      int cliff_detect_min_pts_;
      double cliff_detect_block_length_,cliff_detect_x_min_,cliff_detect_x_max_,cliff_detect_y_min_,cliff_detect_y_max_;
      bool use_ground_filter_,use_radiu_filter_;
      bool is_middle_lidar_;
      double dynamic_height_range_start_,dynamic_height_gap_per_half_mile_;
      double radiu_threshold_;
      int min_neighbors_;
      bool pub_ground_and_nonground_;

      dynamic_reconfigure::Server<pointcloud_to_laserscan::RobotFootprintConfig> *dsrv_;
      void reconfigureCB(pointcloud_to_laserscan::RobotFootprintConfig &config, uint32_t level);
    };

}  // pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN_POINTCLOUD_TO_LASERSCAN_NODELET
