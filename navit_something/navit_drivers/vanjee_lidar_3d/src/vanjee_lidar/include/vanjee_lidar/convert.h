

#ifndef _vanjee_pointcloud_CONVERT_H_
#define _vanjee_pointcloud_CONVERT_H_ 1

#include <ros/ros.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <vanjee_lidar/rawdata.hpp>
#include <vanjee_lidar/lidar720.h>
#include <vanjee_lidar/lidar721.h>
#include "vanjee_lidar/complementary_filter.h"

#include <dynamic_reconfigure/server.h>
#include <pcl_ros/point_cloud.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#include "complementary_filter.h"

namespace vanjee_lidar
{
  class Convert
  {
  public:

    Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
    ~Convert() {}
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

    // imu_tools::ComplementaryFilter m_fiter;

  private:

    boost::shared_ptr<vanjee_rawdata::RawData> data_point[8];

    ros::NodeHandle priv_nh_;
  };

} // namespace vanjee_lidar

#endif // _vanjee_pointcloud_CONVERT_H_
