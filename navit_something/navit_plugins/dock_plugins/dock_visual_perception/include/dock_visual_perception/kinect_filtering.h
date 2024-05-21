#ifndef KINECT_FILTERING_H
#define KINECT_FILTERING_H

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/registration.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/extract_indices.h>
#include <boost/lexical_cast.hpp>
#include <Eigen/StdVector>

#include <opencv2/core/core.hpp>

#include <tf/LinearMath/Matrix3x3.h>

namespace dock_visual_perception
{
typedef pcl::PointXYZRGB ARPoint;
typedef pcl::PointCloud<ARPoint> ARCloud;

// Result of plane fit: inliers and the plane equation
struct PlaneFitResult
{
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  PlaneFitResult() : inliers(ARCloud::Ptr(new ARCloud))
  {
  }
  ARCloud::Ptr inliers;
  pcl::ModelCoefficients coeffs;
};

// Select out a subset of a cloud corresponding to a set of pixel coordinates
ARCloud::Ptr filterCloud(
    const ARCloud& cloud,
    const std::vector<cv::Point, Eigen::aligned_allocator<cv::Point> >& pixels);

// Wrapper for PCL plane fitting
PlaneFitResult fitPlane(ARCloud::ConstPtr cloud);

// Given the coefficients of a plane, and two points p1 and p2, we produce a
// quaternion q that sends p2'-p1' to (1,0,0) and n to (0,0,1), where p1' and
// p2' are the projections of p1 and p2 onto the plane and n is the normal.
// There's a sign ambiguity here, which is resolved by requiring that the
// difference p4'-p3' ends up with a positive y coordinate
int extractOrientation(const pcl::ModelCoefficients& coeffs, const ARPoint& p1,
                       const ARPoint& p2, const ARPoint& p3, const ARPoint& p4,
                       geometry_msgs::Quaternion& retQ);

// Like extractOrientation except return value is a btMatrix3x3
int extractFrame(const pcl::ModelCoefficients& coeffs, const ARPoint& p1,
                 const ARPoint& p2, const ARPoint& p3, const ARPoint& p4,
                 tf::Matrix3x3& retmat);

// Return the centroid (mean) of a point cloud
geometry_msgs::Point centroid(const ARCloud& points);
}

#endif  // include guard
