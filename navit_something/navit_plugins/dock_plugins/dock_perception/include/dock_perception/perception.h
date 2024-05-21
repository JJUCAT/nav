#ifndef DOCK_PERCEPTION_H
#define DOCK_PERCEPTION_H

#include <dock_perception/dock_candidate.h>
#include <dock_perception/laser_processor.h>
#include <dock_perception/linear_pose_filter_2d.h>
#include <navit_auto_dock/plugins/approach_dock_perception.h>
#include <dock_perception/dock_shape_base.h>

#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h> 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <pluginlib/class_loader.hpp>

#include <deque>
#include <mutex>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include <pcl/point_types.h> 
#include <pcl/filters/impl/passthrough.hpp> 
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/impl/pcl_base.hpp> 
#include <pcl/filters/passthrough.h>
#include<pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/segmentation/extract_clusters.h> 
#include <pcl/search/organized.h> 
#include"navit_msgs/PerceptionCalibration.h"
// #include"navit_msgs/PerceptionCalibrationPose.h"


namespace dock_perception {

class Perception : public navit_auto_dock::plugins::ApproachDockPerception {
public:
  Perception():
    dock_shape_("dock_perception", "dock_perception::DockShapeBase")
  {
  
  }

  void initialize(const std::string name,
                  const std::shared_ptr<tf2_ros::Buffer> &tf);

  /**
   * @brief Start dock detection.
   * @param pose The initial estimate of dock pose
   */
  bool start(const geometry_msgs::PoseStamped &initial_dock_pose);

  /** @brief Stop tracking the dock. */
  bool stop();

  /** @brief Get the pose of the dock. */
  bool getPose(geometry_msgs::PoseStamped &pose);

  bool calibrate(navit_msgs::PerceptionCalibration::Request&req,
                 navit_msgs::PerceptionCalibration::Response&res);

  // bool getCalibratePose(navit_msgs::PerceptionCalibrationPose::Request&req,
  //                navit_msgs::PerceptionCalibrationPose::Response&res);

private:
  /** @brief Callback to process laser scans */
  void callback(const sensor_msgs::LaserScanConstPtr &scan);

  void callbackLaserScan(const sensor_msgs::LaserScanConstPtr &scan);

  void callbackPointCloud2(const sensor_msgs::PointCloud2ConstPtr& scan);

  /**
   * @brief Extract a DockCandidate from a cluster, filling in the
   *        lengths and slopes of each line found using ransac.
   * @param cluster The pointer to the cluster to extract from.
   */
  DockCandidatePtr extract(laser_processor::SampleSet *cluster);

  /**
   * @brief Try to fit a dock to candidate
   * @param candidate The candidate to fit to.
   * @param pose The fitted pose, if successful.
   * @returns Fitness score (>0 if successful)
   */
  double fit(const DockCandidatePtr &candidate, geometry_msgs::Pose &pose);

  /**
   * @brief Method to check if the quaternion is valid.
   * @param q Quaternion to check.
   * @return True if quaternion is valid.
   */
  static bool isValid(const tf2::Quaternion &q);

  ros::NodeHandle nh;
  ros::Subscriber scan_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  bool running_; // Should we be trying to find the dock
  bool debug_;   // Should we output debugging info

  LinearPoseFilter2DPtr
      dock_pose_filter_; /// Low pass filter object for filtering dock poses.

  // Best estimate of where the dock is
  geometry_msgs::PoseStamped dock_,calib_dock_;
  geometry_msgs::TransformStamped offset_transform;
  float offset_translation_x_,offset_translation_y_,offset_rotation_x_,
  offset_rotation_y_,offset_rotation_z_,offset_rotation_w_;
  // Mutex on dock_
  std::mutex dock_mutex_;
  // If true, then dock_ is based on actual sensor data
  bool found_dock_;
  // Last time that dock pose was updated
  ros::Time dock_stamp_;

  // Maximum allowable error between scan and "ideal" scan
  double max_alignment_error_;
  // TF frame to track dock within
  std::string tracking_frame_;
  std::string base_frame_;
  // Maximum iterations ICP run
  int max_icp_iterations_;

  bool use_filter_ = false;

  // Publish visualization of which points are detected as dock
  ros::Publisher debug_points_;

  pluginlib::ClassLoader<DockShapeBase> dock_shape_;

  // The ideal cloud, located at origin of dock frame
  std::vector<geometry_msgs::Point> ideal_cloud_;
  // The ideal cloud (when only front is visible)
  std::vector<geometry_msgs::Point> front_cloud_;

  std::vector<geometry_msgs::Point> laser_cloud_buffer;
  int laser_cloud_buffer_count;
  float v_range_x_min,v_range_x_max,v_range_y_min,v_range_y_max,v_range_z_min,v_range_z_max;
  float pose_tolerance_;
  int v_intensity_threshold,v_frame_number_of_reflective_stripe_cloud;
  ros::NodeHandle sn,sn_get_pose;
  ros::ServiceServer service,service_pose;
  ros::Publisher ori_dock_pose_pub_,calib_dock_pose_pub_;
  float v_range_x_min_2d,v_range_x_max_2d,v_range_y_min_2d,v_range_y_max_2d;

};
}
#endif // DOCK_PERCEPTION_H
