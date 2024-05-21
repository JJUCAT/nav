#include <dock_perception/icp_2d.h>
#include <dock_perception/perception.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <list>
#include <queue>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(dock_perception::Perception,
                       navit_auto_dock::plugins::ApproachDockPerception)

namespace dock_perception {
inline double getPoseDistance(const geometry_msgs::Pose a,
                              const geometry_msgs::Pose b) {
  double dx = a.position.x - b.position.x;
  double dy = a.position.y - b.position.y;
  return sqrt(dx * dx + dy * dy);
}

void Perception::initialize(const std::string name,
                            const std::shared_ptr<tf2_ros::Buffer> &tf) {
  running_ = false;
  found_dock_ = false;
  tf_buffer_ = tf;

  ros::NodeHandle pnh("~/" + name);
  tracking_frame_ = "odom";
  pnh.param("tracking_frame", tracking_frame_, tracking_frame_);

  pnh.param("reflective_stripe_range_x_min", v_range_x_min, 0.5f);
  pnh.param("reflective_stripe_range_x_max", v_range_x_max, 2.0f);
  pnh.param("reflective_stripe_range_y_min", v_range_y_min, -5.0f);
  pnh.param("reflective_stripe_range_y_max", v_range_y_max, 5.0f);
  pnh.param("reflective_stripe_range_z_min", v_range_z_min, 0.1f);
  pnh.param("reflective_stripe_range_z_max", v_range_z_max, 0.5f);
  pnh.param("reflective_stripe_intensity_threshold", v_intensity_threshold, 180);//v_frame_number_of_reflective_stripe_cloud
  pnh.param("frame_number_of_reflective_stripe_cloud", v_frame_number_of_reflective_stripe_cloud, 5);
  pnh.param("offset_translation_x", offset_translation_x_, 0.0f);
  pnh.param("offset_translation_y", offset_translation_y_, 0.0f);
  pnh.param("offset_rotation_x", offset_rotation_x_, 0.0f);
  pnh.param("offset_rotation_y", offset_rotation_y_, 0.0f);
  pnh.param("offset_rotation_z", offset_rotation_z_, 0.0f);
  pnh.param("offset_rotation_w", offset_rotation_w_, 1.0f);//pose_tolerance_
  pnh.param("pose_tolerance", pose_tolerance_, 0.4f);
  pnh.param("reflective_stripe_range_x_min_2d", v_range_x_min_2d, -3.0f);
  pnh.param("reflective_stripe_range_x_max_2d", v_range_x_max_2d, -0.4f);
  pnh.param("reflective_stripe_range_y_min_2d", v_range_y_min_2d, -5.0f);
  pnh.param("reflective_stripe_range_y_max_2d", v_range_y_max_2d, 5.0f);

  // Should we publish the debugging cloud
  if (!pnh.getParam("debug", debug_)) {
    debug_ = true;
    ROS_WARN("2d lidar perception debug is set to true");
  }

  // Create coefficient vectors for a second order butterworth filter
  // with a cutoff frequency of 10 Hz assuming the loop is updated at 50 Hz
  // [b, a] = butter(order, cutoff_frequ/half_sampling_freq)
  // [b, a] = butter(2, 10/25)
  float b_arr[] = {0.20657, 0.41314, 0.20657};
  float a_arr[] = {1.00000, -0.36953, 0.19582};
  std::vector<float> b(b_arr, b_arr + sizeof(b_arr) / sizeof(float));
  std::vector<float> a(a_arr, a_arr + sizeof(a_arr) / sizeof(float));
  pnh.getParam("low_pass_filter/b_array", b);
  pnh.getParam("low_pass_filter/a_array", a);

  pnh.param("use_filter", use_filter_, use_filter_);
  dock_pose_filter_.reset(new LinearPoseFilter2D(b, a));

  // Limit the average reprojection error of points onto
  // the ideal dock. This prevents the robot docking
  // with something that is very un-dock-like.
  if (!pnh.getParam("max_alignment_error", max_alignment_error_)) {
    max_alignment_error_ = 0.4;
  }

  std::string dock_shape_name = "v_shape";
  std::string dock_shape_plugin;
  pnh.param("dock_shape", dock_shape_name, dock_shape_name);
  if (dock_shape_name == "v_shape")
     dock_shape_plugin = "dock_perception/DockShapeV"; 
  else if (dock_shape_name == "mir_shape")
     dock_shape_plugin = "dock_perception/DockShapeMir"; 
  else
  {
     ROS_WARN("dock_shape %s not defined, use V shaped instead",dock_shape_name.c_str());
     dock_shape_plugin = "dock_perception/DockShapeV"; 
  }
  auto dock_shape_ptr = dock_shape_.createInstance(dock_shape_plugin);

  //auto cloud = dock_shape_ptr->getIdealCloud(pnh);
  ideal_cloud_ = dock_shape_ptr->getIdealCloud(pnh);

  laser_cloud_buffer.resize(0);
  laser_cloud_buffer_count=0;
  
  // Debugging publishers first
  if (debug_) {
    debug_points_ = nh.advertise<sensor_msgs::PointCloud2>("dock_points", 10);
  }

  // Init base scan only after publishers are created
  std::string scan_topic_name = "base_scan";
  pnh.param("scan_topic_name", scan_topic_name, scan_topic_name);
  bool use_scandata = false;
  pnh.param("use_scandata", use_scandata, use_scandata);
  if (use_scandata) scan_sub_ = nh.subscribe(scan_topic_name, 1, &Perception::callbackLaserScan, this); // 激光类型的数据
  else scan_sub_ = nh.subscribe(scan_topic_name, 1, &Perception::callbackPointCloud2, this); // 点云类型的数据
  pnh.param("max_icp_iterations", max_icp_iterations_, max_icp_iterations_);

  // service_pose=sn_get_pose.advertiseService("get_calibrate_pose", &Perception::getCalibratePose,this);
  service = sn.advertiseService("charge_calibrate", &Perception::calibrate,this);
  if(service){
    ROS_INFO("charge_calibrate service create success");
  }
  else{
    ROS_ERROR("charge_calibrate service create failed");
  }
  offset_transform.header.frame_id=tracking_frame_;
  offset_transform.header.stamp=ros::Time::now();
  offset_transform.child_frame_id=tracking_frame_;
  offset_transform.transform.translation.x=offset_translation_x_;
  offset_transform.transform.translation.y=offset_translation_y_;
  offset_transform.transform.translation.z=0.0;
  offset_transform.transform.rotation.x=offset_rotation_x_;
  offset_transform.transform.rotation.y=offset_rotation_y_;
  offset_transform.transform.rotation.z=offset_rotation_z_;
  offset_transform.transform.rotation.w=offset_rotation_w_;

  ori_dock_pose_pub_=sn.advertise<geometry_msgs::PoseStamped>("ori_perception_dock_pose",1);
  calib_dock_pose_pub_=sn.advertise<geometry_msgs::PoseStamped>("calib_perception_dock_pose",1);

  ROS_INFO_NAMED("perception", "Dock perception initialized");
}

bool Perception::start(const geometry_msgs::PoseStamped &initial_dock_pose) {
  found_dock_ = false;
  running_ = true;
  dock_.header.frame_id = "";
  // if (initial_dock_pose.header.frame_id == tracking_frame_)
  //   dock_ = initial_dock_pose;
  // else {
  //   try {
  //     geometry_msgs::TransformStamped to_tracking_frame_tf;
  //     to_tracking_frame_tf = tf_buffer_->lookupTransform(
  //         tracking_frame_, initial_dock_pose.header.frame_id, ros::Time(0));
  //     dock_.header.frame_id = tracking_frame_;
  //     tf2::doTransform(initial_dock_pose, dock_, to_tracking_frame_tf);
  //   } catch (tf2::TransformException const &ex) {
  //     ROS_WARN_STREAM("Couldn't transform to tracking frame "
  //                     << tracking_frame_);
  //     return false;
  //   }
  // }
  return true;
}

bool Perception::stop() {
  running_ = false;
  dock_.header.frame_id = "";
  return true;
}

bool Perception::getPose(geometry_msgs::PoseStamped &pose) {
  // All of this requires a lock on the dock_
  std::lock_guard<std::mutex> lock(dock_mutex_);
// std::cout<<"Perception::getPose 0"<<std::endl;
  if (!found_dock_)
    return false;
  // TODO: ros param timeout
  if (ros::Time::now() > dock_stamp_ + ros::Duration(pose_tolerance_)) {
    ROS_INFO("deta is %f",(ros::Time::now() - dock_stamp_));
    ROS_ERROR("dock_perception", "Dock pose timed out");
    return false;
  }
  // Check for a valid orientation.
  tf2::Quaternion q;
  tf2::convert(dock_.pose.orientation, q);
  if (!isValid(q)) {
    ROS_ERROR("perception", "Dock orientation invalid.");
    // ROS_DEBUG("perception", "Quaternion magnitude is "
    //                                          << q.length() << " Quaternion is ["
    //                                          << q.x() << ", " << q.y() << ", "
    //                                          << q.z() << ", " << q.w() << "]");
    return false;
  }
  // tf2::Quaternion q_offset;
  // q_offset.setRPY(0, 0, 0);
  // q = q * q_offset;
  // dock_.pose.orientation = tf2::toMsg(q);
  pose = dock_;
   try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        "base_link", dock_.header.frame_id, ros::Time(0));
    tf2::doTransform(pose, pose, transform_scan);   
    pose.header.frame_id=offset_transform.child_frame_id;
    tf2::doTransform(pose, pose, offset_transform);
    pose.header.frame_id="base_link";
    transform_scan = tf_buffer_->lookupTransform(dock_.header.frame_id,
        "base_link", ros::Time(0));
    tf2::doTransform(pose, pose, transform_scan); 
    pose.header.frame_id=dock_.header.frame_id;
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform pose  to baselink frame");
    return false;
  }
  // pose.header.frame_id=offset_transform.child_frame_id;
  // tf2::doTransform(pose, pose, offset_transform);
  // pose.header.frame_id=dock_.header.frame_id;
  // calib_dock_pose_pub_.publish(pose);
  return found_dock_;
}

void Perception::callback(const sensor_msgs::LaserScanConstPtr &scan) {
  // Be lazy about search
  if (!running_) {
    return;
  }
  // Make sure goal is valid (orientation != 0 0 0 0)
  if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0)) {
    // Lock the dock_
    std::lock_guard<std::mutex> lock(dock_mutex_);

    // If goal is invalid, set to a point directly ahead of robot
    for (size_t i = scan->ranges.size() / 2; i < scan->ranges.size(); i++) {
      if (std::isfinite(scan->ranges[i])) {
        double angle = scan->angle_min + i * scan->angle_increment;
        dock_.header = scan->header;
        dock_.pose.position.x = cos(angle) * scan->ranges[i];
        dock_.pose.position.y = sin(angle) * scan->ranges[i];
        dock_.pose.orientation.x = 0.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 1.0;
        ROS_DEBUG_NAMED("dock_perception", "Set initial pose to (%f, %f, %f)",
                        dock_.pose.position.x, dock_.pose.position.y,
                        icp_2d::thetaFromQuaternion(dock_.pose.orientation));
        break;
      }
    }
  }

  // Make sure goal is in the tracking frame
  if (dock_.header.frame_id != tracking_frame_) {
    std::lock_guard<std::mutex> lock(dock_mutex_);
    try {
      geometry_msgs::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
          tracking_frame_, dock_.header.frame_id, ros::Time(0));
      dock_.header.frame_id = tracking_frame_;
      tf2::doTransform(dock_, dock_, transform);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Couldn't transform dock pose to tracking frame");
      return;
    }
    ROS_WARN_NAMED("dock_perception",
                   "Transformed initial pose to (%f, %f, %f)",
                   dock_.pose.position.x, dock_.pose.position.y,
                   icp_2d::thetaFromQuaternion(dock_.pose.orientation));
  }

  // Cluster the laser scan
  laser_processor::ScanMask mask;
  laser_processor::ScanProcessor processor(*scan, mask,v_range_x_min_2d,v_range_x_max_2d,v_range_y_min_2d,v_range_y_max_2d,v_intensity_threshold);
  processor.splitConnected(0.04); // TODO(enhancement) parameterize
  processor.removeLessThan(5);
  // Sort clusters based on distance to last dock
  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>,
                      CompareCandidates>
      candidates;
  for (std::list<laser_processor::SampleSet *>::iterator i =
           processor.getClusters().begin();
       i != processor.getClusters().end(); i++) {
    DockCandidatePtr c = extract(*i);
    if (c && c->valid(found_dock_)) {
      candidates.push(c);
    }
  }
  ROS_DEBUG_STREAM_NAMED("dock_perception", "Extracted " << candidates.size()
                                                         << " clusters");
  // Extract ICP pose/fit on best clusters
  DockCandidatePtr best;
  geometry_msgs::Pose best_pose;
  while (!candidates.empty()) {
    geometry_msgs::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    if (score >= 0) {
      best = candidates.top();
      best_pose = pose;
      break;
    } else // Let's see what's wrong with this point cloud.
    {
      if (debug_) {
        DockCandidatePtr not_best = candidates.top();

        // Create point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = scan->header.stamp;
        cloud.header.frame_id = tracking_frame_;
        cloud.width = cloud.height = 0;

        // Allocate space for points
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(not_best->points.size());

        // Fill in points
        sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
        for (size_t i = 0; i < not_best->points.size(); i++) {
          cloud_iter[0] = not_best->points[i].x;
          cloud_iter[1] = not_best->points[i].y;
          cloud_iter[2] = not_best->points[i].z;
          ++cloud_iter;
        }
        debug_points_.publish(cloud);
      }
    }
    candidates.pop();
  }
  // Did we find dock?
  if (!best) {
    ROS_DEBUG_NAMED("dock_perception", "DID NOT FIND THE DOCK");
    return;
  }
  ROS_DEBUG_NAMED("dock_perception", "Found the dock.");

  // Update
  if (debug_) {
    // Create point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = tracking_frame_;
    cloud.width = cloud.height = 0;

    // Allocate space for points
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(best->points.size());

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
    for (size_t i = 0; i < best->points.size(); i++) {
      cloud_iter[0] = best->points[i].x;
      cloud_iter[1] = best->points[i].y;
      cloud_iter[2] = best->points[i].z;
      ++cloud_iter;
    }
    debug_points_.publish(cloud);
  }

  // Everything after this modifies the dock_
  std::lock_guard<std::mutex> lock(dock_mutex_);

  // Update stamp
  dock_.header.stamp = scan->header.stamp;
  dock_.header.frame_id = tracking_frame_;

  // If this is the first time we've found dock, take whole pose
  if (!found_dock_) {
    dock_.pose = best_pose;
    // Reset the dock pose filter.
    dock_pose_filter_->reset();
    // Set the filter state to the current pose estimate.
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  } else {
    // Check that pose is not too far from current pose
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.05) {
      ROS_DEBUG_STREAM_NAMED("dock_perception", "Dock pose jumped: " << d);
      return;
    }
  }
  // Filter the pose esitmate.
  if (use_filter_)
      dock_.pose = dock_pose_filter_->filter(best_pose);
  else
      dock_.pose = best_pose;
  dock_stamp_ = scan->header.stamp;
  found_dock_ = true;
  ori_dock_pose_pub_.publish(dock_);
}

void Perception::callbackLaserScan(const sensor_msgs::LaserScanConstPtr &scan){
  // Be lazy about search
  if (!running_) {
    return;
  }
  pcl::PointCloud<pcl::PointXYZI> laserCloud;
  pcl::PointXYZI pt;
  pt.z=0.0;
  for(int ind=0;ind<scan->ranges.size();ind++){
    pt.x = cos(scan->angle_min + ind * scan->angle_increment) * scan->ranges[ind];
    pt.y = sin(scan->angle_min + ind * scan->angle_increment) * scan->ranges[ind];
    if(!std::isfinite(pt.x)||!std::isfinite(pt.y))continue;
    pt.intensity=scan->intensities[ind];
    laserCloud.points.push_back(pt);
  }
  if(laserCloud.points.size()<5)return;
  laserCloud.width=laserCloud.points.size();
  laserCloud.height=1;

  sensor_msgs::PointCloud2 scan_cloud;
  pcl::toROSMsg(laserCloud, scan_cloud);
  try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        "base_link", scan->header.frame_id, ros::Time(0));
    tf2::doTransform(scan_cloud, scan_cloud, transform_scan);   
    scan_cloud.header.frame_id="base_link";
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform origin scan to baselink frame");
    return;
  }
  pcl::fromROSMsg(scan_cloud, laserCloud);
  pcl::PassThrough<pcl::PointXYZI> pass;//创建滤波器对象
  pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
  pass.setInputCloud (laserCloud.makeShared());					
  pass.setFilterFieldName ("x");	
  pass.setFilterLimits (v_range_x_min_2d, v_range_x_max_2d);		
  pass.filter (cloud_filtered);	

  pass.setInputCloud (cloud_filtered.makeShared());			
  pass.setFilterFieldName ("y");	
  pass.setFilterLimits (v_range_y_min_2d, v_range_y_max_2d);		
  pass.filter (cloud_filtered);	

  pass.setInputCloud (cloud_filtered.makeShared());			
  pass.setFilterFieldName ("intensity");		
  pass.setFilterLimits (v_intensity_threshold, 255);	
  pass.filter (cloud_filtered);		

  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZI>());
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;// 欧式聚类
	ece.setInputCloud(cloud_filtered.makeShared());
	ece.setClusterTolerance(0.15);   // 设置近邻的搜索半径
	ece.setMinClusterSize(3);  // 设置最小的聚类点数
	ece.setMaxClusterSize(1000);  // 设置最大的聚类点云点数
	ece.setSearchMethod(tree);   
	std::vector<pcl::PointIndices> cluster_indices;
	ece.extract(cluster_indices);  // 输出点云的聚类别

  if(cluster_indices.size()&&cluster_indices[0].indices.size()){
    int max_id=0,max_size=cluster_indices[0].indices.size();
    for(int i=1;i<cluster_indices.size();i++){
      if(cluster_indices[i].indices.size()>max_size){
        max_size=cluster_indices[i].indices.size();
        max_id=i;
      }
    }
    pcl::copyPointCloud(cloud_filtered,cluster_indices[max_id].indices,cloud_filtered);
    pcl::toROSMsg(cloud_filtered, scan_cloud);
     try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        tracking_frame_, "base_link", ros::Time(0));
    tf2::doTransform(scan_cloud, scan_cloud, transform_scan);   
    scan_cloud.header.frame_id=tracking_frame_;
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform origin scan to tracking frame");
    return;
  }
  pcl::fromROSMsg(scan_cloud, cloud_filtered);
  geometry_msgs::Point pt;
  pt.z=0;
  for(auto&ptXYZI:cloud_filtered.points){
    pt.x=ptXYZI.x;
    pt.y=ptXYZI.y;
    laser_cloud_buffer.push_back(pt);
  }
  laser_cloud_buffer_count+=1;
  }
  if(laser_cloud_buffer_count<v_frame_number_of_reflective_stripe_cloud)
  {
   return;
  }
     
  if(laser_cloud_buffer.size()<10){
    laser_cloud_buffer.clear();
    laser_cloud_buffer.resize(0);
    laser_cloud_buffer_count=0;
    dock_.header.frame_id == "";
    return ;
  }

  // Make sure goal is valid (orientation != 0 0 0 0)
  if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0)) {
    // Lock the dock_
    std::lock_guard<std::mutex> lock(dock_mutex_);

    // If goal is invalid, set to a point directly ahead of robot
    for (size_t i = laser_cloud_buffer.size() / 2; i < laser_cloud_buffer.size(); i++) {
      if (std::isfinite(laser_cloud_buffer[i].x)&&std::isfinite(laser_cloud_buffer[i].y)) {
        dock_.header = scan->header;
        dock_.header.frame_id=tracking_frame_;
        dock_.pose.position.x =laser_cloud_buffer[i].x;
        dock_.pose.position.y = laser_cloud_buffer[i].y;
        dock_.pose.orientation.x = 0.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 1.0;
        ROS_DEBUG_NAMED("dock_perception", "Set initial pose to (%f, %f, %f)",
                        dock_.pose.position.x, dock_.pose.position.y,
                        icp_2d::thetaFromQuaternion(dock_.pose.orientation));
        break;
      }
    }
  }
  // // Make sure goal is in the tracking frame
  if (dock_.header.frame_id != tracking_frame_) {
    std::lock_guard<std::mutex> lock(dock_mutex_);
    try {
      geometry_msgs::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
          tracking_frame_, dock_.header.frame_id, ros::Time(0));
      dock_.header.frame_id = tracking_frame_;
      tf2::doTransform(dock_, dock_, transform);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Couldn't transform dock pose to tracking frame");
      laser_cloud_buffer.clear();
      laser_cloud_buffer.resize(0);
      laser_cloud_buffer_count=0;
      dock_.header.frame_id == "";
      return;
    }
    ROS_WARN_NAMED("dock_perception",
                   "Transformed initial pose to (%f, %f, %f)",
                   dock_.pose.position.x, dock_.pose.position.y,
                   icp_2d::thetaFromQuaternion(dock_.pose.orientation));
  }

  // Sort clusters based on distance to last dock
  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>,
                      CompareCandidates> candidates;
 DockCandidatePtr candidate(new DockCandidate());
 geometry_msgs::PointStamped ptStamped;
 ptStamped.header=dock_.header; // Transform each point into tracking frame
 for(size_t i=0;i<laser_cloud_buffer.size();i++){
    ptStamped.point.x =laser_cloud_buffer[i].x;
    ptStamped.point.y =laser_cloud_buffer[i].y;
    ptStamped.point.z = 0;
    candidate->points.push_back(ptStamped.point);
  }
 
   // Get distance from cloud center to previous pose
  geometry_msgs::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx * dx + dy * dy);

  candidates.push(candidate);
  // Extract ICP pose/fit on best clusters
  DockCandidatePtr best;
  geometry_msgs::Pose best_pose;
  while (!candidates.empty()) {
    geometry_msgs::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    if (score >= 0) {
      best = candidates.top();
      best_pose = pose;
      break;
    } else // Let's see what's wrong with this point cloud.
    {
      if (debug_) {
        DockCandidatePtr not_best = candidates.top();

        // Create point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = scan->header.stamp;
        cloud.header.frame_id = tracking_frame_;
        cloud.width = cloud.height = 0;

        // Allocate space for points
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(not_best->points.size());

        // Fill in points
        sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
        for (size_t i = 0; i < not_best->points.size(); i++) {
          cloud_iter[0] = not_best->points[i].x;
          cloud_iter[1] = not_best->points[i].y;
          cloud_iter[2] = not_best->points[i].z;
          ++cloud_iter;
        }
        debug_points_.publish(cloud);
      }
    }
    candidates.pop();
  }

  // Did we find dock?
  if (!best) {
    ROS_DEBUG_NAMED("dock_perception", "DID NOT FIND THE DOCK");
    // laser_cloud_buffer.clear();
    // laser_cloud_buffer.resize(0);
    // laser_cloud_buffer_count=0;
    int remainSize=int(laser_cloud_buffer.size()/2.0);
    std::vector<geometry_msgs::Point> tempBuffer(laser_cloud_buffer.begin()+remainSize,laser_cloud_buffer.end());
    laser_cloud_buffer.swap(tempBuffer);
    laser_cloud_buffer_count=laser_cloud_buffer.size();
    dock_.header.frame_id == "";
    return;
  }

  ROS_DEBUG_NAMED("dock_perception", "Found the dock.");

  // Update
  if (debug_) {
    // Create point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = tracking_frame_;
    cloud.width = cloud.height = 0;

    // Allocate space for points
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(best->points.size());

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
    for (size_t i = 0; i < best->points.size(); i++) {
      cloud_iter[0] = best->points[i].x;
      cloud_iter[1] = best->points[i].y;
      cloud_iter[2] = best->points[i].z;
      ++cloud_iter;
    }
    debug_points_.publish(cloud);
  }

  // Everything after this modifies the dock_
  std::lock_guard<std::mutex> lock(dock_mutex_);

  // Update stamp
  dock_.header.stamp = scan->header.stamp;
  dock_.header.frame_id = tracking_frame_;

  // If this is the first time we've found dock, take whole pose
  if (!found_dock_) {
    dock_.pose = best_pose;
    // Reset the dock pose filter.
    dock_pose_filter_->reset();
    // Set the filter state to the current pose estimate.
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  } else {
    // Check that pose is not too far from current pose
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.05) {
      ROS_DEBUG_STREAM_NAMED("dock_perception", "Dock pose jumped: " << d);
      int remainSize=int(laser_cloud_buffer.size()/2.0);
      std::vector<geometry_msgs::Point> tempBuffer(laser_cloud_buffer.begin()+remainSize,laser_cloud_buffer.end());
      laser_cloud_buffer.swap(tempBuffer);
      laser_cloud_buffer_count=laser_cloud_buffer.size();
      dock_.header.frame_id == "";
      found_dock_=false;
      return;
    }
  }

  // Filter the pose esitmate.
  if (use_filter_)
      dock_.pose = dock_pose_filter_->filter(best_pose);
  else
      dock_.pose = best_pose;
  dock_stamp_ = scan->header.stamp;
  found_dock_ = true;

  laser_cloud_buffer.clear();
  laser_cloud_buffer.resize(0);
  laser_cloud_buffer_count=0;
 // std::cout<<"callbackPointCloud2 8"<<std::endl; 
  ori_dock_pose_pub_.publish(dock_);
  // std::cout<<"ori_dock_pose_pub_="<<dock_<<std::endl;
  calib_dock_=dock_;
  try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        "base_link", dock_.header.frame_id, ros::Time(0));
    tf2::doTransform(calib_dock_, calib_dock_, transform_scan);   
    calib_dock_.header.frame_id=offset_transform.child_frame_id;
    tf2::doTransform(calib_dock_, calib_dock_, offset_transform);
    calib_dock_.header.frame_id="base_link";
    transform_scan = tf_buffer_->lookupTransform(dock_.header.frame_id,
        "base_link", ros::Time(0));
    tf2::doTransform(calib_dock_, calib_dock_, transform_scan); 
    calib_dock_.header.frame_id=dock_.header.frame_id;
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform dock_ scan to baselink frame");
    return;
  }
  // calib_dock_.header.frame_id=offset_transform.child_frame_id;
  // tf2::doTransform(calib_dock_, calib_dock_, offset_transform);
  // calib_dock_.header.frame_id=dock_.header.frame_id;
  calib_dock_pose_pub_.publish(calib_dock_);

}


void Perception::callbackPointCloud2(const sensor_msgs::PointCloud2ConstPtr& scan) {
  // Be lazy about search
  if (!running_) {
    return;
  }
  sensor_msgs::PointCloud2 scan_cloud=*scan;
  try {
    geometry_msgs::TransformStamped transform_scan;
    // transform_scan = tf_buffer_->lookupTransform(
    //     tracking_frame_, scan->header.frame_id, ros::Time(0));
    transform_scan = tf_buffer_->lookupTransform(
        "base_link", scan->header.frame_id, ros::Time(0));
    tf2::doTransform(scan_cloud, scan_cloud, transform_scan);   
    scan_cloud.header.frame_id="base_link";
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform origin scan to baselink frame");
    return;
  }

  pcl::PointCloud<pcl::PointXYZI> laserCloud;
  pcl::fromROSMsg(scan_cloud, laserCloud);
  pcl::PassThrough<pcl::PointXYZI> pass;//创建滤波器对象
  pcl::PointCloud<pcl::PointXYZI> cloud_filtered;
  pass.setInputCloud (laserCloud.makeShared());			
  pass.setFilterFieldName ("z");		
  pass.setFilterLimits (v_range_z_min,v_range_z_max);		
  pass.filter (cloud_filtered);		

  pass.setInputCloud (cloud_filtered.makeShared());			
  pass.setFilterFieldName ("x");	
  pass.setFilterLimits (v_range_x_min, v_range_x_max);		
  pass.filter (cloud_filtered);	

  pass.setInputCloud (cloud_filtered.makeShared());			
  pass.setFilterFieldName ("y");	
  pass.setFilterLimits (v_range_y_min, v_range_y_max);		
  pass.filter (cloud_filtered);	

  pass.setInputCloud (cloud_filtered.makeShared());			
  pass.setFilterFieldName ("intensity");		
  pass.setFilterLimits (v_intensity_threshold, 255);	
  pass.filter (cloud_filtered);		
  
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new  pcl::search::KdTree<pcl::PointXYZI>());
	pcl::EuclideanClusterExtraction<pcl::PointXYZI> ece;// 欧式聚类
	ece.setInputCloud(cloud_filtered.makeShared());
	ece.setClusterTolerance(0.15);   // 设置近邻的搜索半径
	ece.setMinClusterSize(3);  // 设置最小的聚类点数
	ece.setMaxClusterSize(1000);  // 设置最大的聚类点云点数
	ece.setSearchMethod(tree);   
	std::vector<pcl::PointIndices> cluster_indices;
	ece.extract(cluster_indices);  // 输出点云的聚类别

  if(cluster_indices.size()&&cluster_indices[0].indices.size()){
    int max_id=0,max_size=cluster_indices[0].indices.size();
    for(int i=1;i<cluster_indices.size();i++){
      if(cluster_indices[i].indices.size()>max_size){
        max_size=cluster_indices[i].indices.size();
        max_id=i;
      }
    }
    pcl::copyPointCloud(cloud_filtered,cluster_indices[max_id].indices,cloud_filtered);
    pcl::toROSMsg(cloud_filtered, scan_cloud);
     try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        tracking_frame_, "base_link", ros::Time(0));
    tf2::doTransform(scan_cloud, scan_cloud, transform_scan);   
    scan_cloud.header.frame_id=tracking_frame_;
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform origin scan to tracking frame");
    return;
  }
  pcl::fromROSMsg(scan_cloud, cloud_filtered);

    geometry_msgs::Point pt;
    pt.z=0;
    for(auto&ptXYZI:cloud_filtered.points){
      pt.x=ptXYZI.x;
      pt.y=ptXYZI.y;
      laser_cloud_buffer.push_back(pt);
    }
    laser_cloud_buffer_count+=1;
  }
  if(laser_cloud_buffer_count<v_frame_number_of_reflective_stripe_cloud)
  {
   return;
  }
     
  if(laser_cloud_buffer.size()<10){
    laser_cloud_buffer.clear();
    laser_cloud_buffer.resize(0);
    laser_cloud_buffer_count=0;
    dock_.header.frame_id == "";
    return ;
  }


  // Make sure goal is valid (orientation != 0 0 0 0)
  if (dock_.header.frame_id == "" ||
      (dock_.pose.orientation.z == 0.0 && dock_.pose.orientation.w == 0.0)) {
    // Lock the dock_
    std::lock_guard<std::mutex> lock(dock_mutex_);

    // If goal is invalid, set to a point directly ahead of robot
    for (size_t i = laser_cloud_buffer.size() / 2; i < laser_cloud_buffer.size(); i++) {
      if (std::isfinite(laser_cloud_buffer[i].x)&&std::isfinite(laser_cloud_buffer[i].y)) {
        dock_.header = scan->header;
        dock_.header.frame_id=tracking_frame_;
        dock_.pose.position.x =laser_cloud_buffer[i].x;
        dock_.pose.position.y = laser_cloud_buffer[i].y;
        dock_.pose.orientation.x = 0.0;
        dock_.pose.orientation.y = 0.0;
        dock_.pose.orientation.z = 0.0;
        dock_.pose.orientation.w = 1.0;
        ROS_DEBUG_NAMED("dock_perception", "Set initial pose to (%f, %f, %f)",
                        dock_.pose.position.x, dock_.pose.position.y,
                        icp_2d::thetaFromQuaternion(dock_.pose.orientation));
        break;
      }
    }
  }
  // // Make sure goal is in the tracking frame
  if (dock_.header.frame_id != tracking_frame_) {
    std::lock_guard<std::mutex> lock(dock_mutex_);
    try {
      geometry_msgs::TransformStamped transform;
      transform = tf_buffer_->lookupTransform(
          tracking_frame_, dock_.header.frame_id, ros::Time(0));
      dock_.header.frame_id = tracking_frame_;
      tf2::doTransform(dock_, dock_, transform);
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM_THROTTLE(
          1.0, "Couldn't transform dock pose to tracking frame");
      laser_cloud_buffer.clear();
      laser_cloud_buffer.resize(0);
      laser_cloud_buffer_count=0;
      dock_.header.frame_id == "";
      return;
    }
    ROS_WARN_NAMED("dock_perception",
                   "Transformed initial pose to (%f, %f, %f)",
                   dock_.pose.position.x, dock_.pose.position.y,
                   icp_2d::thetaFromQuaternion(dock_.pose.orientation));
  }

  // Sort clusters based on distance to last dock
  std::priority_queue<DockCandidatePtr, std::vector<DockCandidatePtr>,
                      CompareCandidates> candidates;
 DockCandidatePtr candidate(new DockCandidate());
 geometry_msgs::PointStamped ptStamped;
 ptStamped.header=dock_.header; // Transform each point into tracking frame
 for(size_t i=0;i<laser_cloud_buffer.size();i++){
    ptStamped.point.x =laser_cloud_buffer[i].x;
    ptStamped.point.y =laser_cloud_buffer[i].y;
    ptStamped.point.z = 0;
    // try {
    //   tf_buffer_->transform(ptStamped, ptStamped, tracking_frame_, ros::Duration(0.1));
    // } catch (tf2::TransformException const &ex) {
    //   ROS_WARN_STREAM("couldn't transform laser point to tracking frame");
    // }
    candidate->points.push_back(ptStamped.point);
  }
 
   // Get distance from cloud center to previous pose
  geometry_msgs::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx * dx + dy * dy);

  candidates.push(candidate);
  // Extract ICP pose/fit on best clusters
  DockCandidatePtr best;
  geometry_msgs::Pose best_pose;
  while (!candidates.empty()) {
    geometry_msgs::Pose pose = dock_.pose;
    double score = fit(candidates.top(), pose);
    if (score >= 0) {
      best = candidates.top();
      best_pose = pose;
      break;
    } else // Let's see what's wrong with this point cloud.
    {
      if (debug_) {
        DockCandidatePtr not_best = candidates.top();

        // Create point cloud
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp = scan->header.stamp;
        cloud.header.frame_id = tracking_frame_;
        cloud.width = cloud.height = 0;

        // Allocate space for points
        sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
        cloud_mod.setPointCloud2FieldsByString(1, "xyz");
        cloud_mod.resize(not_best->points.size());

        // Fill in points
        sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
        for (size_t i = 0; i < not_best->points.size(); i++) {
          cloud_iter[0] = not_best->points[i].x;
          cloud_iter[1] = not_best->points[i].y;
          cloud_iter[2] = not_best->points[i].z;
          ++cloud_iter;
        }
        debug_points_.publish(cloud);
      }
    }
    candidates.pop();
  }

  // Did we find dock?
  if (!best) {
    ROS_DEBUG_NAMED("dock_perception", "DID NOT FIND THE DOCK");
    // laser_cloud_buffer.clear();
    // laser_cloud_buffer.resize(0);
    // laser_cloud_buffer_count=0;
    int remainSize=int(laser_cloud_buffer.size()/2.0);
    std::vector<geometry_msgs::Point> tempBuffer(laser_cloud_buffer.begin()+remainSize,laser_cloud_buffer.end());
    laser_cloud_buffer.swap(tempBuffer);
    laser_cloud_buffer_count=laser_cloud_buffer.size();
    dock_.header.frame_id == "";
    return;
  }

  ROS_DEBUG_NAMED("dock_perception", "Found the dock.");

  // Update
  if (debug_) {
    // Create point cloud
    sensor_msgs::PointCloud2 cloud;
    cloud.header.stamp = scan->header.stamp;
    cloud.header.frame_id = tracking_frame_;
    cloud.width = cloud.height = 0;

    // Allocate space for points
    sensor_msgs::PointCloud2Modifier cloud_mod(cloud);
    cloud_mod.setPointCloud2FieldsByString(1, "xyz");
    cloud_mod.resize(best->points.size());

    // Fill in points
    sensor_msgs::PointCloud2Iterator<float> cloud_iter(cloud, "x");
    for (size_t i = 0; i < best->points.size(); i++) {
      cloud_iter[0] = best->points[i].x;
      cloud_iter[1] = best->points[i].y;
      cloud_iter[2] = best->points[i].z;
      ++cloud_iter;
    }
    debug_points_.publish(cloud);
  }

  // Everything after this modifies the dock_
  std::lock_guard<std::mutex> lock(dock_mutex_);

  // Update stamp
  dock_.header.stamp = scan->header.stamp;
  dock_.header.frame_id = tracking_frame_;

  // If this is the first time we've found dock, take whole pose
  if (!found_dock_) {
    dock_.pose = best_pose;
    // Reset the dock pose filter.
    dock_pose_filter_->reset();
    // Set the filter state to the current pose estimate.
    dock_pose_filter_->setFilterState(dock_.pose, dock_.pose);
  } else {
    // Check that pose is not too far from current pose
    double d = getPoseDistance(dock_.pose, best_pose);
    if (d > 0.05) {
      ROS_DEBUG_STREAM_NAMED("dock_perception", "Dock pose jumped: " << d);
      int remainSize=int(laser_cloud_buffer.size()/2.0);
      std::vector<geometry_msgs::Point> tempBuffer(laser_cloud_buffer.begin()+remainSize,laser_cloud_buffer.end());
      laser_cloud_buffer.swap(tempBuffer);
      laser_cloud_buffer_count=laser_cloud_buffer.size();
      dock_.header.frame_id == "";
      found_dock_=false;
      return;
    }
  }

  // Filter the pose esitmate.
  if (use_filter_)
      dock_.pose = dock_pose_filter_->filter(best_pose);
  else
      dock_.pose = best_pose;
  dock_stamp_ = scan->header.stamp;
  found_dock_ = true;

  laser_cloud_buffer.clear();
  laser_cloud_buffer.resize(0);
  laser_cloud_buffer_count=0;
 // std::cout<<"callbackPointCloud2 8"<<std::endl; 
  ori_dock_pose_pub_.publish(dock_);
  // std::cout<<"ori_dock_pose_pub_="<<dock_<<std::endl;
  calib_dock_=dock_;
  try {
    geometry_msgs::TransformStamped transform_scan;
    transform_scan = tf_buffer_->lookupTransform(
        "base_link", dock_.header.frame_id, ros::Time(0));
    tf2::doTransform(calib_dock_, calib_dock_, transform_scan);   
    calib_dock_.header.frame_id=offset_transform.child_frame_id;
    tf2::doTransform(calib_dock_, calib_dock_, offset_transform);
    calib_dock_.header.frame_id="base_link";
    transform_scan = tf_buffer_->lookupTransform(dock_.header.frame_id,
        "base_link", ros::Time(0));
    tf2::doTransform(calib_dock_, calib_dock_, transform_scan); 
    calib_dock_.header.frame_id=dock_.header.frame_id;
  } catch (tf2::TransformException const &ex) {
    ROS_WARN_STREAM_THROTTLE(
        1.0, "Couldn't transform dock_ scan to baselink frame");
    return;
  }
  // calib_dock_.header.frame_id=offset_transform.child_frame_id;
  // tf2::doTransform(calib_dock_, calib_dock_, offset_transform);
  // calib_dock_.header.frame_id=dock_.header.frame_id;
  calib_dock_pose_pub_.publish(calib_dock_);

}


DockCandidatePtr Perception::extract(laser_processor::SampleSet *cluster) {
  DockCandidatePtr candidate(new DockCandidate());

  // TODO: check if this is correct
  // Transform each point into tracking frame
  size_t i = 0;
  for (laser_processor::SampleSet::iterator p = cluster->begin();
       p != cluster->end(); p++, i++) {
    geometry_msgs::PointStamped pt;
    pt.header = cluster->header;
    pt.point.x = (*p)->x;
    pt.point.y = (*p)->y;
    pt.point.z = 0;

    try {
      tf_buffer_->transform(pt, pt, tracking_frame_, ros::Duration(0.1));
    } catch (tf2::TransformException const &ex) {
      ROS_WARN_STREAM("couldn't transform laser point to tracking frame");
    }
    candidate->points.push_back(pt.point);
  }

  // Get distance from cloud center to previous pose
  geometry_msgs::Point centroid = icp_2d::getCentroid(candidate->points);
  double dx = centroid.x - dock_.pose.position.x;
  double dy = centroid.y - dock_.pose.position.y;
  candidate->dist = (dx * dx + dy * dy);

  return candidate;
}

double Perception::fit(const DockCandidatePtr &candidate,
                       geometry_msgs::Pose &pose) {
  // Setup initial pose
  geometry_msgs::Transform transform;
  transform.translation.x = pose.position.x;
  transform.translation.y = pose.position.y;
  transform.rotation = pose.orientation;

  // Initial yaw. Presumably the initial goal orientation estimate.
  tf2::Quaternion q;
  tf2::Quaternion init_pose, cand_pose;
  tf2::convert(pose.orientation, init_pose);
  if (!isValid(init_pose)) {
    ROS_ERROR_STREAM_NAMED("perception",
                           "Initial dock orientation estimate is invalid.");
    ROS_DEBUG_STREAM_NAMED(
        "perception", "Quaternion magnitude is "
                          << init_pose.length() << " Quaternion is ["
                          << init_pose.x() << ", " << init_pose.y() << ", "
                          << init_pose.z() << ", " << init_pose.w() << "]");
    return -1.0;
  }

  // ICP the dock
  double fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform,
                                    max_icp_iterations_);
  tf2::convert(transform.rotation, cand_pose);
  if (!isValid(cand_pose)) {
    ROS_WARN_STREAM_NAMED("perception",
                          "Dock candidate orientation estimate is invalid.");
    ROS_DEBUG_STREAM_NAMED(
        "perception", "Quaternion magnitude is "
                          << cand_pose.length() << " Quaternion is ["
                          << cand_pose.x() << ", " << cand_pose.y() << ", "
                          << cand_pose.z() << ", " << cand_pose.w() << "]");
  }

  // If the dock orientation seems flipped, flip it.
  // Check it by finding the relative roation between the two quaternions.
  if (fabs((tf2::getYaw(tf2::inverse(cand_pose) * init_pose))) >
      3.1415 * (2.0 / 3.0)) {
    q.setRPY(0, 0, 3.1415 + tf2::getYaw(transform.rotation));
    transform.rotation = tf2::toMsg(q);
  }

  if (fitness >= 0.0) {
    // Initialize the number of times we retry if the fitness is bad.
    double retry = 5;
    // If the fitness is hosed or the angle is borked, try again.
    tf2::convert(transform.rotation, cand_pose);
    while (retry-- && (fitness > max_alignment_error_ ||
                       fabs(tf2::getYaw(tf2::inverse(cand_pose) * init_pose)) >
                           3.1415 / 4.0)) {
      // Try one more time.

      // Perturb the pose to try to get it out of the local minima.
      transform.translation.x +=
          retry * (0.75 / 100.0) * static_cast<double>((rand() % 200) - 100);
      transform.translation.y +=
          retry * (0.75 / 100.0) * static_cast<double>((rand() % 200) - 100);
      q.setRPY(0, 0, retry * (0.28 / 100.0) * double((rand() % 200) - 100) +
                         tf2::getYaw(transform.rotation));
      tf2::convert(q, transform.rotation);

      // Rerun ICP.
      fitness = icp_2d::alignICP(ideal_cloud_, candidate->points, transform,
                                 max_icp_iterations_);

      // If the dock orientation seems flipped, flip it.
      //tf2::convert(transform.rotation, cand_pose);
      //if (fabs(tf2::getYaw(tf2::inverse(cand_pose) * init_pose)) >
      //    3.1415 * (2.0 / 3.0)) {
      //  q.setRPY(0, 0, 3.1415 + tf2::getYaw(transform.rotation));
      //  transform.rotation = tf2::toMsg(q);
      //}
    }

    // If the dock orientation is still really borked, fail.
    tf2::convert(transform.rotation, cand_pose);
    if (!isValid(cand_pose)) {
      ROS_ERROR_STREAM_NAMED("perception",
                             "Dock candidate orientation estimate is invalid.");
      ROS_DEBUG_STREAM_NAMED(
          "perception", "Quaternion magnitude is "
                            << cand_pose.length() << " Orientation is ["
                            << cand_pose.x() << ", " << cand_pose.y() << ", "
                            << cand_pose.z() << ", " << cand_pose.w() << "]");
      return -1.0;
    }
    if (fabs(tf2::getYaw(tf2::inverse(cand_pose) * init_pose)) > 3.1415 / 2.0) {
      fitness = -1.0;
    }

    // Check that fitness is good enough
    if (!found_dock_ && fabs(fitness) > max_alignment_error_) {
      // If not, signal no fit
      fitness = -1.0;
    }

    // If width of candidate is smaller than the width of dock
    // then the whole dock is not visible...
    // TODO(Max): replace this magic number with DockShape attribute
    if (candidate->width() < 0.075) {
      // ... and heading is unreliable when close to dock
      ROS_DEBUG_STREAM_NAMED("perception",
                             "Dock candidate width is unreliable.");
      transform.rotation = pose.orientation;
      fitness = 0.001234;
      // Probably can use a different algorithm here, if necessary, which it
      // might not be.
    }

    // Transform ideal cloud, and store for visualization
    candidate->points = icp_2d::transform(
        ideal_cloud_, transform.translation.x, transform.translation.y,
        icp_2d::thetaFromQuaternion(transform.rotation));

    // Get pose
    pose.position.x = transform.translation.x;
    pose.position.y = transform.translation.y;
    pose.position.z = transform.translation.z;
    pose.orientation = transform.rotation;
    return fitness;
  }

  // Signal no fit
  ROS_DEBUG_NAMED("dock_perception", "Did not converge");
  ROS_INFO_STREAM("Did not converge.");
  return -1.0;
}

bool Perception::isValid(const tf2::Quaternion &q) {
  return 1e-3 >= fabs(1.0 - q.length());
}

 bool Perception::calibrate(navit_msgs::PerceptionCalibration::Request&req,
                 navit_msgs::PerceptionCalibration::Response&res)
      {
       if(!found_dock_)
          return true;
       if(req.state==1){
          geometry_msgs::PoseStamped pose_base_link=dock_;
          try {
            geometry_msgs::TransformStamped transform_scan;
            transform_scan = tf_buffer_->lookupTransform(
                "base_link", dock_.header.frame_id, ros::Time(0));
            tf2::doTransform(pose_base_link, pose_base_link, transform_scan);   
            pose_base_link.header.frame_id="base_link";
          } catch (tf2::TransformException const &ex) {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "Couldn't transform dock_ scan to baselink frame");
            return true;
          }
          // geometry_msgs::Point position1= dock_.pose.position;
          // geometry_msgs::Quaternion orientation1=dock_.pose.orientation;
          geometry_msgs::Point position1= pose_base_link.pose.position;
          geometry_msgs::Quaternion orientation1=pose_base_link.pose.orientation;
          geometry_msgs::Point position2= req.ground_truth_dock_pose.pose.position;
          geometry_msgs::Quaternion orientation2=req.ground_truth_dock_pose.pose.orientation;
          tf2::Vector3 tf_translation1(position1.x,position1.y,position1.z);
          tf2::Quaternion tf_rotation1(orientation1.x,orientation1.y,orientation1.z,orientation1.w);
          tf2::Transform tf_transform1(tf_rotation1,tf_translation1);
          tf2::Vector3 tf_translation2(position2.x,position2.y,position2.z);
          tf2::Quaternion tf_rotation2(orientation2.x,orientation2.y,orientation2.z,orientation2.w);
          tf2::Transform tf_transform2(tf_rotation2,tf_translation2);
          tf2::Transform tf_translation=tf_transform2*tf_transform1.inverse();
          res.calibration_tf_msg.header.frame_id="calibrated_charge_port_link";  
          res.calibration_tf_msg.header.stamp=dock_.header.stamp;
          res.calibration_tf_msg.child_frame_id="charge_port_link";
          tf2::convert(tf_translation,res.calibration_tf_msg.transform);
          offset_transform=res.calibration_tf_msg;

          std::string param_file;
          if(ros::param::get("~approach_dock_path",param_file)){
          YAML::Node config = YAML::LoadFile(param_file);
          config["2d_lidar"]["offset_translation_x"] = offset_transform.transform.translation.x;
          config["2d_lidar"]["offset_translation_y"] = offset_transform.transform.translation.y;
          config["2d_lidar"]["offset_rotation_x"] = offset_transform.transform.rotation.x;
          config["2d_lidar"]["offset_rotation_y"] = offset_transform.transform.rotation.y;
          config["2d_lidar"]["offset_rotation_z"] = offset_transform.transform.rotation.z;
          config["2d_lidar"]["offset_rotation_w"] = offset_transform.transform.rotation.w;
          std::ofstream fout(param_file);
          fout << config;
          fout.close();}
          else{
            std::cout<<"failed to get approach_dock_path"<<std::endl;
            ROS_ERROR("Failed to get approach_dock_path");
          }
      }
      res.calibration_tf_msg=offset_transform;
      res.percep_dock_pose=dock_;
      res.calib_truth_dock_pose=dock_;
      try {
        geometry_msgs::TransformStamped transform_scan;
        transform_scan = tf_buffer_->lookupTransform(
            "base_link", dock_.header.frame_id, ros::Time(0));
        tf2::doTransform(res.calib_truth_dock_pose, res.calib_truth_dock_pose, transform_scan);   
        res.calib_truth_dock_pose.header.frame_id=offset_transform.child_frame_id;
        tf2::doTransform(res.calib_truth_dock_pose, res.calib_truth_dock_pose, offset_transform);
        res.calib_truth_dock_pose.header.frame_id="base_link";
        transform_scan = tf_buffer_->lookupTransform(dock_.header.frame_id,
            "base_link", ros::Time(0));
        tf2::doTransform(res.calib_truth_dock_pose, res.calib_truth_dock_pose, transform_scan); 
        res.calib_truth_dock_pose.header.frame_id=dock_.header.frame_id;
      } catch (tf2::TransformException const &ex) {
        ROS_WARN_STREAM_THROTTLE(
            1.0, "Couldn't transform dock_  to baselink frame");
        return true;
      }
        
      return true;
      }
}
