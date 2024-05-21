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

#include <pointcloud_to_laserscan/pointcloud_to_laserscan_nodelet.h>
#include <sensor_msgs/LaserScan.h>
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

namespace pointcloud_to_laserscan
{

  PointCloudToLaserScanNodelet::PointCloudToLaserScanNodelet() {}

  void PointCloudToLaserScanNodelet::onInit()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    private_nh_ = getPrivateNodeHandle();

    private_nh_.param<std::string>("target_frame", target_frame_, "");
    private_nh_.param<double>("transform_tolerance", tolerance_, 0.01);
    private_nh_.param<double>("min_height", min_height_, 0.0);
    private_nh_.param<double>("max_height", max_height_, 1.0);

    private_nh_.param<double>("angle_min", angle_min_, -M_PI / 2.0);
    private_nh_.param<double>("angle_max", angle_max_, M_PI / 2.0);
    private_nh_.param<double>("angle_increment", angle_increment_, M_PI / 360.0);
    private_nh_.param<double>("scan_time", scan_time_, 1.0 / 30.0);
    private_nh_.param<double>("range_min", range_min_, 0.45);
    private_nh_.param<double>("range_max", range_max_, 4.0);
    private_nh_.param<double>("range_x_min", range_x_min_, -0.5f);
    private_nh_.param<double>("range_x_max", range_x_max_, 1.5f);
    private_nh_.param<double>("range_y_min", range_y_min_, -0.7f);
    private_nh_.param<double>("range_y_max", range_y_max_, 0.7f);
    private_nh_.param<double>("range_z_min", range_z_min_, -5.0f);
    private_nh_.param<double>("range_z_max", range_z_max_, 5.0f);
    private_nh_.param<bool>("use_cliff_detect", use_cliff_detect_, false);
    private_nh_.param<int>("cliff_detect_min_pts", cliff_detect_min_pts_, 5);
    private_nh_.param<double>("cliff_detect_block_length", cliff_detect_block_length_, 0.1f);
    private_nh_.param<double>("cliff_detect_x_min", cliff_detect_x_min_, 1.8f);
    private_nh_.param<double>("cliff_detect_x_max", cliff_detect_x_max_, 3.0f);
    private_nh_.param<double>("cliff_detect_y_min", cliff_detect_y_min_, -0.5f);
    private_nh_.param<double>("cliff_detect_y_max", cliff_detect_y_max_, 0.5f);
    private_nh_.param<bool>("use_brush_range_filter", use_brush_range_filter_, false);
    private_nh_.param<double>("brush_x_min", brush_x_min_, 1.2f);
    private_nh_.param<double>("brush_x_max", brush_x_max_, 1.6f);
    private_nh_.param<double>("brush_y_min", brush_y_min_, -0.7f);
    private_nh_.param<double>("brush_y_max", brush_y_max_, 0.7f);
    private_nh_.param<double>("brush_z_min", brush_z_min_, -5.0f);
    private_nh_.param<double>("brush_z_max", brush_z_max_, 0.3f);
    private_nh_.param<bool>("use_ground_filter", use_ground_filter_, false);
    private_nh_.param<bool>("is_middle_lidar", is_middle_lidar_, false);
    private_nh_.param<double>("dynamic_height_range_start", dynamic_height_range_start_, 2.5f);
    private_nh_.param<double>("dynamic_height_gap_per_half_mile", dynamic_height_gap_per_half_mile_, 0.02f);
    private_nh_.param<bool>("use_radiu_filter", use_radiu_filter_, false);
    private_nh_.param<double>("radiu_threshold", radiu_threshold_, 0.1);
    private_nh_.param<int>("min_neighbors", min_neighbors_, 3);
    private_nh_.param<bool>("pub_ground_and_nonground", pub_ground_and_nonground_, false);
    

    int concurrency_level;
    private_nh_.param<int>("concurrency_level", concurrency_level, 1);
    private_nh_.param<bool>("use_inf", use_inf_, true);

	  PatchworkppGroundSeg_.reset(new PatchWorkpp(&private_nh_));

    setupDynamicReconfigure(private_nh_);

    //Check if explicitly single threaded, otherwise, let nodelet manager dictate thread pool size
    if (concurrency_level == 1)
    {
      nh_ = getNodeHandle();
    }
    else
    {
      nh_ = getMTNodeHandle();
    }

    // Only queue one pointcloud per running thread
    if (concurrency_level > 0)
    {
      input_queue_size_ = concurrency_level;
    }
    else
    {
      input_queue_size_ = boost::thread::hardware_concurrency();
    }

    // if pointcloud target frame specified, we need to filter by transform availability
    if (!target_frame_.empty())
    {
      tf2_.reset(new tf2_ros::Buffer());
      tf2_listener_.reset(new tf2_ros::TransformListener(*tf2_));
      message_filter_.reset(new MessageFilter(sub_, *tf2_, target_frame_, input_queue_size_, nh_));
      message_filter_->registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
      message_filter_->registerFailureCallback(boost::bind(&PointCloudToLaserScanNodelet::failureCb, this, _1, _2));
    }
    else // otherwise setup direct subscription
    {
      sub_.registerCallback(boost::bind(&PointCloudToLaserScanNodelet::cloudCb, this, _1));
    }

    pub_ = nh_.advertise<sensor_msgs::LaserScan>("scan", 10,
                                                 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
                                                 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
	pub_ground_ = nh_.advertise<sensor_msgs::PointCloud2>("ground", 10,
												 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
												 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
	pub_nonground_ = nh_.advertise<sensor_msgs::PointCloud2>("nonground", 10,
												 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
												 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
	pub_no_footprint_ = nh_.advertise<sensor_msgs::PointCloud2>("no_footprint_cloud", 10,
												 boost::bind(&PointCloudToLaserScanNodelet::connectCb, this),
												 boost::bind(&PointCloudToLaserScanNodelet::disconnectCb, this));
  }
  void PointCloudToLaserScanNodelet::setupDynamicReconfigure(ros::NodeHandle& nh)
  {
    dsrv_ = new dynamic_reconfigure::Server<pointcloud_to_laserscan::RobotFootprintConfig>(nh);
    dynamic_reconfigure::Server<pointcloud_to_laserscan::RobotFootprintConfig>::CallbackType cb =
        [this](auto& config, auto level){ reconfigureCB(config, level); };
    dsrv_->setCallback(cb);
  }

  void PointCloudToLaserScanNodelet::reconfigureCB(pointcloud_to_laserscan::RobotFootprintConfig &config, uint32_t level)
  {
    range_x_min_ = config.range_x_min;
    range_x_max_ = config.range_x_max;
    range_y_min_ = config.range_y_min;
    range_y_max_ = config.range_y_max;
  }

  void PointCloudToLaserScanNodelet::connectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() > 0 && sub_.getSubscriber().getNumPublishers() == 0)
    {
      NODELET_INFO("Got a subscriber to scan, starting subscriber to pointcloud");
      sub_.subscribe(nh_, "cloud_in", input_queue_size_);
    }
  }

  void PointCloudToLaserScanNodelet::disconnectCb()
  {
    boost::mutex::scoped_lock lock(connect_mutex_);
    if (pub_.getNumSubscribers() == 0)
    {
      NODELET_INFO("No subscibers to scan, shutting down subscriber to pointcloud");
      sub_.unsubscribe();
    }
  }

  void PointCloudToLaserScanNodelet::failureCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg,
                                               tf2_ros::filter_failure_reasons::FilterFailureReason reason)
  {
    NODELET_WARN_STREAM_THROTTLE(1.0, "Can't transform pointcloud from frame " << cloud_msg->header.frame_id << " to "
        << message_filter_->getTargetFramesString());
  }

  void PointCloudToLaserScanNodelet::cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
  {

    //build laserscan output
    sensor_msgs::LaserScan output;
    output.header = cloud_msg->header;
    if (!target_frame_.empty())
    {
      output.header.frame_id = target_frame_;
    }

    output.angle_min = angle_min_;
    output.angle_max = angle_max_;
    output.angle_increment = angle_increment_;
    output.time_increment = 0.0;
    output.scan_time = scan_time_;
    output.range_min = range_min_;
    output.range_max = range_max_;

    //determine amount of rays to create
    uint32_t ranges_size = std::ceil((output.angle_max - output.angle_min) / output.angle_increment);

    //determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_)
    {
      output.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    }
    else
    {
      output.ranges.assign(ranges_size, output.range_max + 1.0);
    }

    // sensor_msgs::PointCloud2ConstPtr cloud_out;
    sensor_msgs::PointCloud2Ptr cloud;

    // Transform cloud if necessary
    if (!(output.header.frame_id == cloud_msg->header.frame_id))
    {
      try
      {
        cloud.reset(new sensor_msgs::PointCloud2);
        tf2_->transform(*cloud_msg, *cloud, target_frame_, ros::Duration(tolerance_));
        // cloud_out = cloud;
      }
      catch (tf2::TransformException ex)
      {
        NODELET_ERROR_STREAM("Transform failure: " << ex.what());
        return;
      }
    }
    else
    {
      // cloud_out = cloud_msg;
      *cloud=*cloud_msg;
    }
   //filter cloud by range
   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI>::Ptr front_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
   pcl::PointCloud<pcl::PointXYZI>::Ptr back_cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>);
   {
    pcl::fromROSMsg(*cloud, *cloud_filtered);

    if(is_middle_lidar_){
      std::vector<int>idx1;
      int cloud_sz=cloud_filtered->points.size();
      idx1.reserve(cloud_sz);
      float x_,y_,z_;
      float ext_min_x_=range_x_min_-0.15,ext_max_x_=range_x_max_+0.15;
      float ext_min_y_=range_y_min_ -0.15,ext_max_y_=range_y_max_+0.15;
      for(int i=0;i<cloud_sz;i++){
         x_=cloud_filtered->points[i].x;
         y_=cloud_filtered->points[i].y;

        //  if(x_>ext_min_x_&&x_<ext_max_x_&&y_>ext_min_y_&&y_<ext_max_y_)
         if(x_<=ext_min_x_||x_>=ext_max_x_||y_<=ext_min_y_||y_>=ext_max_y_)
          idx1.push_back(i);
      }
      pcl::copyPointCloud(*cloud_filtered,idx1,*cloud_filtered);
    }
    else if(use_brush_range_filter_){
      std::vector<int>idx2,idx3,idx4;
      int cloud_sz=cloud_filtered->points.size();
      idx2.reserve(cloud_sz);
      idx3.reserve(cloud_sz); idx4.reserve(cloud_sz);
      float x_,y_,z_;
      float ext_min_x_=range_x_min_-0.15,ext_max_x_=brush_x_max_;
      float ext_min_y_=brush_y_min_,ext_max_y_=brush_y_max_;
      // float mid_min_x_=range_x_min_-0.15,mid_max_x_=range_x_max_+0.15;
      // float mid_min_y_=range_y_min_-0.15,mid_max_y_=range_y_max_+0.15;
      float lh_min_x_=range_x_min_-0.05,lh_max_x_=range_x_max_+0.05 ;
      // float lh_min_y_=range_y_min_-0.08,lh_max_y_=range_y_max_+0.08;
      float body_min_x=range_x_min_+0.08,body_max_x=range_x_max_-0.08;
      for(int i=0;i<cloud_sz;i++){
         x_=cloud_filtered->points[i].x;
         y_=cloud_filtered->points[i].y;
         if(x_>ext_min_x_&&x_<ext_max_x_&&y_>ext_min_y_&&y_<ext_max_y_){
            if(x_>lh_min_x_&&x_<lh_max_x_&&y_>range_y_min_&&y_<range_y_max_)continue;
            z_=cloud_filtered->points[i].z;
            // if(z_>0.6&&x_>mid_min_x_&&x_<mid_max_x_&&y_>mid_min_y_&&y_<mid_max_y_)continue;
            // if(z_>0.2&&z_<0.4&&x_>mid_min_x_&&x_<mid_max_x_&&y_>mid_min_y_&&y_<mid_max_y_)continue;
            // if(z_>brush_z_min_&&z_<brush_z_max_&&x_>brush_x_min_&&x_<brush_x_max_&&y_>brush_y_min_&&y_<brush_y_max_)continue;
            if((z_>0.32&&z_<0.85)||(x_>body_min_x&&x_<body_max_x))
              idx2.push_back(i);
         }
         else {
          // idx1.push_back(i);
          if(x_>=ext_max_x_)
           idx3.push_back(i);
          else 
           idx4.push_back(i);
         }
      }
      pcl::copyPointCloud(*cloud_filtered,idx3,*front_cloud_filtered);
      idx4.insert(idx4.end(),idx2.begin(),idx2.end());
      pcl::copyPointCloud(*cloud_filtered,idx4,*back_cloud_filtered);
      idx4.insert(idx4.end(),idx3.begin(),idx3.end());     
      pcl::copyPointCloud(*cloud_filtered,idx4,*cloud_filtered);

    }
    else{
      std::vector<int>idx1;
      int cloud_sz=cloud_filtered->points.size();
      idx1.reserve(cloud_sz);
      float x_,y_,z_;
      float ext_min_x_=range_x_min_-0.15,ext_max_x_=range_x_max_+0.15;
      float ext_min_y_=brush_y_min_,ext_max_y_=brush_y_max_;
      float lh_min_x_=range_x_min_-0.05,lh_max_x_=range_x_max_+0.05 ;
      float body_min_x=range_x_min_+0.08,body_max_x=range_x_max_-0.08;
       for(int i=0;i<cloud_sz;i++){
         x_=cloud_filtered->points[i].x;
         y_=cloud_filtered->points[i].y;
         if(std::isnan(x_) || std::isnan(y_))
            continue;
         if(x_>ext_min_x_&&x_<ext_max_x_&&y_>ext_min_y_&&y_<ext_max_y_){
            if(x_>lh_min_x_&&x_<lh_max_x_&&y_>range_y_min_&&y_<range_y_max_)continue;
            z_=cloud_filtered->points[i].z;
            if((z_>0.32&&z_<0.85)||(x_>body_min_x&&x_<body_max_x))
              idx1.push_back(i);
         }
         else
            idx1.push_back(i);
       }
      pcl::copyPointCloud(*cloud_filtered,idx1,*cloud_filtered);
    }
    pcl::toROSMsg(*cloud_filtered, *cloud);
	  pub_no_footprint_.publish(cloud);
   // cloud_out = cloud;
   }

if(use_ground_filter_){
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_current(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_ground(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_no_ground(new pcl::PointCloud<pcl::PointXYZI>);
    double time_taken = 0.0;

    // pcl::fromROSMsg(*cloud, *pc_current);
    pc_current=front_cloud_filtered;

    try {
      PatchworkppGroundSeg_->estimate_ground(pc_current, pc_ground, pc_no_ground, time_taken);
      if(use_cliff_detect_){
        int block_size=int(1+(cliff_detect_x_max_-cliff_detect_x_min_)/cliff_detect_block_length_);
        std::vector<std::vector<float>> ground_blocks_z(block_size,std::vector<float>{}),nonground_blocks_z(block_size,std::vector<float>{});
        int ground_cloud_size=pc_ground->points.size();
        for(int i=0;i<ground_cloud_size;i++){
            auto& pt=pc_ground->points[i];
            if(pt.x<cliff_detect_x_min_||pt.x>cliff_detect_x_max_||pt.y<cliff_detect_y_min_||pt.y>cliff_detect_y_max_)continue;
            int block_id=int((pt.x-cliff_detect_x_min_)/cliff_detect_block_length_);
            ground_blocks_z[block_id].push_back(pt.z);
        }
        int nonground_cloud_size=pc_no_ground->points.size();
        for(int i=0;i<nonground_cloud_size;i++){
            auto& pt=pc_no_ground->points[i];
            if(pt.x<cliff_detect_x_min_||pt.x>cliff_detect_x_max_||pt.y<cliff_detect_y_min_||pt.y>cliff_detect_y_max_)continue;
            int block_id=int((pt.x-cliff_detect_x_min_)/cliff_detect_block_length_);
            nonground_blocks_z[block_id].push_back(pt.z);
        }
        vector<bool>cliff_state(block_size,false);
        vector<float>bolck_heights(block_size,10);
        for(int i=0;i<block_size;i++){
          if(ground_blocks_z[i].size()<cliff_detect_min_pts_&&nonground_blocks_z[i].size()<cliff_detect_min_pts_){
            cliff_state[i]=true;
            break;
          }
          bolck_heights[i]=std::accumulate(ground_blocks_z[i].begin(),ground_blocks_z[i].end(),0)/ground_blocks_z[i].size();
        }
        for(int i=0;i<block_size;i++){
          if(cliff_state[i]||(i!=0&&(bolck_heights[i-1]-bolck_heights[i]>0.1))){
            float blk_x_min=cliff_detect_x_min_+i*cliff_detect_block_length_;
            float blk_x_max=blk_x_min+cliff_detect_block_length_;
            pcl::PointCloud<pcl::PointXYZI> cliff_cloud;
            pcl::PointXYZI new_pt;
            new_pt.z=0.5;
            new_pt.intensity=0;
            for(float x=blk_x_min;x<=blk_x_max;x+=0.05){
              for(float y=cliff_detect_y_min_;y<=cliff_detect_y_max_;y+=0.1)
              {
                new_pt.x=x;new_pt.y=y;
                cliff_cloud.points.push_back(new_pt);
              }
            }
            pc_no_ground->points.insert(pc_no_ground->points.end(),cliff_cloud.points.begin(),cliff_cloud.points.end());
            pc_no_ground->width=pc_no_ground->points.size();
            pc_no_ground->height=1;
            break;
          }
        }
      }
      pc_no_ground->points.insert(pc_no_ground->points.end(),back_cloud_filtered->points.begin(),back_cloud_filtered->points.end());
      pc_no_ground->width=pc_no_ground->points.size();
      pc_no_ground->height=1;
    }
    catch (std::exception& e) {
      ROS_ERROR("Ground Segmentation error: %s", e.what());
      return;
    }

    // ROS_DEBUG_STREAM("\033[1;32m" << "Input PointCloud: " << pc_current->size() << " -> Ground: " << pc_ground->size() <<  "/ NonGround: " << pc_no_ground->size()
    //       << " (running_time: " << time_taken << " sec)" << "\033[0m");

    if(pub_ground_and_nonground_){
      pub_ground_.publish(PointCloudToLaserScanNodelet::cloud2msg(*pc_ground, cloud_msg->header.stamp, target_frame_));
      auto cloud_no_ground_msg = PointCloudToLaserScanNodelet::cloud2msg(*pc_no_ground, cloud_msg->header.stamp, target_frame_);
      pub_nonground_.publish(cloud_no_ground_msg);
    }
    // convert sensor_msgs::PointCloud2 to  sensor_msgs::PointCloud2ConstPtr
    // cloud = boost::make_shared<sensor_msgs::PointCloud2>(cloud_no_ground_msg);
    front_cloud_filtered=pc_no_ground;
}

if(use_radiu_filter_){
  // pcl::fromROSMsg(*cloud, *cloud_filtered);
  pcl::RadiusOutlierRemoval<pcl::PointXYZI> outrem;
  outrem.setInputCloud(cloud_filtered);
  outrem.setMinNeighborsInRadius(min_neighbors_);
  outrem.setRadiusSearch(radiu_threshold_);
  outrem.filter(*cloud_filtered);
  // pcl::toROSMsg(*cloud_filtered, *cloud);
}

	float dynamic_min_height=min_height_;
    // Iterate through pointcloud
  for(auto&pt:cloud_filtered->points){
      double range = hypot(pt.x, pt.y);
      if (range < range_min_)
      {
        NODELET_DEBUG("rejected for range %f below minimum value %f. Point: (%f, %f, %f)", range, range_min_, pt.x, pt.y,
                      pt.z);
        continue;
      }
      dynamic_min_height=(range<=dynamic_height_range_start_)?min_height_:min_height_+(range-dynamic_height_range_start_)*2*dynamic_height_gap_per_half_mile_;
      if (pt.z > max_height_ || pt.z < dynamic_min_height)
      {
        NODELET_DEBUG("rejected for height %f not in range (%f, %f)\n", pt.z, min_height_, max_height_);
        continue;
      }
      double angle = atan2(pt.y, pt.x);
      if (angle < output.angle_min || angle > output.angle_max)
      {
        NODELET_DEBUG("rejected for angle %f not in range (%f, %f)\n", angle, output.angle_min, output.angle_max);
        continue;
      }
      //overwrite range at laserscan ray if new range is smaller
      int index = (angle - output.angle_min) / output.angle_increment;
      if (range < output.ranges[index])
      {
        output.ranges[index] = range;
      }
    }
    pub_.publish(output);
  }

}

PLUGINLIB_EXPORT_CLASS(pointcloud_to_laserscan::PointCloudToLaserScanNodelet, nodelet::Nodelet);
