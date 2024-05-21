#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

#include "include/livox_ros_driver2.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"

namespace livox_ros
{

class LivoxDriverNodelet : public nodelet::Nodelet
{
public:
  LivoxDriverNodelet() {}

  void onInit()
  {
    livox_node_.GetNode().getParam("xfer_format", xfer_format_);
    livox_node_.GetNode().getParam("multi_topic", multi_topic_);
    livox_node_.GetNode().getParam("publish_freq", publish_freq_);
    livox_node_.GetNode().getParam("data_src", data_src_);
    livox_node_.GetNode().getParam("output_data_type", output_type_);
    livox_node_.GetNode().getParam("frame_id", frame_id_);
    livox_node_.GetNode().getParam("enable_lidar_bag", lidar_bag_);
    livox_node_.GetNode().getParam("enable_imu_bag", imu_bag_);

  if (publish_freq_ > 100.0) {
    publish_freq_ = 100.0;
  } else if (publish_freq_ < 0.5) {
    publish_freq_ = 0.5;
  } 

  livox_node_.future_ = livox_node_.exit_signal_.get_future();

  /** Lidar data distribute control and lidar data source set */
  livox_node_.lddc_ptr_ = std::make_unique<Lddc>(xfer_format_, multi_topic_, data_src_, output_type_,
                        publish_freq_, frame_id_, lidar_bag_, imu_bag_);
  livox_node_.lddc_ptr_->SetRosNode(&livox_node_);

  if (data_src_ == kSourceRawLidar) {
    DRIVER_INFO(livox_node_, "Data Source is raw lidar.");

    std::string user_config_path;
    livox_node_.getParam("user_config_path", user_config_path);
    DRIVER_INFO(livox_node_, "Config file : %s", user_config_path.c_str());

    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq_);
    livox_node_.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

    if ((read_lidar->InitLdsLidar(user_config_path))) {
      DRIVER_INFO(livox_node_, "Init lds lidar successfully!");
    } else {
      DRIVER_ERROR(livox_node_, "Init lds lidar failed!");
    }
  } else {
    DRIVER_ERROR(livox_node_, "Invalid data src (%d), please check the launch file", data_src_);
  }

  livox_node_.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &livox_node_);
  livox_node_.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &livox_node_);

  }

private:
  DriverNode livox_node_;

  int xfer_format_;
  int multi_topic_;
  int data_src_;
  double publish_freq_;
  int output_type_;
  std::string frame_id_;
  bool lidar_bag_;
  bool imu_bag_;

};

void DriverNode::PointCloudDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributePointCloudData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}

void DriverNode::ImuDataPollThread()
{
  std::future_status status;
  std::this_thread::sleep_for(std::chrono::seconds(3));
  do {
    lddc_ptr_->DistributeImuData();
    status = future_.wait_for(std::chrono::microseconds(0));
  } while (status == std::future_status::timeout);
}





















}  // namespace livox_ros
PLUGINLIB_EXPORT_CLASS(livox_ros::LivoxDriverNodelet, nodelet::Nodelet)
