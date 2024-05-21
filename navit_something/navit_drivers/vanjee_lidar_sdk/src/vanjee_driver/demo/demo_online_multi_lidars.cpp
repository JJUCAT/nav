

#include <vanjee_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif


typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace vanjee::lidar;

class DriverClient
{
public:

  DriverClient(const std::string name)
    : name_(name)
  {
  }

  bool init(const WJDriverParam& param)
  {
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    WJ_INFO << "                      " << name_ << WJ_REND;
    WJ_INFO << "------------------------------------------------------" << WJ_REND;
    param.print();

    driver_.regPointCloudCallback (std::bind(&DriverClient::driverGetPointCloudFromCallerCallback, this),
                                  std::bind(&DriverClient::driverReturnPointCloudToCallerCallback, this, std::placeholders::_1));
    driver_.regExceptionCallback (std::bind(&DriverClient::exceptionCallback, this, std::placeholders::_1));
    driver_.regImuPacketCallback(std::bind(&DriverClient::getImuPacketCallback, this), 
                                  std::bind(&DriverClient::putImuPacketCallback, this, std::placeholders::_1));
    driver_.regScanDataCallback(std::bind(&DriverClient::getScanDataCallback, this), 
                                  std::bind(&DriverClient::putScanDataCallback, this, std::placeholders::_1));

    if (!driver_.init(param))
    {
      WJ_ERROR << name_ << ": Failed to initialize driver." << WJ_REND;
      return false;
    }

    return true;
  }

  bool start()
  {
    to_exit_process_ = false;
    cloud_handle_thread_ = std::thread(std::bind(&DriverClient::processCloud, this));

    driver_.start();
    WJ_DEBUG << name_ << ": Started driver." << WJ_REND;

    return true;
  }

  void stop()
  {
    driver_.stop();

    to_exit_process_ = true;
    cloud_handle_thread_.join();
  }

protected:

  std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
  {
    std::shared_ptr<PointCloudMsg> msg = free_cloud_queue_.pop();
    if (msg.get() != NULL)
    {
      return msg;
    }

    return std::make_shared<PointCloudMsg>();
  }

  void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
  {
    stuffed_cloud_queue_.push(msg);
  }

  std::shared_ptr<ImuPacket> getImuPacketCallback()
  {
    std::shared_ptr<ImuPacket> pkt = free_imu_queue_.pop();
    if(pkt.get() != NULL)
    {
      return pkt;
    }

    return std::make_shared<ImuPacket>();
  }

  void putImuPacketCallback(std::shared_ptr<ImuPacket> msg)
  {
    WJ_MSG << name_ << ": msg--imu: " << msg->linear_acce[0] << WJ_REND;
    imu_queue_.push(msg);
  }

  std::shared_ptr<ScanData> getScanDataCallback()
  {
    std::shared_ptr<ScanData> pkt = free_scan_data_queue_.pop();
    if(pkt.get() != NULL)
    {
      return pkt;
    }

    return std::make_shared<ScanData>();
  }

  void putScanDataCallback(std::shared_ptr<ScanData> msg)
  {
    scan_data_queue_.push(msg);
  }

  void processCloud(void)
  {
    while (!to_exit_process_)
    {
      std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue_.popWait();
      if (msg.get() == NULL)
      {
        continue;
      }

      WJ_MSG << name_ << ": msg: " << msg->seq << " point cloud size: " << msg->points.size() << WJ_REND;

      free_cloud_queue_.push(msg);
    }
  }

  void exceptionCallback(const Error& code)
  {
    WJ_WARNING << name_ << ": " << code.toString() << WJ_REND;
  }

  std::string name_;
  LidarDriver<PointCloudMsg> driver_;
  bool to_exit_process_;
  std::thread cloud_handle_thread_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue_;
  SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue_;
  SyncQueue<std::shared_ptr<ImuPacket>> free_imu_queue_;
  SyncQueue<std::shared_ptr<ImuPacket>> imu_queue_;
  SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
  SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;
};

int main(int argc, char* argv[])
{

  WJDriverParam param_left;                  
  param_left.input_type = InputType::ONLINE_LIDAR;
  param_left.input_param.host_msop_port = 3001;   
  param_left.input_param.lidar_msop_port = 3333;    
  param_left.input_param.host_address = "192.168.2.88";    
  param_left.input_param.lidar_address = "192.168.2.86";
  param_left.input_param.group_address = "0.0.0.0";
  param_left.input_param.pcap_path = "/pointCloudFile/720C231400030001_2023-10-18-14-48-22-869.pcap";   
  param_left.decoder_param.imu_param_path = "/src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";   
  param_left.decoder_param.config_from_file = false;
  param_left.decoder_param.wait_for_difop = true; 
  param_left.lidar_type = LidarType::vanjee_720;   

  DriverClient client_left("LEFT ");
  if (!client_left.init(param_left))                         
  {
    return -1;
  }

  WJDriverParam param_right;                  
  param_right.input_type = InputType::ONLINE_LIDAR;
  param_right.input_param.host_msop_port = 3002; 
  param_right.input_param.lidar_msop_port = 6050;    
  param_right.input_param.host_address = "192.168.2.88";    
  param_right.input_param.lidar_address = "192.168.2.85";    
  param_right.decoder_param.config_from_file = false;
  param_right.decoder_param.wait_for_difop = false; 
  param_right.lidar_type = LidarType::vanjee_719;   

  DriverClient client_right("RIGHT");
  if (!client_right.init(param_right))                         
  {
    return -1;
  }

  // WJDriverParam param_mid;                  
  // param_mid.input_type = InputType::ONLINE_LIDAR;
  // param_mid.input_param.host_msop_port = 3003; 
  // param_mid.input_param.lidar_msop_port = 6050;    
  // param_mid.input_param.host_address = "192.168.2.88";    
  // param_mid.input_param.lidar_address = "192.168.2.2";    
  // param_mid.decoder_param.config_from_file = false;
  // param_mid.decoder_param.wait_for_difop = false; 
  // param_mid.lidar_type = LidarType::vanjee_719;   

  // DriverClient client_mid("MID");
  // if (!client_mid.init(param_mid))                         
  // {
  //   return -1;
  // }

  client_left.start();
  client_right.start();
  // client_mid.start();

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  client_left.stop();
  client_right.stop();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}

