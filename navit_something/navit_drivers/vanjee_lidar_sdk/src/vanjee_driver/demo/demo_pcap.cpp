

#include <vanjee_driver/api/lidar_driver.hpp>

#ifdef ENABLE_PCL_POINTCLOUD
#include <vanjee_driver/msg/pcl_point_cloud_msg.hpp>
#else
#include <vanjee_driver/msg/point_cloud_msg.hpp>
#endif


typedef PointXYZI PointT;
typedef PointCloudT<PointT> PointCloudMsg;

using namespace vanjee::lidar;

SyncQueue<std::shared_ptr<PointCloudMsg>> free_cloud_queue;
SyncQueue<std::shared_ptr<PointCloudMsg>> stuffed_cloud_queue;
SyncQueue<std::shared_ptr<ImuPacket>> free_imu_queue_;
SyncQueue<std::shared_ptr<ImuPacket>> imu_queue_;
SyncQueue<std::shared_ptr<ScanData>> free_scan_data_queue_;
SyncQueue<std::shared_ptr<ScanData>> scan_data_queue_;

// @brief point cloud callback function. The caller should register it to the lidar driver.
std::shared_ptr<PointCloudMsg> driverGetPointCloudFromCallerCallback(void)
{
  std::shared_ptr<PointCloudMsg> msg = free_cloud_queue.pop();
  if (msg.get() != NULL)
  {
    return msg;
  }

  return std::make_shared<PointCloudMsg>();
}

// @brief point cloud callback function. The caller should register it to the lidar driver.
void driverReturnPointCloudToCallerCallback(std::shared_ptr<PointCloudMsg> msg)
{
  stuffed_cloud_queue.push(msg);
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
   printf("recv imu data, time = %lf, seq = %d, angular_voc = %lf, %lf, %lf, inear_acc = %lf, %lf, %lf\r\n", msg->timestamp, msg->seq, 
  msg->angular_voc[0], msg->angular_voc[1], msg->angular_voc[2], 
  msg->linear_acce[0], msg->linear_acce[1], msg->linear_acce[2]);
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

// @brief exception callback function. The caller should register it to the lidar driver.
void exceptionCallback(const Error& code)
{
  WJ_WARNING << code.toString() << WJ_REND;
}

bool to_exit_process = false;
void processCloud(void)
{
  while (!to_exit_process)
  {
    std::shared_ptr<PointCloudMsg> msg = stuffed_cloud_queue.popWait();
    if (msg.get() == NULL)
    {
      continue;
    }

    WJ_MSG << "msg: " << msg->seq << " point cloud size: " << msg->points.size() << WJ_REND;

#if 0
    for (auto it = msg->points.begin(); it != msg->points.end(); it++)
    {
      std::cout << std::fixed << std::setprecision(3) 
                << "(" << it->x << ", " << it->y << ", " << it->z << ", " << (int)it->intensity << ")" 
                << std::endl;
    }
#endif

    free_cloud_queue.push(msg);
  }
}

int main(int argc, char* argv[])
{

  WJDriverParam param; 
  param.input_type = InputType::PCAP_FILE;
  param.input_param.host_msop_port = 3001;    
  param.input_param.lidar_msop_port = 3333;    
  param.input_param.host_address = "192.168.2.88";    
  param.input_param.lidar_address = "192.168.2.86";   
  param.input_param.pcap_path = "/pointCloudFile/720C231400030001_2023-10-18-14-48-22-869.pcap";   
  param.decoder_param.imu_param_path = "/src/vanjee_lidar_sdk/param/vanjee_720_imu_param.csv";   
  param.decoder_param.config_from_file = true;
  param.decoder_param.wait_for_difop = false;
  param.lidar_type = LidarType::vanjee_720;                        
  param.print();

  LidarDriver<PointCloudMsg> driver;               
  driver.regPointCloudCallback(driverGetPointCloudFromCallerCallback, driverReturnPointCloudToCallerCallback); 
  driver.regExceptionCallback(exceptionCallback);  
  driver.regImuPacketCallback(getImuPacketCallback, putImuPacketCallback);
  driver.regScanDataCallback(getScanDataCallback, putScanDataCallback);
  if (!driver.init(param))                         
  {
    WJ_ERROR << "Driver Initialize Error..." << WJ_REND;
    return -1;
  }

  std::thread cloud_handle_thread = std::thread(processCloud);

  driver.start();  

  WJ_DEBUG << "Vanjee Lidar-Driver Linux pcap demo start......" << WJ_REND;

#ifdef ORDERLY_EXIT
  std::this_thread::sleep_for(std::chrono::seconds(10));

  driver.stop();

  to_exit_process = true;
  cloud_handle_thread.join();
#else
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
#endif

  return 0;
}
