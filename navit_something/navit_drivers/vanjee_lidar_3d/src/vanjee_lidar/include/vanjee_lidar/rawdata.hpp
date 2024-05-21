
#ifndef __VANJEE_RAWDATA_H
#define __VANJEE_RAWDATA_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>
#include <queue>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <vanjee_msgs/VanjeeScan.h>
#include <vanjee_lidar/point_types.h>
#include <vanjee_lidar/calibration.h>
#include <ros/package.h>
#include <boost/thread.hpp>
#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>
#include <vanjee_lidar/yaml_reader.hpp>

namespace vanjee_rawdata {
// Shorthand typedefs for point cloud representations
typedef vanjee_lidar::PointXYZIR VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud2;
/**
 * Raw vanjee packet constants and structures.
 */
static const int SIZE_BLOCK = 78;
static const int RAW_SCAN_SIZE = 4;
static const int SCANS_PER_BLOCK = 19;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const float ROTATION_RESOLUTION = 0.01f;  // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;     // [deg/100]
static const float DISTANCE_RESOLUTION = 0.004f; // [m]

/** @todo make this work for both big and little-endian machines */
static const uint16_t UPPER_BANK = 0xeeff;
static const uint16_t LOWER_BANK = 0xddff;

/** Special Defines for VLP16 support **/
static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;   // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;   // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]

/** \brief Raw vanjee data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block {
	uint16_t header;        ///< UPPER_BANK or LOWER_BANK
	uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
	uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

struct IMUpcak{
		float gyro_x;        /**< Gyroscope X axis, Unit:rad/s */
  		float gyro_y;        /**< Gyroscope Y axis, Unit:rad/s */
  		float gyro_z;        /**< Gyroscope Z axis, Unit:rad/s */
  		float acc_x;         /**< Accelerometer X axis, Unit:g */
  		float acc_y;         /**< Accelerometer Y axis, Unit:g */
  		float acc_z;         /**< Accelerometer Z axis, Unit:g */
		double timsec;
		long timetemp;     /**< time, ns */
		int pcknum;
		int anglestart;
	} ;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
	uint16_t uint;
	uint8_t bytes[2];
};

static const int BLOCKS_PER_PACKET = 15;
static const int PACKET_HEADER_SIZE = 5;
static const int PACKET_STATUS_SIZE = 4;
static const int SCANS_PER_PACKET = (SCANS_PER_BLOCK * BLOCKS_PER_PACKET);

/** \brief Raw vanjee packet.
 *
 *  revolution is described in the device manual as incrementing
 *    (mod 65536) for each physical turn of the device.  Our device
 *    seems to alternate between two different values every third
 *    packet.  One value increases, the other decreases.
 *
 *  \todo figure out if revolution is only present for one of the
 *  two types of status fields
 *
 *  status has either a temperature encoding or the microcode level
 */
typedef struct raw_packet {
	// uint8_t header[PACKET_HEADER_SIZE];
	raw_block_t blocks[BLOCKS_PER_PACKET];
	uint16_t revolution;
	uint8_t status[PACKET_STATUS_SIZE];
} raw_packet_t;

/** \brief vanjee data conversion class */
class RawData {
public:

	RawData()
	{
		uprpm = false;
	}

	~RawData()
	{
	}

	void readyamlconfig(ros::NodeHandle node,ros::NodeHandle private_nh,std::string configpath,int lidarNO)
	{
		YAML::Node config;
		config = YAML::LoadFile(configpath);
		// std::cout << config["lidar"] << std::endl; //可以直接用下标访问

		YAML::Node common_config = lidar::yamlSubNodeAbort(config, "lidar");
		config_lidaryaml.point_cloudtopic = common_config[lidarNO]["ros_topic"]["ros_send_point_cloud_topic"].as<std::string>();
		//ROS_INFO("point_cloudtopic : %s", config_lidaryaml.point_cloudtopic.c_str());

		config_lidaryaml.point_imutopic = common_config[lidarNO]["ros_topic"]["ros_send_imu_topic"].as<std::string>();
		//ROS_INFO("point_imutopic : %s", config_lidaryaml.point_imutopic.c_str());

		config_lidaryaml.point_packetscantopic = common_config[lidarNO]["ros_topic"]["ros_recv_packet_topic"].as<std::string>();
		//ROS_INFO("point_packetscantopic : %s", config_lidaryaml.point_packetscantopic.c_str());		

		config_lidaryaml.frameid = common_config[lidarNO]["ros_topic"]["point_frame_id"].as<std::string>();
		//ROS_INFO("frameid : %s", config_lidaryaml.frameid.c_str());		

		config_lidaryaml.imu_frameid = common_config[lidarNO]["ros_topic"]["imu_frame_id"].as<std::string>();
		//ROS_INFO("imu frameid : %s", config_lidaryaml.imu_frameid.c_str());	

		config_lidaryaml.startang = common_config[lidarNO]["driver"]["min_angle"].as<float>();
		//ROS_INFO("startang : %d", config_lidaryaml.startang);	

		config_lidaryaml.endang = common_config[lidarNO]["driver"]["max_angle"].as<float>();
		//ROS_INFO("endang : %d", config_lidaryaml.endang);	

		config_lidaryaml.Hvalconpath = common_config[lidarNO]["driver"]["calibration"].as<std::string>();//calibutfipath
		//ROS_INFO("Hvalconpath : %s", config_lidaryaml.Hvalconpath.c_str());	

		config_lidaryaml.calibutfipath = common_config[lidarNO]["driver"]["calibutfipath"].as<std::string>();

		config_lidaryaml.pcapfilepatn = common_config[lidarNO]["driver"]["pcap"].as<std::string>();

		config_lidaryaml.axisoffset = common_config[lidarNO]["driver"]["axisoffset"].as<int>();
		//ROS_INFO("axisoffset : %d", config_lidaryaml.axisoffset);	

		config_lidaryaml.time_mode = common_config[lidarNO]["driver"]["time_mode"].as<bool>();
		//ROS_INFO("time_mode : %d", config_lidaryaml.time_mode);	
	}

	virtual int setup() = 0;
	virtual void unpack(const vanjee_msgs::VanjeePacket &pkt, VPointCloud &pc) = 0;
	virtual void unpackIMU(const vanjee_msgs::VanjeePacket &pkt, IMUpcak *pimu) = 0;
	virtual int setCalibration() = 0;
	virtual double setTimetemp(uint8_t *pdata,bool timetype) = 0;
	virtual void processScantopoint(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg) = 0;
	virtual void initbutt(const sensor_msgs::Imu::ConstPtr &msg) = 0;
	virtual void pblishImuQueue() = 0;
	virtual void upDatapointEx(const vanjee_msgs::VanjeePacket &pkt) = 0;

	struct configlidaryaml
	{
		std::string lidartype;
		std::string frameid;
		std::string imu_frameid;
		std::string point_cloudtopic;
		std::string point_imutopic;
		std::string point_packetscantopic;

		float startang;
		float endang;
		std::string Hvalconpath;
		std::string calibutfipath;
		std::string pcapfilepatn;
		int rmp;
		bool axisinversion;
		int  axisoffset;
		bool time_mode;
	};

	configlidaryaml config_lidaryaml;

	ros::Subscriber vanjee_pointscan_;
    ros::Subscriber vanjee_pointbut_;
    ros::Publisher output_pointcloud;
    ros::Publisher output_pointcloudIMU;

	boost::mutex mutex_imupublish;
	std::queue<sensor_msgs::Imu::ConstPtr> queue_imupublish;
	

public:
    float angleResolutionVal,lineAngleGap,groupAngleVal; 
	std::string deviceIP;
	bool uprpm;
	
	typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

};

} // namespace vanjee_rawdata

#endif // __VANJEE_RAWDATA_H
