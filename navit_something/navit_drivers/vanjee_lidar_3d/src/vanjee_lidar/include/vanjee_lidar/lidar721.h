
#ifndef __VANJEE_LIDAR721_H
#define __VANJEE_LIDAR721_H

#include <errno.h>
#include <stdint.h>
#include <string>
#include <boost/format.hpp>
#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <vanjee_msgs/VanjeeScan.h>
#include <vanjee_lidar/point_types.h>
#include <vanjee_lidar/calibration.h>
#include <vanjee_lidar/rawdata.hpp>

namespace vanjee_rawdata
{

	static const int16_t LIDAR721_PACKNUM = 5;
	static const int16_t LIDAR721_PACKLEN = 260;
	static const int16_t LIDAR721_CHANNEL = 64;
	static const int16_t LIDAR721_CHANLEN = 4;
	static const float   LIDAR721_DISPARA = 0.001;
	/** \brief vanjee data conversion class */
	class Lidar721 : public RawData
	{

	public:
		Lidar721();
		~Lidar721()
		{
		}

		int setup();
		inline void unpack(const vanjee_msgs::VanjeePacket &pkt, VPointCloud &pc);
		inline void unpackIMU(const vanjee_msgs::VanjeePacket &pkt, IMUpcak *pimu);
		int setCalibration();

		virtual void processScantopoint(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg);
		virtual void upDatapointEx(const vanjee_msgs::VanjeePacket &pkt);
		inline void publishpointfunc(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg);

		virtual void initbutt(const sensor_msgs::Imu::ConstPtr &msg);

		virtual void pblishImuQueue();

	private:
		float angleResolutionVal, lineAngleGap, groupAngleVal;
		double lightgroupVal;
		float VERT_ANGLE[64];
		float AzimuthDiff[64];
		std::string deviceIP;
		/** configuration parameters */

		int return_mode_; // return mode.0=single return mode,1=dual return mode.

		double setTimetemp(uint8_t *pdata,bool timetype);

		typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

		mutable boost::mutex mutex_getpoint;	
	};

} // namespace vanjee_rawdata

#endif // __VANJEE_LIDAR721_H
