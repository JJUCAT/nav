
#ifndef __VANJEE_LIDAR720_H
#define __VANJEE_LIDAR720_H

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
#include "vanjee_lidar/complementary_filter.h"

namespace vanjee_rawdata {

/** \brief vanjee data conversion class */
class Lidar720 : public RawData
{
public:

	Lidar720();
	~Lidar720() 
	{
	}

	int setup();
	inline void unpack(const vanjee_msgs::VanjeePacket &pkt, VPointCloud &pc);
	inline void unpackIMU(const vanjee_msgs::VanjeePacket &pkt, IMUpcak *pimu);
	int setCalibration();

	void setlightangle(int rmp);

	virtual void processScantopoint(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg);
	virtual void upDatapointEx(const vanjee_msgs::VanjeePacket &pkt);
	inline void publishpointfunc(const vanjee_msgs::VanjeeScan::ConstPtr scanMsg);

	//about imu
	virtual void initbutt(const sensor_msgs::Imu::ConstPtr &msg);
	void membutt(const sensor_msgs::Imu::ConstPtr msg);
	void membuttimu(float &gx,float &gy,float &gz);
	void readtxtpath();
	void rotateimu(float &gx, float &gy, float &gz, float &ax, float &ay, float &az);
	void IMUZeroDrift(float gx,float gy,float gz,float ax,float ay,float az);
	int IMUSlidequally(float gx,float gy,float gz, float ax,float ay,float az, int num);
	void initDataXY(float *data_x,float *data_y,int data_n);
	inline void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,double timsec,long time,int i);//,float *pitch,float *roll,float *yaw);
	virtual void pblishImuQueue();

private:
    float angleResolutionVal,lineAngleGap,groupAngleVal; 
	double lightgroupVal;
	double lightlineVal;
    float VERT_ANGLE[19];
	float AzimuthDiff[19];
	double light_vec1[57600];
	double light_vec2[68400];
	std::string deviceIP;

	double time_stamp_ns;
	int point_ex;
	int recAzimuth;
	float first_Angazimuth;
	float second_Angazimuth;
	float third_Angazimuth;
	float fourth_Angazimuth;
	double timeimu_start;
	double timeimu_start_last;			// last timeimu_start value
	double timeimu_start_cur;			// current timeimu_start for cal
	double timeimu_step;
	float templature;

	bool timeredressflag;
	double timeredress;

	//************************down about imu ****************************
	imu_tools::ComplementaryFilter m_fiter;
    float da[4][500];
    int da_line;
    float d_k ,d_b;
    float q0, q1, q2, q3;          // 四元数的元素，代表估计方向
    double exInt, eyInt, ezInt;        // 按比例缩小积分误差
    double exPre, eyPre, ezPre;
    double pitch;
    double roll;
    double yaw;
    int IMUZeroDriftNum;
    int IMUZeroDriftALLNum;
    double IMUSlideArray[3][1000];
	double IMUSAcdeArray[3][1000];
    int IMUSlideLen;
    int IMUSlideALLlen;
    int IMUSlidelenNow;
    int timePre;
    int timenum;
    double qa0;
    double qa1;
    double qa2;
    double qa3;

    struct IMUZDrift{
      double ImuZero;
      double ImuAgv;
	  double ImuAccagv;
    };

    IMUZDrift m_imuXstu;
    IMUZDrift m_imuYstu;
    IMUZDrift m_imuZstu;

    IMUZDrift m_imuXstu_a;
    IMUZDrift m_imuYstu_a;
    IMUZDrift m_imuZstu_a;

    float k_bX[2];
    float k_bY[2];
    float k_bZ[2];

	float DS_par[3][2];
    float rotate_B[2][2];
	//************************up about imu ****************************

	double setTimetemp(uint8_t *pdata,bool timetype);

	typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

	mutable boost::mutex mutex_bot;
	mutable boost::mutex mutex_getpoint;

	mutable boost::mutex mutex_cloudpoint;

	// void readyamlconfig(std::string configpath,int lidarNO);

public:
	double time_start_;
	
};

} // namespace vanjee_rawdata

#endif // __VANJEE_LIDAR720_H
