// #define Kp 8.75f    
// #define Ki 0.061f   

#include <fstream>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include <string>
#include <ctime>
#include <chrono>
using namespace std;

#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>

#include <pcl/common/transforms.h>
#include <vanjee_lidar/lidar721.h>
#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)
namespace vanjee_rawdata
{
	////////////////////////////////////////////////////////////////////////
	//
	// Lidar721 base class implementation
	//
	////////////////////////////////////////////////////////////////////////

	Lidar721::Lidar721()
	{
		angleResolutionVal = 0;
		lineAngleGap = 0;
		groupAngleVal = 0;
	}

	/** Set up for on-line operation. */
	int Lidar721::setup()
	{
		// Set up cached values for sin and cos of all the possible headings
		std::string anglePath;
		angleResolutionVal = config_lidaryaml.rmp / 1000.0;

		config_lidaryaml.axisoffset = 360 - config_lidaryaml.axisoffset;

		//ROS_INFO("angleResolutionVal:%f", angleResolutionVal);
	
		return 0;
	}

	/** @brief reset calibration. */
	int Lidar721::setCalibration()
	{
		std::string anglePath;
		anglePath = ros::package::getPath("vanjee_lidar");
        anglePath = anglePath +  "/data/Vanjee_lidar_64/" + config_lidaryaml.Hvalconpath;
		ROS_INFO("config_lidaryaml.Hvalconpath:%s ",anglePath.c_str());

		ifstream file;
		file.open(anglePath.c_str());
		unsigned int i = 0;
		//fprintf(stderr, "set Calibration in pointcloud\n");
		if (!file)
		{
			ROS_WARN_STREAM("[cloud][rawdata][ 721 ] " << anglePath << " does not exist , vanjeelidar  will use the default value");

			//file.close();
			anglePath = ros::package::getPath("vanjee_lidar");
			anglePath = anglePath +  "/data/Vanjee_lidar_64/Vanjee64-carside.csv";
			ROS_INFO("[default value csv][%s]",anglePath.c_str());
			file.open(anglePath.c_str());
			if(!file)
			{
				ROS_WARN_STREAM("Default value does not exist.out read");
			    return -1;
			}
		}

		string wow, mem, key;
		unsigned int x = 0;
		while (true)
		{
			getline(file, wow);
			if (file.fail())
				break; // check for error
			while (x < wow.length())
			{
				if (wow[x] == ',')
				{
					key = mem;
					mem.clear();
					x++; // step over ','
				}
				else
					mem += wow[x++];
			}
			istringstream isAng(key);
			isAng >> VERT_ANGLE[i];
			if (abs(VERT_ANGLE[i]) > 50)
			{
				ROS_WARN_STREAM("[ Vertical angle found error ]");
				return -1;
			}
			istringstream isAzimuth(mem);
			isAzimuth >> AzimuthDiff[i];
			// fprintf(stderr,"output:%f,%f\n",VERT_ANGLE[i],AzimuthDiff[i]);
			i++;
			mem.clear(); // reset memory
			key.clear();
			x = 0; // reset index
		}
		file.close();

		return 0;
	}

	void Lidar721::processScantopoint(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg)
	{
		boost::shared_ptr<boost::thread> publishpoint_ptr;
        publishpoint_ptr.reset(new boost::thread(boost::bind(&Lidar721::publishpointfunc,this,scanMsg)));
	}

	void Lidar721::upDatapointEx(const vanjee_msgs::VanjeePacket &pkt)
	{

	}

	inline void Lidar721::publishpointfunc(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg)
	{
		//boost::unique_lock<boost::mutex> lock_getpoint(mutex_getpoint);
		vanjee_rawdata::VPointCloud::Ptr
			outMsg(new vanjee_rawdata::VPointCloud());

		vanjee_rawdata::IMUpcak l_packimu;
		
		// process each packet provided by the driver 0.2:120
		for (int i = 0; i < scanMsg->packets.size(); ++i)
		{
			unpack(scanMsg->packets[i], *outMsg);		
			
		}
		//ROS_INFO("OUT");
		unpackIMU(scanMsg->packets[scanMsg->packets.size()-1], &l_packimu);

		

		vanjee_rawdata::VPointCloud::Ptr outMsg_hight(new vanjee_rawdata::VPointCloud());
		// ROS_INFO("%d",outMsg->width);
		if (outMsg->width == 115200 || outMsg->width == 38400 || outMsg->width == 23044 || outMsg->width == 76800)
		{
			outMsg_hight->height = 64;
			outMsg_hight->width = outMsg->width / 64;

			for (int i = 0; i < outMsg_hight->height; i++)
			{
				for (int j = 0; j < outMsg_hight->width; j++)
				{
					// outMsg_hight->points[j*1800+i].x = outMsg->points[i*1800+j].x;
					outMsg_hight->points.push_back(outMsg->points[j * outMsg_hight->height + i]);
				}
			}

			if (!config_lidaryaml.time_mode)
			{
				//outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
				double time = ros::Time::now().toSec();
                outMsg_hight->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(time));
			}
			else
			{
				double sec = l_packimu.timsec;
				double secm = l_packimu.timetemp;
				double timelidar = sec + secm / 1000000.0; // * (10-timenum);
				//ROS_INFO(" %f   %f    %f",sec,secm,timelidar);
				if(timelidar >= 0)
				{
					outMsg_hight->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timelidar));
				}
				else
				{
					ROS_INFO("timelidar < 0  %f   %f    %f",sec,secm,timelidar);
				}
				
			} 

			outMsg_hight->header.frame_id = config_lidaryaml.frameid;;
			output_pointcloud.publish(outMsg_hight);
			return;
		}
		else
		{
			if (!config_lidaryaml.time_mode)
			{
				//outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;
				double time = ros::Time::now().toSec();
                outMsg->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(time));
			}
			else
			{
				double sec = l_packimu.timsec;
				double secm = l_packimu.timetemp;
				double timelidar = sec + secm / 1000000.0; // * (10-timenum);
				//ROS_INFO(" %f   %f    %f",sec,secm,timelidar);
				if(timelidar >= 0)
				{
					outMsg->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timelidar));
				}
				else
				{
					ROS_INFO("timelidar < 0  %f   %f    %f",sec,secm,timelidar);
				}
				
			} 
			outMsg->height = 1;
			outMsg->header.frame_id = config_lidaryaml.frameid;;
			output_pointcloud.publish(outMsg);
		}
	}

	void Lidar721::pblishImuQueue()
	{

	}

	inline void Lidar721::unpackIMU(const vanjee_msgs::VanjeePacket &pkt, IMUpcak *pimu)
	{
		//boost::unique_lock<boost::mutex> lock_getpoint(mutex_getpoint);

		const uint8_t *pbuf = nullptr;
		const uint8_t *pdata = (const uint8_t *)&pkt.data[0];
		unsigned short l_header = (pdata[0] * 256) + pdata[1];
		//fprintf(stderr,"hearer  ****   %x\n",l_header);
		switch (l_header)
		{
		case 0xFFEE:
		{
			pbuf = (const uint8_t *)&pkt.data[1300];
		}
		break;

		case 0xFFDD:
		{
			pbuf = (const uint8_t *)&pkt.data[1040];
		}
		break;

		default:
			break;
		}

		if (pbuf != nullptr)
		{
			// int len_data;
			// if (pdata[1] == 0xEE)
			// {
			// 	len_data = 1324;
			// }

			// pimu->timsec = setTimetemp((u_int8_t *)&pdata[1324 - 24], true);
			pimu->timsec = setTimetemp((u_int8_t *)&pbuf[0], true);
			//pimu->pcknum = (pbuf[56] << 8) + pbuf[57];
			//pimu->timetemp = ((pbuf[18]<<24)&0x0f) + (pbuf[19] << 16) + (pbuf[20] << 8) + ((pbuf[21]));
			pimu->timetemp = ((pbuf[19]<<24)&0x0f) + (pbuf[18] << 16) + (pbuf[17] << 8) + ((pbuf[16]));
			//fprintf(stderr,"pimu->timsec %ld   0x%x   0x%x   0x%x   0x%x   0x%x   0x%\n",pimu->timetemp,pbuf[16],pbuf[17],pbuf[18],pbuf[19],pbuf[20],pbuf[21]);
		}
		else
		{
			
		}

	}

	void Lidar721::initbutt(const sensor_msgs::Imu::ConstPtr &msg)
	{
		
	}

	/** @brief convert raw packet to point cloud
	 *
	 *  @param pkt raw packet to unpack
	 *  @param pc shared pointer to point cloud (points are appended)
	 */
	void Lidar721::unpack(const vanjee_msgs::VanjeePacket &pkt,
						 VPointCloud &pc)
	{
		ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
		const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
		const uint8_t *pdata = (const uint8_t *)&pkt.data[0];
		static unsigned int l_packetistr = 1300;

		unsigned int l_bish = pdata[0] << 8 | pdata[1];

		if(!uprpm)
		{
			
			switch(l_bish)
			{
				case 0xFFDD:
				{
					config_lidaryaml.rmp = pdata[1050] * 256 + pdata[1051];
					l_packetistr = 1040;
				}
				break;

				case 0xFFEE:
				{
					config_lidaryaml.rmp = pdata[1310] * 256 + pdata[1311];
					l_packetistr = 1300;
				}
				break;
			}

			setup();
			uprpm = true;
		}

		// fprintf(stderr," packet %02X %02X\n",pkt.data[0],pkt.data[1]);
		switch(l_bish)
		{
			case 0xFFEE:
			{
				double timetemp = setTimetemp((u_int8_t *)&pdata[l_packetistr], config_lidaryaml.time_mode);
				for (int i = 0; i < LIDAR721_PACKNUM; i++)
				{
					int azimuth = pdata[LIDAR721_PACKLEN * i + 2] << 8 | pdata[LIDAR721_PACKLEN * i + 3];

					// fprintf(stderr,"amu: %d \n",azimuth);
					for (int j = 0, k = 0; j < LIDAR721_CHANNEL; j++, k += LIDAR721_CHANLEN)
					{
						float distance = pdata[LIDAR721_PACKLEN * i + k + 4] << 8 | pdata[LIDAR721_PACKLEN * i + k + 5];
						distance = distance / 1000.0 * 4;
						float confidence = pdata[LIDAR721_PACKLEN * i + k + 6];

						static int recAzimuth = -1;
						float xy, x, y, z;

						float azimuthInRadians = 0, azimuthAngle = 0;

						// azimuthAngle = (azimuth) / 100.0;
						azimuthAngle = ((int)(azimuth + config_lidaryaml.axisoffset * 100) % 36000) / 100.0;
						// fprintf(stderr,"amu: %f \n",azimuthAngle);
						// if (azimuthAngle >= config_lidaryaml.startang && azimuthAngle <= config_lidaryaml.endang)
						{
							azimuthAngle += AzimuthDiff[j];

							if (0 > azimuthAngle)
							{
								azimuthAngle = 360 + azimuthAngle;
							}

							azimuthInRadians = HDL_Grabber_toRadians(azimuthAngle);

							recAzimuth = azimuth;
							float verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[j]);
							xy = distance * cos(verticalVal);
							x = xy * (sin(azimuthInRadians));
							y = xy * (cos(azimuthInRadians));
							z = distance * sin(verticalVal);
							VPoint point;
							if (x == 0 && y == 0 && z == 0)
							{
								point.x = NAN; //_coord;
								point.y = NAN; //_coord;
								point.z = NAN; //_coord;
							}
							else
							{
								point.x = x; //_coord;
								point.y = y; //_coord;
								point.z = z; //_coord;
							}
							point.intensity = confidence;
							point.ring = j;
							point.timestamp = timetemp;

							// append this point to the cloud
							pc.points.push_back(point);
							++pc.width;
						}
					}
				}
			}
			break;

			case 0xFFDD:
			{
				double timetemp = setTimetemp((u_int8_t *)&pdata[l_packetistr], config_lidaryaml.time_mode);
				for (int i = 0; i < (LIDAR721_PACKNUM-1); i++)
				{
					int azimuth = pdata[LIDAR721_PACKLEN * i + 2] << 8 | pdata[LIDAR721_PACKLEN * i + 3];

					// fprintf(stderr,"amu: %d \n",azimuth);
					for (int j = 0, k = 0; j < LIDAR721_CHANNEL; j++, k += LIDAR721_CHANLEN)
					{
						float distance = pdata[LIDAR721_PACKLEN * i + k + 4] << 8 | pdata[LIDAR721_PACKLEN * i + k + 5];
						distance = distance / 1000.0 * 4;
						float confidence = pdata[LIDAR721_PACKLEN * i + k + 6];

						static int recAzimuth = -1;
						float xy, x, y, z;

						float azimuthInRadians = 0, azimuthAngle = 0;

						// azimuthAngle = (azimuth) / 100.0;
						azimuthAngle = ((int)(azimuth + config_lidaryaml.axisoffset * 100) % 36000) / 100.0;
						// fprintf(stderr,"amu: %f \n",azimuthAngle);
						// if (azimuthAngle >= config_lidaryaml.startang && azimuthAngle <= config_lidaryaml.endang)
						{
							azimuthAngle += AzimuthDiff[j];

							if (0 > azimuthAngle)
							{
								azimuthAngle = 360 + azimuthAngle;
							}

							azimuthInRadians = HDL_Grabber_toRadians(azimuthAngle);

							recAzimuth = azimuth;
							float verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[j]);
							xy = distance * cos(verticalVal);
							x = xy * (sin(azimuthInRadians));
							y = xy * (cos(azimuthInRadians));
							z = distance * sin(verticalVal);
							VPoint point;
							if (x == 0 && y == 0 && z == 0)
							{
								point.x = NAN; //_coord;
								point.y = NAN; //_coord;
								point.z = NAN; //_coord;
							}
							else
							{
								point.x = x; //_coord;
								point.y = y; //_coord;
								point.z = z; //_coord;
							}
							point.intensity = confidence;
							point.ring = j;
							point.timestamp = timetemp;

							// append this point to the cloud
							pc.points.push_back(point);
							++pc.width;
						}
					}
				}
			}
			break;
		}
		
		
	}


	double Lidar721::setTimetemp(uint8_t *pdata,bool timetype)
	{
		if (!timetype)
		{
			const auto t = std::chrono::system_clock::now();
			const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
			return (double)t_sec.count();
		}
		else
		{
			std::tm stm;
			memset(&stm, 0, sizeof(stm));

			stm.tm_year = pdata[12] - 100;
			stm.tm_mon = pdata[13];
			stm.tm_mday = pdata[14];
			stm.tm_hour = pdata[15];
			stm.tm_min = pdata[16];
			stm.tm_sec = pdata[17];

			//printf("%d   %d   %d   %d   %d   %d %d\n",pdata[12],pdata[13],pdata[14],pdata[15],pdata[16],pdata[17],std::mktime(&stm));

			int sec = pdata[15] << 24 | pdata[14] << 16 | pdata[13] << 8 | pdata[12];
			//printf("%d   %d   %d   %d\n",pdata[12],pdata[13],pdata[14],pdata[15]);
			return sec;
			//return std::mktime(&stm);
		}
	}

} // namespace vanjee_rawdata
