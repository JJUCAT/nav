#define Kp 8.75f    
#define Ki 0.061f   

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
#include <vanjee_lidar/lidar720.h>
#define HDL_Grabber_toRadians(x) ((x)*M_PI / 180.0)
namespace vanjee_rawdata
{
	////////////////////////////////////////////////////////////////////////
	//
	// Lidar720 base class implementation
	//
	////////////////////////////////////////////////////////////////////////

	Lidar720::Lidar720()
	{
		angleResolutionVal = 0;
		lineAngleGap = 0;
		lightgroupVal = 0.00005555;
		lightlineVal = 0.00000234;

		templature = -100;

		q0 = 1;
		q1 = 0;
		q2 = 0;
		q3 = 0;
		exInt = 0;
		eyInt = 0;
		ezInt = 0;

		exPre = 0;
		eyPre = 0;
		ezPre = 0;

		IMUZeroDriftNum = 0;
		IMUZeroDriftALLNum = 1000.0;

		m_imuXstu.ImuZero = 0;
		m_imuXstu.ImuAgv = 0;
		m_imuXstu.ImuAccagv = 0;

		m_imuYstu.ImuZero = 0;
		m_imuYstu.ImuAgv = 0;
		m_imuYstu.ImuAccagv = 0;

		m_imuZstu.ImuZero = 0;
		m_imuZstu.ImuAgv = 0;
		m_imuZstu.ImuAccagv = 0;

		memset(IMUSlideArray, 0, sizeof(IMUSlideArray));
		memset(IMUSAcdeArray, 0, sizeof(IMUSAcdeArray)); 
		IMUSlideLen = 0;
		IMUSlideALLlen = 5;
		IMUSlidelenNow = 0;
		timePre = 0;
		timenum = 0;

		da_line = 0;
		memset(da, 0,sizeof(da));

		DS_par[0][0] = DS_par[1][0] = DS_par[2][0] = 0;
		DS_par[0][1] = DS_par[1][1] = DS_par[2][1] = 1;

		boost::shared_ptr<boost::thread> imuthread_ptr1;
        imuthread_ptr1.reset(new boost::thread(boost::bind(&Lidar720::pblishImuQueue, this)));

		timeredressflag = false;
	    timeredress = 0;

		time_stamp_ns = 0;
	    point_ex = 0;
	}

	/** Set up for on-line operation. */
	int Lidar720::setup()
	{
		config_lidaryaml.axisoffset = 360 - config_lidaryaml.axisoffset;
		//ROS_INFO("axisoffset : %d",config_lidaryaml.axisoffset);
		switch (config_lidaryaml.rmp)
		{
		case 300:
			angleResolutionVal = 0.1;
			lineAngleGap = 0.0025;
			break;
		case 600:
			angleResolutionVal = 0.2;
			lineAngleGap = 0.005;
			break;
		case 900:
			angleResolutionVal = 0.3;
			lineAngleGap = 0.0075;
			break;
		case 1200:
			angleResolutionVal = 0.4;
			lineAngleGap = 0.01;
			break;
		default:
			break;
		}
		//fprintf(stderr, "angleResolutionVal:%f\n", angleResolutionVal);
		groupAngleVal = angleResolutionVal / 4;

		setlightangle(config_lidaryaml.rmp);

		readtxtpath();

		rotate_B[0][0] = cos(HDL_Grabber_toRadians(config_lidaryaml.axisoffset));
		rotate_B[0][1] = -sin(HDL_Grabber_toRadians(config_lidaryaml.axisoffset));
		rotate_B[1][0] = sin(HDL_Grabber_toRadians(config_lidaryaml.axisoffset));
		rotate_B[1][1] = cos(HDL_Grabber_toRadians(config_lidaryaml.axisoffset));

		//return 0;
		return 2;
	}

	void Lidar720::setlightangle(int rmp)
	{
		double light_angle1[16];
		double light_angle2[19];
		int point = 1080000 / rmp;
		//fprintf(stderr,"point  %d\n",point);

		for(int i=0;i<16;i++)
		{
			if(i<4)
			{
				light_angle1[i] = lightlineVal * i;
			}
			else if(i<7)
			{
				light_angle1[i] = lightlineVal * (i-4) + lightgroupVal / 4;
			}
			else if(i<12)
			{
				light_angle1[i] = lightlineVal * (i-7) + lightgroupVal / 4 * 2;
			}
			else
			{
				light_angle1[i] = lightlineVal * (i-12) + lightgroupVal / 4 * 3;
			}
			//fprintf(stderr,"%.12f******\n",light_angle1[i]);
		}

		// for(int i=0;i<point;i++)
		// {
		// 	for(int j=0;j<16;j++)
		// 	{
		// 		light_vec1[j+16*i]= i * lightgroupVal + light_angle1[j];
		// 	}
		// }
		for(int i=0;i<3600;i++)
		{
			for(int j=0;j<16;j++)
			{
				light_vec1[j+16*i]= i * lightgroupVal + light_angle1[j];
				
			}
		}

		for(int i = 0;i<19;i++)
		{
			if(i<5)
			{
				light_angle2[i] = lightlineVal * i;
			}
			else if(i<9)
			{
				light_angle2[i] = lightlineVal * (i-5) + lightgroupVal / 4;
			}
			else if(i<15)
			{
				light_angle2[i] = lightlineVal * (i-9) + lightgroupVal / 4 * 2;
			}
			else
			{
				light_angle2[i] = lightlineVal * (i-15) + lightgroupVal / 4 * 3;
			}
			//fprintf(stderr,"%f&&&\n",light_angle2[i]);
		}

		// for(int i=0;i<point;i++)
		// {
		// 	for(int j=0;j<19;j++)
		// 	{
		// 		light_vec2[j+19*i]= i * lightgroupVal + light_angle1[j];
		// 	}
		// }
		for(int i=0;i<3600;i++)
		{
			for(int j=0;j<19;j++)
			{
				light_vec2[j+19*i]= i * lightgroupVal + light_angle2[j];
			}
		}
	}

	/** @brief reset calibration. */
	int Lidar720::setCalibration()
	{
		std::string anglePath;// = config_lidaryaml.Hvalconpath;
		anglePath = ros::package::getPath("vanjee_lidar");
        anglePath = anglePath +  "/data/Vanjee_lidar_16/" + config_lidaryaml.Hvalconpath;
		ROS_INFO("config_lidaryaml.Hvalconpath:%s ",anglePath.c_str());

		ifstream file;
		file.open(anglePath.c_str());
		unsigned int i = 0;
		//fprintf(stderr, "set Calibration in pointcloud\n");
		if (!file)
		{
			ROS_WARN_STREAM("[cloud][rawdata] " << anglePath << " does not exist , vanjeelidar  will use the default value");
			anglePath = ros::package::getPath("vanjee_lidar");
			anglePath = anglePath +  "/data/Vanjee_lidar_16/Vanjee16-carside.csv";
			ROS_INFO("[default value csv][%s]",anglePath.c_str());
			file.open(anglePath.c_str());
			if(!file)
			{
				ROS_WARN_STREAM("Default value does not exist.");
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
			if (VERT_ANGLE[i] > 20 || VERT_ANGLE[i] < -20)
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

		//return 0;
		return 4;
	}


	inline void Lidar720::unpackIMU(const vanjee_msgs::VanjeePacket &pkt, IMUpcak *pimu)
	{
		//boost::unique_lock<boost::mutex> lock_getpoint(mutex_getpoint);

		const uint8_t *pbuf = nullptr;
		const uint8_t *pdata = (const uint8_t *)&pkt.data[0];
		unsigned short l_header = (pdata[0] * 256) + pdata[1];
		//fprintf(stderr,"hearer  ****   %x\n",l_header);
		switch (l_header)
		{
		case 0xFFEE:
			pbuf = (const uint8_t *)&pkt.data[1200];
			pimu->anglestart = pdata[3] * 256 + pdata[2];
			break;
		case 0xFFDD:
			int l_len = (pdata[3] * pdata[4]) * (pdata[2] * 4 + 2) + 5;
			pbuf = (const uint8_t *)&pkt.data[l_len];
			pimu->anglestart = pdata[6] * 256 + pdata[5];
			break;
		}

		if (pbuf != nullptr)
		{
			pimu->gyro_x = (short)(pbuf[14] | (((pbuf[15]) << 8) & 0xff00)); // * 8.75;
			pimu->gyro_y = (short)(pbuf[16] | (((pbuf[17]) << 8) & 0xff00)); // * 8.75;
			pimu->gyro_z = (short)(pbuf[18] | (((pbuf[19]) << 8) & 0xff00)); // * 8.75;
			pimu->acc_x = (short)(pbuf[20] + ((pbuf[21]) * 256));			 // * 0.061;
			pimu->acc_y = (short)(pbuf[22] + ((pbuf[23]) * 256));			 // * 0.061;
			pimu->acc_z = (short)(pbuf[24] + ((pbuf[25]) * 256));			 // * 0.061;

			int len_data;
			if (pdata[1] == 0xEE)
			{
				len_data = 1260;
			}
			else if (pdata[1] == 0xDD)
			{
				len_data = (pdata[2] * 4 + 2) * (pdata[3] * pdata[4]) + 65;
			}

			pimu->timsec = setTimetemp((u_int8_t *)&pdata[len_data - 60], true);
			pimu->pcknum = (pbuf[56] << 8) + pbuf[57];
			pimu->timetemp = pbuf[10] + (pbuf[11] << 8) + (pbuf[12] << 16) + ((pbuf[13] & 0x0F) << 24);
			//fprintf(stderr,"pimu->timsec %d   0x%x   0x%x   0x%x   0x%x\n",pimu->timetemp,pbuf[10],pbuf[11],pbuf[12],pbuf[13]&0x0f);

			//ROS_INFO("pimu->timsec = %f   " , pimu->timsec);
		}
		else
		{
			
			pimu->gyro_x = 8.75;
			//fprintf(stderr, "y:0x%x\n", l_header);
			pimu->gyro_y = 8.75;
			pimu->gyro_z = 8.75;
			pimu->acc_x = 0.061;
			pimu->acc_y = 0.061;
			pimu->acc_z = 0.061;
		}
	}

	/** @brief convert raw packet to point cloud
	 *
	 *  @param pkt raw packet to unpack
	 *  @param pc shared pointer to point cloud (points are appended)
	 */
	inline void Lidar720::unpack(const vanjee_msgs::VanjeePacket &pkt,
						 VPointCloud &pc)
	{
		ROS_DEBUG_STREAM("Received packet, time: " << pkt.stamp);
		const raw_packet_t *raw = (const raw_packet_t *)&pkt.data[0];
		const uint8_t *pdata = (const uint8_t *)&pkt.data[0];

		if(!uprpm)
		{
			int l_rmp = 0;
			
			if(pdata[0] == 0xFF && pdata[1] == 0xDD)
			{
				int l_len = ((unsigned char)pdata[2] * 4 + 2) * ((unsigned char)pdata[4]* (unsigned char)pdata[3]) + 65;
				l_rmp = ((pdata[2] * 4 + 2) * pdata[4] * pdata[3]) + 65;
			}
			else if(pdata[0] == 0xFF && pdata[1] == 0xEE)
			{
				l_rmp = 1260;
			}
			config_lidaryaml.rmp = pdata[l_rmp - 21] * 256 + pdata[l_rmp -22];
			setup();
			uprpm = true;
		}

		// static int l_prepack = 0;
		// static int nummmm = 0;
		// ROS_INFO("mpancket  %d   %d   %d  nummmm=%d",config_lidaryaml.rmp , pdata[1249]*256+pdata[1250] , pdata[1195]*256+pdata[1196] , nummmm++);
		// if(l_prepack != pdata[1195]*256+pdata[1196])
		// {
		// 	if (nummmm != 100)
		// 	{
		// 		ROS_INFO("\n\n cha = %d \n\n" , pdata[1195]*256+pdata[1196] - l_prepack);
		// 	}
		// 	nummmm = 0;
		// }
		// l_prepack = pdata[1195]*256+pdata[1196];
		//static double timetemp = 0;
			
		//fprintf(stderr," packet %02X %02X %02X %02X\n",pkt.data[0],pkt.data[1],pkt.data[7],pkt.data[8]);
		if (pdata[0] == 0xFF && pdata[1] == 0xEE) // 0xFFEE 1260B
		{
			int arrayN=0;
			for (int i = 0; i < BLOCKS_PER_PACKET; i++)
			{
				for (int j = 0, k = 0; j < SCANS_PER_BLOCK; j++, k += RAW_SCAN_SIZE)
				{
					recAzimuth = -1;
					float xy, x, y, z;
					float intensity;
					union two_bytes tmp;
					tmp.bytes[0] = raw->blocks[i].data[k];
					tmp.bytes[1] = raw->blocks[i].data[k + 1];

					float distance = tmp.uint * DISTANCE_RESOLUTION;
					int azimuth = raw->blocks[i].rotation / 10 - 1;
					azimuth = azimuth % 36000;
					intensity = raw->blocks[i].data[k + 2];
					float confidence = raw->blocks[i].data[k + 3];
					int line;
					
					float azimuthInRadians = 0, azimuthAngle = 0;
					azimuthAngle = (raw->blocks[i].rotation) / 100.0;
					// if(azimuthAngle == 360)
					// {
					// 	time_stamp_ns = setTimetemp((u_int8_t *)&pdata[1200], config_lidaryaml.time_mode);
					// }

					if (azimuthAngle >= 0 && azimuthAngle <= 360)
					{
						first_Angazimuth = 0, second_Angazimuth = 0, third_Angazimuth = 0, fourth_Angazimuth = 0;
						if (recAzimuth != azimuth)
						{
							// first_Angazimuth=(raw->blocks[i].rotation)/ 100.0-angleResolutionVal;
							first_Angazimuth = ((int)(raw->blocks[i].rotation + config_lidaryaml.axisoffset * 100) % 36000) / 100.0 - angleResolutionVal;
							second_Angazimuth = first_Angazimuth + groupAngleVal;
							third_Angazimuth = first_Angazimuth + 2 * groupAngleVal;
							fourth_Angazimuth = first_Angazimuth + 3 * groupAngleVal;
						}
						switch (j)
						{
						case 0:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 1:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 1;
							break;
						case 2:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 2;
							break;
						case 3:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 3;
							break;
						case 4:
							azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 4;
							break;
						case 5:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 6:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 5;
							break;
						case 7:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 6;
							break;
						case 8:
							azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 7;
							break;
						case 9:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 10:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 9;
							break;
						case 11:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 10;
							break;
						case 12:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 11;
							break;
						case 13:
							azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 12;
							break;
						case 14:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + AzimuthDiff[j]);
							line = 8;
							break;
						case 15:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + lineAngleGap + AzimuthDiff[j]);
							line = 13;
							break;
						case 16:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
							line = 14;
							break;
						case 17:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
							line = 15;
							break;
						case 18:
							azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
							line = 16;
							break;
						default:
							break;
						}
						recAzimuth = azimuth;
						float verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[j]);
						xy = distance * cos(verticalVal);
						x = xy * (sin(azimuthInRadians));
						y = xy * (cos(azimuthInRadians));
						z = distance * sin(verticalVal);
						VPoint point;
						if(x==0 && y==0 && z==0)
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
						point.ring = line - 1;
						arrayN = azimuth / (config_lidaryaml.rmp / 30) * 19 + j;
						point.timestamp = time_stamp_ns + light_vec2[arrayN];
						//fprintf(stderr,"[%.0f %.0f]   %d",intensity, confidence,pc.width);
						// append this point to the cloud
						pc.points.push_back(point);
						++pc.width;
					}
				}
			}
		}
		else if (pdata[0] == 0xFF && pdata[1] == 0xDD) // 0xFFDD 1001 1352
		{
			int arrayN=0;
			int start_idx = 5;
			int channel_nums = pdata[2];
			int echo_nums = pdata[3];
			int block_nums = pdata[4];
			int all_block_nums = block_nums * echo_nums;
			int block_size = 2 + 4 * channel_nums;
			int channel_size = 4;
			int len_data = (channel_nums * 4 + 2) * all_block_nums + 65;
			
			// fprintf(stderr,"channel_nums:%d block_nums:%d all_block_nums:%d\n",channel_nums,block_nums,all_block_nums);

			if (channel_nums == 16)
			{
				for (int i = 0; i < all_block_nums; i++)
				{
					const uint8_t *block_ptr = &(pdata[start_idx + i * block_size]);
					union two_bytes tmp;
					tmp.bytes[0] = block_ptr[0];
					tmp.bytes[1] = block_ptr[1];
					int azimuth = tmp.uint;
					azimuth = azimuth % 36000;
					// if (i == 0)
					// {
					//    ROS_INFO("azimuth = %d   time_stamp_ns = %.9f  pktnum = %d" , azimuth , time_stamp_ns , pdata[1249]*256+pdata[1250]);
					// }
					// if (azimuth == 36000)
					// {
					// 	time_stamp_ns = setTimetemp((u_int8_t *)&pdata[len_data - 60], config_lidaryaml.time_mode);
					// }

					for (int j = 0, k = 0; j < channel_nums; j++, k += channel_size)
					{
						recAzimuth = -1;
						float xy, x, y, z;
						float intensity;
						float confidence;
						int azimuth_id = tmp.uint / 10 - 1;

						const uint8_t *channel_ptr = &(pdata[start_idx + i * block_size + 2 + j * channel_size]);
						tmp.bytes[0] = channel_ptr[0];
						tmp.bytes[1] = channel_ptr[1];
						float distance = tmp.uint * DISTANCE_RESOLUTION;
						intensity = channel_ptr[2];
						confidence = channel_ptr[3];
						// fprintf(stderr,"distance:%.1f\n",distance);
						int line;
						float verticalVal = 0;
						float azimuthInRadians = 0, azimuthAngle = 0;
						// azimuthAngle = (azimuth) / 100.0;
						azimuthAngle = ((int)(azimuth + config_lidaryaml.axisoffset * 100) % 36000) / 100.0;
					
						if (azimuthAngle >= 0 && azimuthAngle <= 360)
						{
							azimuthInRadians = HDL_Grabber_toRadians(azimuthAngle);
							line = j + 1;
							switch (j)
							{
							case 0:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[1]);
								break;
							case 1:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[2]);
								break;
							case 2:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[3]);
								break;
							case 3:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[4]);
								break;
							case 4:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[6]);
								break;
							case 5:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[7]);
								break;
							case 6:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[8]);
								break;
							case 7:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[9]);
								break;
							case 8:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[10]);
								break;
							case 9:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[11]);
								break;
							case 10:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[12]);
								break;
							case 11:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[13]);
								break;
							case 12:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[15]);
								break;
							case 13:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[16]);
								break;
							case 14:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[17]);
								break;
							case 15:
								verticalVal = HDL_Grabber_toRadians(VERT_ANGLE[18]);
								break;
							default:
								break;
							}
							recAzimuth = azimuth;

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

							arrayN = azimuth / (config_lidaryaml.rmp / 30) * channel_nums + j;
							point.timestamp = time_stamp_ns + light_vec1[arrayN];							
							// append this point to the cloud

							pc.points.push_back(point);
							++pc.width;
						}
					}
				}
			}
			else if (channel_nums == 19)
			{
				for (int i = 0; i < all_block_nums; i++)
				{
					const uint8_t *block_ptr = &(pdata[start_idx + i * block_size]);
					union two_bytes tmp;
					tmp.bytes[0] = block_ptr[0];
					tmp.bytes[1] = block_ptr[1];
					int azimuth = tmp.uint;
					azimuth = azimuth % 36000;
					// if (azimuth == 36000)
					// {
					// 	time_stamp_ns = setTimetemp((u_int8_t *)&pdata[len_data - 60], config_lidaryaml.time_mode);
					// }
					// fprintf(stderr,"azimuth:%d\n",azimuth);
					for (int j = 0, k = 0; j < channel_nums; j++, k += channel_size)
					{
						recAzimuth = -1;
						float xy, x, y, z;
						float intensity;
						float confidence;
						int azimuth_id = tmp.uint / 10 - 1;

						const uint8_t *channel_ptr = &(pdata[start_idx + i * block_size + 2 + j * channel_size]);
						tmp.bytes[0] = channel_ptr[0];
						tmp.bytes[1] = channel_ptr[1];
						float distance = tmp.uint * DISTANCE_RESOLUTION;
						intensity = channel_ptr[2];
						confidence = channel_ptr[3];
						// fprintf(stderr,"distance:%.1f\n",distance);
						int line;

						float azimuthInRadians = 0, azimuthAngle = 0;
						azimuthAngle = (azimuth) / 100.0;

						if (azimuthAngle >= 0 && azimuthAngle <= 360)
						{

							first_Angazimuth = 0, second_Angazimuth = 0, third_Angazimuth = 0, fourth_Angazimuth = 0;
							if (recAzimuth != azimuth)
							{
								// first_Angazimuth = (azimuth) / 100.0 - angleResolutionVal;
								first_Angazimuth = ((int)(azimuth + config_lidaryaml.axisoffset * 100) % 36000) / 100.0 - angleResolutionVal;
								second_Angazimuth = first_Angazimuth + groupAngleVal;
								third_Angazimuth = first_Angazimuth + 2 * groupAngleVal;
								fourth_Angazimuth = first_Angazimuth + 3 * groupAngleVal;
							}
							switch (j)
							{
							case 0:
								azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + AzimuthDiff[j]);
								line = 8;
								break;
							case 1:
								azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + lineAngleGap + AzimuthDiff[j]);
								line = 1;
								break;
							case 2:
								azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
								line = 2;
								break;
							case 3:
								azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
								line = 3;
								break;
							case 4:
								azimuthInRadians = HDL_Grabber_toRadians(first_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
								line = 5;
								break;
							case 5:
								azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + AzimuthDiff[j]);
								line = 8;
								break;
							case 6:
								azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + lineAngleGap + AzimuthDiff[j]);
								line = 5;
								break;
							case 7:
								azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
								line = 6;
								break;
							case 8:
								azimuthInRadians = HDL_Grabber_toRadians(second_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
								line = 7;
								break;
							case 9:
								azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + AzimuthDiff[j]);
								line = 8;
								break;
							case 10:
								azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + lineAngleGap + AzimuthDiff[j]);
								line = 9;
								break;
							case 11:
								azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
								line = 10;
								break;
							case 12:
								azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
								line = 11;
								break;
							case 13:
								azimuthInRadians = HDL_Grabber_toRadians(third_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
								line = 12;
								break;
							case 14:
								azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + AzimuthDiff[j]);
								line = 13;
								break;
							case 15:
								azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + lineAngleGap + AzimuthDiff[j]);
								line = 8;
								break;
							case 16:
								azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 2 * lineAngleGap + AzimuthDiff[j]);
								line = 14;
								break;
							case 17:
								azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 3 * lineAngleGap + AzimuthDiff[j]);
								line = 15;
								break;
							case 18:
								azimuthInRadians = HDL_Grabber_toRadians(fourth_Angazimuth + 4 * lineAngleGap + AzimuthDiff[j]);
								line = 16;
								break;
							default:
								break;
							}
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
							point.ring = line - 1;
							arrayN = azimuth / (config_lidaryaml.rmp / 30) * channel_nums + j;
							point.timestamp = time_stamp_ns + light_vec2[arrayN];
							// append this point to the cloud-
							pc.points.push_back(point);
							++pc.width;
						}
					}
				}
			}
		}
	}

	double Lidar720::setTimetemp(uint8_t *pdata,bool timetype)
	{
		double l_timetemp;
		if (!timetype)
		{
			// const auto t = std::chrono::system_clock::now();
			// const auto t_sec = std::chrono::duration_cast<std::chrono::duration<double>>(t.time_since_epoch());
			// //fprintf(stderr,"%f   %f\n",(double)t_sec.count() , ros::Time::now().toSec());
			// l_timetemp =  (double)t_sec.count();

			
			lightgroupVal = 0.00005555 * (timeimu_step / 0.01);
		    lightlineVal = 0.00000234 * (timeimu_step / 0.01);
			//ROS_INFO("timeimu_step=%.10f   %.10f    %.10f",timeimu_step,lightgroupVal,lightlineVal);
			setlightangle(config_lidaryaml.rmp);
			l_timetemp =  timeimu_start_cur;
		}
		else 
		{
			std::tm stm;
			memset(&stm, 0, sizeof(stm));

			stm.tm_year = pdata[9] + 100;
			stm.tm_mon = pdata[8] - 1;
			stm.tm_mday = pdata[7];
			stm.tm_hour = pdata[6];
			stm.tm_min = pdata[5];
			stm.tm_sec = pdata[4];
			double nsec = (pdata[10] + (pdata[11] << 8) + (pdata[12] << 16) + ((pdata[13] & 0x0F) << 24)) / 100000000.0;

			//fprintf(stderr,"%f  %f\n",(double)std::mktime(&stm),nsec);
			l_timetemp =  std::mktime(&stm) + nsec;
		}

		return l_timetemp;
	}

	void Lidar720::processScantopoint(const vanjee_msgs::VanjeeScan::ConstPtr &scanMsg)
	{
		boost::shared_ptr<boost::thread> publishpoint_ptr;
        //publishpoint_ptr.reset(new boost::thread(boost::bind(&Lidar720::publishpointfunc,this,scanMsg)));

		publishpointfunc(scanMsg);
	}

	void Lidar720::upDatapointEx(const vanjee_msgs::VanjeePacket &pkt)
	{
		const uint8_t *pdata = (const uint8_t *)&pkt.data[0];

		unsigned int header = pdata[0] * 256 + pdata[1];
		int amuith = 0;
		int rmp = 0;
		int len = 1253;
		int light_ex = 0;

		switch (header)
		{
		case 0xFFEE:
			amuith = pdata[3] * 256 + pdata[2];
			rmp = ((unsigned char)pdata[1260 - 21] * 256 + (unsigned char)pdata[1260 - 22]);
			len = 1260;
			break;

		case 0xFFDD:
			amuith = pdata[6] * 256 + pdata[5];
			amuith = amuith % 36000;
			len = (pdata[2] * 4 + 2) * (pdata[3] * pdata[4]) + 65;
			rmp = ((unsigned char)pdata[len - 21] * 256 + (unsigned char)pdata[len - 22]);

			light_ex = amuith / (rmp / 30) * pdata[2];
			break;
		}

		//ROS_INFO("header = 0x%x  len === %d   %d  %d",header, len , pdata[0],pdata[1]);
		double timepck = setTimetemp((u_int8_t *)&pdata[len - 60], true);

		timeimu_start = timepck + timeredress - light_vec1[light_ex];
		if (timeimu_start_last < 1)	// no last imu time
		{
			if (!config_lidaryaml.time_mode)
			{
				timeimu_start_cur = timeimu_start - 0.1;
			}
			else
			{
				timeimu_start_cur = timeimu_start;
			}
			
			timeimu_step = 0.01;
		}
		else
		{
			timeimu_start_cur = timeimu_start_last;
			timeimu_step = (timeimu_start - timeimu_start_cur)/10.0;
		}
		timeimu_start_last = timeimu_start;// buffer for next time use

		// if(config_lidaryaml.time_mode)
		// {
		// 	time_stamp_ns = timepck - light_vec1[light_ex];
		// }
		// else
		// {
		// 	time_stamp_ns = timeimu_start;
		// }

		if (!config_lidaryaml.time_mode)
		{
			time_stamp_ns = timeimu_start - 0.1;
		}
		else
		{
			time_stamp_ns = timeimu_start;
		}
		

	}

	inline void Lidar720::publishpointfunc(const vanjee_msgs::VanjeeScan::ConstPtr scanMsg)
	{
		//return;
		boost::unique_lock<boost::mutex> lock (mutex_bot);

		vanjee_rawdata::IMUpcak l_packimu;

		vanjee_rawdata::VPointCloud::Ptr
			outMsg(new vanjee_rawdata::VPointCloud());
		static double timepre = 0;
		unpackIMU(scanMsg->packets[0], &l_packimu);
		int s_ang = l_packimu.anglestart;
		int s_pkt = l_packimu.pcknum;
		static int s_pktpre = 0;

    if (fabs(ros::Time::now().toSec() - l_packimu.timsec - timeredress) > 0.3)
    {
      timeredressflag = false;
      printf(" ******** reset timeredressflag %lf %lf %lf\n", ros::Time::now().toSec(), l_packimu.timsec, timeredress);
    }

    if (!timeredressflag)
		{
			if (!config_lidaryaml.time_mode)
			{
				timeredress = ros::Time::now().toSec();
			}
			else
			{
				timeredress = l_packimu.timsec;
			}
			
			timeredress = timeredress - l_packimu.timsec;
			timeredressflag = true;
			//fprintf(stderr, "\n\ntimeredress = %f\n\n", timeredress);
		}	
		
		bool imuTimeGetFlag = false;
		double timeimu = ros::Time::now().toSec();
		double timeimuFirst = 0;//timsec + time / 100000000.0;
		// process each packet provided by the driver 0.2:120
		// fprintf(stderr,"scanMsg->packets.size() = %d \n",scanMsg->packets.size());
		for (int i = 0; i < scanMsg->packets.size(); ++i)
		{
			//timeimu_start = timeimu_start + 0.001;
			if(i == 0)
			{
				//const uint8_t *p_data = (const uint8_t *)&scanMsg->packets[i].data[0];
				//ROS_INFO("amiuth = %d   packet  %d   %d   %d",p_data[6]*256+p_data[5] , config_lidaryaml.rmp , p_data[1249]*256+p_data[1250] , p_data[1195]*256+p_data[1196]);
			    upDatapointEx(scanMsg->packets[i]);
			
			}

			unpack(scanMsg->packets[i], *outMsg);
			if((i % 10) == 5)
			{
				if(config_lidaryaml.pcapfilepatn != "nothing")
				{
					//ROS_INFO("nothing");
					unpackIMU(scanMsg->packets[i], &l_packimu);
					
					if (!imuTimeGetFlag)
					{
						timeimuFirst = l_packimu.timsec;// + l_packimu.timetemp / 100000000.0;
						//ROS_INFO("timeimuFirst %f      timeimu = %f",timeimuFirst , timeimu);
						imuTimeGetFlag = true;
					}
					
					if(!config_lidaryaml.time_mode)
					{
						l_packimu.timsec = timeimu - timeimuFirst + l_packimu.timsec;
					}
					
			        IMUupdate(l_packimu.gyro_x, l_packimu.gyro_y, l_packimu.gyro_z, l_packimu.acc_x, l_packimu.acc_y, l_packimu.acc_z, l_packimu.timsec, l_packimu.timetemp, i);
				    timeimu_start_cur += timeimu_step;
				}
				
			}
						
		}
		//unpackIMU(scanMsg->packets[scanMsg->packets.size() -1], &l_packimu);
        unpackIMU(scanMsg->packets[0], &l_packimu);


		s_pktpre = s_pkt;

		static double tttttt = 0;
		static int tttpcknum =  0;
		vanjee_rawdata::VPointCloud::Ptr outMsg_hight(new vanjee_rawdata::VPointCloud());
		if (outMsg->width == 57600 || outMsg->width == 28800 || outMsg->width == 14400)
		{
			outMsg_hight->height = 16;
			outMsg_hight->width = outMsg->width / 16;
			//ROS_INFO("%d   %d",outMsg_hight->height,l_i);

			for (int i = 0; i < outMsg_hight->height; i++)
			{
				for (int j = 0; j < outMsg_hight->width; j++)
				{
					// outMsg_hight->points[j*1800+i].x = outMsg->points[i*1800+j].x;
					outMsg_hight->points.push_back(outMsg->points[j * outMsg_hight->height + i]);
					double qqde = outMsg->points[j * outMsg_hight->height + i].timestamp;
					//ROS_INFO("num = %d , index = %d , time = %f" , i * 16 + j , j * outMsg_hight->height + i , qqde);
				}
			}

			if (!config_lidaryaml.time_mode)
			{
				double time = time_stamp_ns + 0.1;
				
				outMsg_hight->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(time));
				//ROS_INFO("tmepoint0 = %f , tmepoint1 = %f , header = %f",tmepoint0 , tmepoint1 , time);
			}
			else
			{
				//double sec = l_packimu.timsec;
				//double secm = l_packimu.timetemp;
				//double timelidar = sec;// + secm / 100000000.0; // * (10-timenum);
				double timelidar = time_stamp_ns + 0.1;
				outMsg_hight->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timelidar));
				//ROS_INFO("[POINTCLOUD 2] : %ld     %f",outMsg_hight->header.stamp,timelidar);
			}
			outMsg_hight->header.frame_id = config_lidaryaml.frameid;
			// ROS_INFO("*****************outMsg_hight %ld  %ld",outMsg_hight->size(),sizeof(outMsg_hight));
			output_pointcloud.publish(outMsg_hight);

			//return;
		}
		else
		{
			if (!config_lidaryaml.time_mode)
			{
				//double time = ros::Time::now().toSec();
				//double time = l_packimu.timsec + timeredress;
				//outMsg->header.stamp = pcl_conversions::toPCL(scanMsg->header).stamp;

				//double time = time_stamp_ns + light_vec1[28799];
				double time = time_stamp_ns + 0.1;
				outMsg->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(time));
				tttttt = time;
				tttpcknum = l_packimu.pcknum;
			}
			else
			{
				//double sec = l_packimu.timsec;
				//double secm = l_packimu.timetemp;
				//double timelidar = sec;// + secm / 100000000.0; // * (10-timenum);

				double timelidar = time_stamp_ns + 0.1;
				outMsg->header.stamp = pcl_conversions::toPCL(ros::Time().fromSec(timelidar));
			}

			outMsg->header.frame_id = config_lidaryaml.frameid; // scanMsg->header.frame_id;
			//ROS_INFO("outMsg->width:%d   outMsg->height:%d",outMsg->width,outMsg->height);
			outMsg->height = 1;
			output_pointcloud.publish(outMsg);
			
			//fprintf(stderr, "not 28800   %d   %.9f   %.9f\n" , outMsg->width  , timeimu_start , timeimu_start_cur);
		}

		for (int y = 0; y < outMsg_hight->width; y++)
		{
			// outMsg_hight->points[j*1800+i].x = outMsg->points[i*1800+j].x;
			//outMsg_hight->points.push_back(outMsg->points[j * outMsg_hight->height + i]);
			outMsg->points.pop_back();
		}
		

		//ROS_INFO("publish pointcloud");
		//printf("out\n");
	}

	void Lidar720::pblishImuQueue()
	{
		sensor_msgs::Imu::ConstPtr msg;
		ros::Rate loop_(1000);

		while (1)
		{
			loop_.sleep();
			if (queue_imupublish.size() > 0)
			{
				{
					boost::unique_lock<boost::mutex> lock(mutex_imupublish);
					msg = queue_imupublish.front();
					queue_imupublish.pop();
				}

				double x;
				if (msg->orientation.x != -10000)
				{
					x = msg->orientation.x;

					double gx = msg->angular_velocity.x;
					double gy = msg->angular_velocity.y;
					double gz = msg->angular_velocity.z;
					double ax = msg->linear_acceleration.x;
					double ay = msg->linear_acceleration.y;
					double az = msg->linear_acceleration.z;

					gx -= k_bX[0] * (x - 40) + k_bX[1];
					gy -= k_bY[0] * (x - 40) + k_bY[1];
					gz -= k_bZ[0] * (x - 40) + k_bZ[1];

					double x_timesec = msg->header.stamp.toSec();
					double x_timensec = msg->orientation.z;// * 100000000;

					IMUupdate(gx, gy, gz, ax, ay, az, x_timesec, (long)x_timensec, 0);
				}
			}
		}
	}

	void Lidar720::initbutt(const sensor_msgs::Imu::ConstPtr &msg)
	{
		
		// boost::shared_ptr<boost::thread> publishpoint_ptr;
        // publishpoint_ptr.reset(new boost::thread(boost::bind(&Lidar720::membutt,this,msg)));

		// boost::unique_lock<boost::mutex> lock (mutex_bot);
        // templature = msg->angular_velocity.x;

		//ROS_INFO("in imu Thread  %f " , templature);

		boost::unique_lock<boost::mutex> lock(mutex_imupublish);
        queue_imupublish.push(msg);  
	}

	void Lidar720::membutt(const sensor_msgs::Imu::ConstPtr msg)
	{
		double x;
		if (msg->orientation.x != -10000)
		{
			x = msg->orientation.x;

			double gx = msg->angular_velocity.x;
			double gy = msg->angular_velocity.y;
			double gz = msg->angular_velocity.z;
			double ax = msg->linear_acceleration.x;
			double ay = msg->linear_acceleration.y;
			double az = msg->linear_acceleration.z;

			// gx -= k_bX[0] * (x - 40) + k_bX[1];
			// gy -= k_bY[0] * (x - 40) + k_bY[1];
			// gz -= k_bZ[0] * (x - 40) + k_bZ[1];
			
			// double x_timesec = msg->header.stamp.toSec();
			// long x_timensec = (msg->header.stamp.toSec() - (long)x_timesec) * 100000000;

			double x_timesec = msg->header.stamp.toSec();// = msg->orientation.y;
			double x_timensec = msg->orientation.z;// * 100000000;

			//ROS_INFO("x_timensec = %f" , x_timensec);

			//ROS_INFO("imu time %f, %f",x_timesec,x_timensec);
			// IMUupdate(msg->angular_velocity.x,
			// 		  msg->angular_velocity.y,
			// 		  msg->angular_velocity.z,
			// 		  msg->linear_acceleration.x,
			// 		  msg->linear_acceleration.y,
			// 		  msg->linear_acceleration.z,
			// 		  x_timesec,
			// 		  x_timensec,
			// 		  0);

			IMUupdate(gx,gy,gz,ax,ay, az,x_timesec,(long)x_timensec,0);


		}
		
	}

	void Lidar720::membuttimu(float &gx,float &gy,float &gz)
	{
		if (templature != -100)
		{
			// fprintf(stderr, "but = %f %f %f", gx, k_bX[0] * (templature - 40) + k_bX[1],templature);
			gx -= k_bX[0] * (templature - 40) + k_bX[1];
			gy -= k_bY[0] * (templature - 40) + k_bY[1];
			gz -= k_bZ[0] * (templature - 40) + k_bZ[1];
			// fprintf(stderr, "  = %f\n", gx);
		}
	}

	void Lidar720::readtxtpath()
	{
		
		std::string l_calibutfipath;

		//std::string l_linpath = config_lidaryaml.Hvalconpath;
		std::string l_linpath = config_lidaryaml.calibutfipath;
		//l_linpath = l_linpath.substr(0, l_linpath.find_last_of(".")) + ".txt";
		l_calibutfipath = ros::package::getPath("vanjee_lidar") + "/cfg/" + l_linpath;
		ROS_INFO("[imu paramter ]%s", l_calibutfipath.c_str());

		ROS_INFO("**IMU**SN**:%s", l_calibutfipath.c_str());
		FILE *fp = NULL;
		unsigned int tmp = 0;
		char data[100];	   //"123,456,789\n";
		std::string l_str; // = data;
		// fprintf(stderr, "SN:%s\n", l_str.c_str());

		fp = fopen(l_calibutfipath.c_str(), "rb");
		if (!fp)
		{
			fprintf(stderr, "open error\n");
			return;
		}
		else
		{
			while (fgets(data, 100, fp))
			{

				l_str = data;
				// fprintf(stderr,"%s\n",l_str.c_str());
				char *p;
				std::string strT, strX, strY, strZ;
				char *by = strtok_r(data, ",", &p);

				if (*by == NULL)
				{
					break;
				}

				l_str = by;
				// fprintf(stderr,"%s\n",l_str.c_str());

				if (l_str == "acc") //|| l_str != "\n")
				{
					l_str = strtok_r(p, ",", &p);
					if (l_str == "X")
					{
						by = strtok_r(p, ",", &p);
						DS_par[0][0] = atof(by);

						by = strtok_r(p, ",", &p);
						DS_par[0][1] = atof(by);
					}

					if (l_str == "Y")
					{
						by = strtok_r(p, ",", &p);
						DS_par[1][0] = atof(by);

						by = strtok_r(p, ",", &p);
						DS_par[1][1] = atof(by);
					}

					if (l_str == "Z")
					{
						by = strtok_r(p, ",", &p);
						DS_par[2][0] = atof(by);

						by = strtok_r(p, ",", &p);
						DS_par[2][1] = atof(by);
					}

					// fprintf(stderr,"2323232323\n");
				}
				else if(l_str == "lin") //|| l_str != "\n")
				{
					l_str = strtok_r(p, ",", &p);
					if (l_str == "X")
					{
						by = strtok_r(p, ",", &p);
						k_bX[0] = atof(by);

						by = strtok_r(p, ",", &p);
						k_bX[1] = atof(by);
					}

					if (l_str == "Y")
					{
						by = strtok_r(p, ",", &p);
						k_bY[0] = atof(by);

						by = strtok_r(p, ",", &p);
						k_bY[1] = atof(by);
					}

					if (l_str == "Z")
					{
						by = strtok_r(p, ",", &p);
						k_bZ[0] = atof(by);

						by = strtok_r(p, ",", &p);
						k_bZ[1] = atof(by);
					}

					
				}//lin or acc
			}
		}

		ROS_INFO("[imu_l_KB][%f:%f ; %f:%f ; %f:%f ;]", DS_par[0][0], DS_par[0][1],DS_par[1][0], DS_par[1][1], DS_par[2][0], DS_par[2][1]);
		ROS_INFO("[imu_a_KB][%f:%f ; %f:%f ; %f:%f ;]", k_bX[0], k_bX[1], k_bY[0], k_bY[1], k_bZ[0], k_bZ[1]);

		fclose(fp);
	}

	void Lidar720::rotateimu(float &gx, float &gy, float &gz, float &ax, float &ay, float &az)
	{
		float rotate_Ag[2] = {gx, gy};
		float rotate_Aa[2] = {ax, ay};

		gx = rotate_Ag[0] * rotate_B[0][0] + rotate_Ag[1] * rotate_B[1][0];
		gy = rotate_Ag[0] * rotate_B[0][1] + rotate_Ag[1] * rotate_B[1][1];

		ax = rotate_Aa[0] * rotate_B[0][0] + rotate_Aa[1] * rotate_B[1][0];
		ay = rotate_Aa[0] * rotate_B[0][1] + rotate_Aa[1] * rotate_B[1][1];
	}

	void Lidar720::IMUZeroDrift(float gx, float gy, float gz, float ax, float ay, float az)
	{
		if (IMUZeroDriftNum >= (IMUZeroDriftALLNum - IMUSlideALLlen))
		{
			IMUSlideArray[0][IMUSlideLen] = gx;
			IMUSlideArray[1][IMUSlideLen] = gy;
			IMUSlideArray[2][IMUSlideLen] = gz;

			IMUSAcdeArray[0][IMUSlideLen] = ax;
		    IMUSAcdeArray[1][IMUSlideLen] = ay;
		    IMUSAcdeArray[2][IMUSlideLen] = az;	

			m_imuXstu.ImuAgv += (gx / IMUSlideALLlen);
			m_imuYstu.ImuAgv += (gy / IMUSlideALLlen);
			m_imuZstu.ImuAgv += (gz / IMUSlideALLlen);  

			m_imuXstu.ImuAccagv += (ax / IMUSlideALLlen);
			m_imuYstu.ImuAccagv += (ay / IMUSlideALLlen);
			m_imuZstu.ImuAccagv += (az / IMUSlideALLlen); 

			// fprintf(stderr,"publish %f   %f   %f %d  %d  %d\n",m_imuXstu.ImuAgv,m_imuYstu.ImuAgv,m_imuZstu.ImuAgv,IMUZeroDriftALLNum,IMUSlideALLlen,IMUZeroDriftNum);
			IMUSlideLen++;
		}

		if (IMUZeroDriftNum < IMUZeroDriftALLNum)
		{
			// fprintf(stderr,"%f   %f   %f\n",ax,ay,az);
			m_imuXstu.ImuZero += gx;
			m_imuYstu.ImuZero += gy;
			m_imuZstu.ImuZero += gz;
			// fprintf(stderr,"%d   %f  %f  %f \n",IMUZeroDriftNum,m_imuXstu.ImuZero,m_imuYstu.ImuZero,m_imuZstu.ImuZero);

			IMUZeroDriftNum++;
		}

		exPre = gx;
		eyPre = gy;
		ezPre = gz;
	}

	int Lidar720::IMUSlidequally(float gx, float gy, float gz, float ax,float ay,float az, int num)
	{
		// fprintf(stderr,"num:%d   (%f;%f;%f)   (%f;%f;%f\n)",num,ax,ay,az,m_imuXstu.ImuAgv,m_imuYstu.ImuAgv,m_imuZstu.ImuAgv);
		float l_x = IMUSlideArray[0][num];
		float l_y = IMUSlideArray[1][num];
		float l_z = IMUSlideArray[2][num];

		IMUSlideArray[0][num] = gx;
		IMUSlideArray[1][num] = gy;
		IMUSlideArray[2][num] = gz;

		m_imuXstu.ImuAgv += (gx - l_x) / IMUSlideALLlen;
		m_imuYstu.ImuAgv += (gy - l_y) / IMUSlideALLlen;
		m_imuZstu.ImuAgv += (gz - l_z) / IMUSlideALLlen;

		float l_ax = IMUSAcdeArray[0][num];
		float l_ay = IMUSAcdeArray[1][num];
		float l_az = IMUSAcdeArray[2][num];

		IMUSAcdeArray[0][num] = ax;
		IMUSAcdeArray[1][num] = ay;
		IMUSAcdeArray[2][num] = az;		

		m_imuXstu.ImuAccagv += (ax - l_ax) / IMUSlideALLlen;
		m_imuYstu.ImuAccagv += (ay - l_ay) / IMUSlideALLlen;
		m_imuZstu.ImuAccagv += (az - l_az) / IMUSlideALLlen;

		//return 0;
		return 3;
		// fprintf(stderr,"num:%d   (%f;%f;%f)   (%f;%f;%f)\n\n",num,ax,ay,az,m_imuXstu.ImuAgv,m_imuYstu.ImuAgv,m_imuZstu.ImuAgv);
	}

	void Lidar720::initDataXY(float *data_x, float *data_y, int data_n)
	{
		float A = 0.0;
		float B = 0.0;
		float C = 0.0;
		float D = 0.0;
		float E = 0.0;
		float F = 0.0;

		for (int y = 0; y < data_n; y++)
		{
			data_x[y] = data_x[y] - 40.0;
		}

		for (int i = 0; i < data_n; i++)
		{
			A += data_x[i] * data_x[i];
			B += data_x[i];
			C += data_x[i] * data_y[i];
			D += data_y[i];
		}

		// 计算斜率a和截距b
		float a, b, temp = 0;
		if (temp = (data_n * A - B * B)) // 判断分母不为0
		{
			a = (data_n * C - B * D) / temp;
			b = (A * D - B * C) / temp;
		}
		else
		{
			a = 1;
			b = 0;
		}

		d_k = a;
		d_b = b;
	}

	inline void Lidar720::IMUupdate(float gx, float gy, float gz, float ax, float ay, float az, double timsec, long time, int i) //,float *pitch,float *roll,float *yaw)
	{
		sensor_msgs::Imu imu_data;
		imu_data.header.frame_id = config_lidaryaml.imu_frameid;

		imu_data.header.stamp = ros::Time().fromSec(timsec);

		//membuttimu(gx, gy, gz);
		
		ax = (ax / 1000 * Ki * 9.81) * DS_par[0][1] - DS_par[0][0];
		ay = (ay / 1000 * Ki * 9.81) * DS_par[1][1] - DS_par[1][0];
		az = (az / 1000 * Ki * 9.81) * DS_par[2][1] - DS_par[2][0]; //单位转换DS_par		

		gx = gx / 1000 * Kp * 0.0174533;
		gy = gy / 1000 * Kp * 0.0174533;
		gz = gz / 1000 * Kp * 0.0174533;
		//ROS_INFO("%f %f %f",gx,gy,gz);

		rotateimu(gx, gy, gz, ax, ay, az);
		//ROS_INFO("%f %f %f\n",gx,gy,gz);

		float l_dertatime;
		{
			if (time == timePre)
			{
				// printf("time ***********  %ld    ***** %ld\n",time,timePre);
				return;
			}
			else
			{
				if (timePre <= time)
				{
					l_dertatime = time - timePre;
				}
				else
				{
					l_dertatime = 100000000 - timePre + time;
				}

				l_dertatime = l_dertatime / 100000000;
				timenum++;
				timenum = timenum % 10;
			}

			//ROS_INFO("l_dertatime = %f    %d    %ld", l_dertatime, time, timePre);
			timePre = time;
		}

		//l_dertatime = time / 100000000.0;
		//ROS_INFO("l_dertatime = %f", l_dertatime);
		

		if (IMUZeroDriftNum < (IMUZeroDriftALLNum))
		{
			IMUZeroDrift(gx, gy, gz, ax, ay, az);
			return;
		}
		else if (IMUZeroDriftNum == (IMUZeroDriftALLNum))
		{
			// imu_an::updateZero();//求取艾伦方差
			m_imuXstu.ImuZero = m_imuXstu.ImuZero / IMUZeroDriftNum;
			m_imuYstu.ImuZero = m_imuYstu.ImuZero / IMUZeroDriftNum;
			m_imuZstu.ImuZero = m_imuZstu.ImuZero / IMUZeroDriftNum;

			exInt = 0;
			eyInt = 0;
			ezInt = 0;
			IMUZeroDriftNum++;
			ROS_INFO("[publish imu][%f   %f   %f]", m_imuXstu.ImuZero, m_imuYstu.ImuZero, m_imuZstu.ImuZero);
		}
		else
		{
			IMUSlidequally(gx, gy, gz, ax , ay , az , IMUSlidelenNow);
			IMUSlidelenNow++;
			IMUSlidelenNow = IMUSlidelenNow % IMUSlideALLlen;
		}

		double norm = sqrt(m_imuXstu.ImuAccagv * m_imuXstu.ImuAccagv + m_imuYstu.ImuAccagv * m_imuYstu.ImuAccagv + m_imuZstu.ImuAccagv * m_imuZstu.ImuAccagv);

		// m_imuXstu.ImuAccagv /= norm * 9.81;
		// m_imuYstu.ImuAccagv /= norm * 9.81;
		// m_imuZstu.ImuAccagv /= norm * 9.81;

		// ax = m_imuXstu.ImuAccagv;
		// ay = m_imuYstu.ImuAccagv;
		// az = m_imuZstu.ImuAccagv;

		gx = (gx - m_imuXstu.ImuZero);
		gy = (gy - m_imuYstu.ImuZero);
		gz = (gz - m_imuZstu.ImuZero);
		imu_data.angular_velocity.x = gx; // / 0.0174533;//m_imuXstu.ImuAgv;//gx; // 1000;
		imu_data.angular_velocity.y = gy; // / 0.0174533;//m_imuYstu.ImuAgv;//gy; // 1000;
		imu_data.angular_velocity.z = gz; // / 0.0174533;//m_imuZstu.ImuAgv;//gz; // 1000;
		//ROS_INFO("%f %f %f\n",gx,gy,gz);

		// ax = ax - m_imuXstu_a.ImuZero;
		// ay = ay - m_imuYstu_a.ImuZero;
		// az = az - m_imuZstu_a.ImuZero;
		imu_data.linear_acceleration.x = ax; // m_imuXstu.ImuAgv;//gx; // 1000;
		imu_data.linear_acceleration.y = ay; // m_imuYstu.ImuAgv;//gy; // 1000;
		imu_data.linear_acceleration.z = az; // m_imuZstu.ImuAgv;//gz; // 1000;

		const geometry_msgs::Vector3 &a = imu_data.linear_acceleration;
		const geometry_msgs::Vector3 &w = imu_data.angular_velocity;

		//ROS_INFO("l_dertatime = %f   %f   %d" , l_dertatime , timsec , time);
		

		m_fiter.update(a.x, a.y, a.z, w.x, w.y, w.z, l_dertatime, qa0, qa1, qa2, qa3);
		m_fiter.getOrientation(qa0, qa1, qa2, qa3);

		imu_data.angular_velocity.x = w.x;	  // / 0.0174533; // m_imuXstu.ImuAgv;//gx; // 1000;
		imu_data.angular_velocity.y = w.y;	  // / 0.0174533; // m_imuYstu.ImuAgv;//gy; // 1000;
		imu_data.angular_velocity.z = w.z;	  // / 0.0174533; // m_imuZstu.ImuAgv;//gz; // 1000;
		// imu_data.linear_acceleration.x = a.x; // 1000;
		// imu_data.linear_acceleration.y = a.y; // 1000;
		// imu_data.linear_acceleration.z = a.z; // 1000;  

		imu_data.linear_acceleration.x = m_imuXstu.ImuAccagv; // 1000;
		imu_data.linear_acceleration.y = m_imuYstu.ImuAccagv; // 1000;
		imu_data.linear_acceleration.z = m_imuZstu.ImuAccagv; // 1000; 

		
		//   float g1,g2,g3,g4,g5;
		//   g1 = 2.0 * ((qa1 * qa3) - (qa0 * qa2));
		//   g2 = 2.0 * (qa0 * qa1 + qa2*qa3);
		//   g3 = qa0*qa0 - qa1*qa1 - qa2*qa2 + qa3*qa3;
		//   g4 = 2.0 * (qa1 * qa2 + qa0 * qa3);
		//   g5 = qa0*qa0 + qa1*qa1 - qa2*qa2 - qa3*qa3;
		//   imu_data.linear_acceleration.x = 0 - asinf(g1) * 57.3; // 1000;
		//   imu_data.linear_acceleration.y = atanf(g2/g3) * 57.3; // 1000;
		//   imu_data.linear_acceleration.z = atan2f(g4,g5) * 57.3; // 1000;

		tf2::Quaternion q;
		q.setRPY(exInt, eyInt, ezInt);
		imu_data.orientation.x = qa1;
		imu_data.orientation.y = qa2;
		imu_data.orientation.z = qa3;
		imu_data.orientation.w = qa0;

		output_pointcloudIMU.publish(imu_data);
		//ROS_INFO("publish imu");
	}

} // namespace vanjee_rawdata
