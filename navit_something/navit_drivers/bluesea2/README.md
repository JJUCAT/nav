# 蓝海2D激光雷达LDS-50C-C30E（网口）使用方法
## 启动单个激光雷达
1) WIN系统下，使用上位机配置雷达IP和端口、Upload机器IP和端口（当前雷达IP为192.168.1.99，端口为6543；Upload机器IP为192.168.1.99，端口为6668）；
2）在ubuntu系统下，首先使用网线将雷达与主机连接，修改主机IP为上面的Upload机器IP，子网掩码为255.255.255.0，网关为192.168.1.1,然后测试能否ping通雷达ip；接着下载本代码库，新建一个catkin工作空间，将本代码库放入src文件夹，运行catkin_make编译；
3) 对于激光雷达LDS-50C-C30E，需要修改对应launch文件的参数（LDS-50C-C30E.launch），一般需要修改的重要参数有：
    <param name="topic" value="scan"/>  //话题名称
    <param name="frame_id" value="livox_frame" />  //雷达坐标系名称
    <param name="ip" value="192.168.1.99" />  //雷达IP
	其余的参数根据雷达型号和雷达配置进行修改，具体参数请参考launch文件。
4) 运行roslaunch bluesea2 LDS-50C-C30E.launch，即可运行雷达。
5) 打开rviz，添加点云图，选择雷达话题，即可显示雷达点云。

## 启动多个激光雷达
1) 配置与上述类似，注意在WIN上位机中给激光雷达设置好不同的IP和端口号
2) 在Double-LDS-50C-C30E.launch文件中，修改重要的参数，如topic，frame_id，ip，lidar_port，local_port等
3) 运行roslaunch bluesea2 Double-LDS-50C-C30E.launch，即可运行雷达。
4) 打开rviz，添加点云图，选择雷达话题，即可显示雷达点云。

* 下载 Windows、Linux等平台下的SDK开发包及示例程序，请访问：https://github.com/BlueseaLidar/lanhai-driver
* 下载Ros驱动，请访问：https://github.com/BlueSeaLidar/bluesea2

=========================================== 以下为原README.md ==============================================

# bluesea
ROS driver for Lanhai USB 2D LiDAR 

How to build Lanhai ros driver
=====================================================================
    1) Clone this project to your catkin's workspace src folder
    2) Running catkin_make to build 

How to run Lanhai ros node (Serial Port Version)
=====================================================================
1) Copy UDEV rule file : sudo cp src/LHLiDAR.rules /etc/udev/rules.d/
2) or Run : sudo chmod 666 /dev/ttyUSB0 # make usb serial port readable


## if your lidar model is LDS-50C-2 :
* rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=500000 _firmware_version:=2 _output_scan:=true _output_cloud:=true _with_resample:=true _resample_res:=0.5 _unit_is_mm:=true _with_confidence:=true
* or use roslaunch src/bluesea/launch/LDS-50C-2.launch
    
## if your lidar model is LDS-15BDM or LDS-25BDM:
* rosrun bluesea2 bluesea2_node _frame_id:=map _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2 _output_scan:=true _output_cloud:=true _unit_is_mm:=false _with_confidence:=true _raw_bytes:=2
* or use roslaunch src/bluesea2/launch/LDS-15BDM.launch    

3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

How to start/stop LiDAR detection 
=====================================================================
1) resume detection : rosservice call /your_node/start_motor
2) stop detection : rosservice call /your_node/stop_motor

How to run Lanhai ros node (UDP Network Version)
=====================================================================
1) sudo ifconfig eth0:1 192.168.158.200 # add sub net
2) rosrun bluesea2 bluesea2_node _frame_id:=map _type:=udp _lidar_ip:=192.168.158.91 _firmware_version:=2
3) optional : rostopic hz /scan
4) optional : rosrun rviz rviz # 

## if your lidar model is LDS-50C-E :
* use roslaunch src/bluesea/launch/LDS-50C-E.launch


Parameters
=====================================================================
* std::string type; // LiDAR comm type, could be "uart", or "udp"
* std::string platform; // LiDAR hardware platform
* std::string dump;	// file path of dump raw data, for debug

// for serial port comm
* std::string port; // serial port device path
* int baud_rate; // baud rate, -1 : auto detect current baud rate

// for network comm
* std::string lidar_ip; // LiDAR's network address 
* std::string group_ip; // multicast address
* int lidar_port; // lidar's port (UDP)
* int local_port; // ROS machine's port (UDP)

// for intput data format
* bool unit_is_mm; //  true : unit of raw data distance is CM, false: MM
* bool with_confidence; // true: raw data with intensity, false: no intensity
* bool with_checksum; // true : enable packet checksum

// output data type
* bool output_scan; // true: enable output angle+distance mode, false: disable
* bool output_cloud; // true: enable output xyz format data, false : disable
* bool output_360; // true: collect multiple RawData packets (360 degree), then publish
				// false: publish every RawData (36 degree)
* std::string frame_id;	// frame information, could be used for rviz
* bool from_zero; // true : angle range [0 - 360), false: angle range [-180, 180)

// is lidar inverted
* bool inverted; // inverted installed
* bool reversed; // data's angle increment

// angle composate
* bool with_resample; // resample angle resolution
* double resample_res; // 0.5: resample angle resolution @ 0.5 degree 


// output data format
* int normal_size; // abnormal packet (points number < normal_size) will be droped

// angle filter
* bool with_angle_filter ; // true: enable angle filter, false: diable
* double min_angle; // angle filter's low threshold, default value: -pi
* double max_angle; // angle filters' up threashold, default value: pi

* double max_dist;


Dynamic Reconfigure Parameters
=====================================================================
int rpm; // motor's scaning RPM [300, 1500]

command line like this:
rosrun dynamic_reconfigure dynparam set /lidar1/lidar01 "{'rpm':700}"


How to control Lanhai ros node  start  and stop
=====================================================================
* client:      
 			
												   arg1  state    arg2:choose lidar serial number

			start  or stop  one lidar
			rosrun bluesea2  bluesea2_node_client  start/stop     0/1/2/... 
			start or stop   all lidar
			rosrun bluesea2  bluesea2_node_client  start/stop     -1      



	        rosrun bluesea2  bluesea2_node_client switchZone  1    192.168.0.110  //arg1: zone  arg2:ip

* server:     

 			roslaunch bluesea2  xxx.launch



How to enable multiple radars and use only one port
=====================================================================
refer to  dual-LDS-50C-C30E.launch
warming: The following parameters value is must different

	lidar_ip/lidar*_ip  

	lidar_port/lidar*_port

	topic/topic*   
