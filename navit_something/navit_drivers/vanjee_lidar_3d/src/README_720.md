## 1 驱动包简介

**示例使用版本vanjee_lidar_v1.10.2驱动包是万集科技在Ubuntu环境下的雷达驱动软件包，仅供WLR-720/721系列型号雷达使用。


## 2 依赖介绍

### 2.1 ROS 

在ROS环境下使用雷达驱动，需要安装ROS相关依赖库。
+ Ubuntu 16.04 - ROS Kinetic desktop
+ Ubuntu 18.04 - ROS Melodic desktop
+ Ubuntu 20.04 - ROS Noetic desktop

安装方法请参考 http://wiki.ros.org。

**强烈建议安装ROS desktop-full版。这个过程会自动安装一些兼容版本的依赖库，如PCL库等。这样可以避免花大量时间，去逐个安装和配置它们。**

### 2.2 Yaml (必需)

版本号:  >= v0.5.2 

**若已安装ROS desktop-full, 可跳过**

安装方法如下:

```sh
sudo apt-get update
sudo apt-get install -y libyaml-cpp-dev
```

### 2.3 libpcap (必需)

版本号： >= v1.7.4

安装方法如下：

```sh
sudo apt-get install -y  libpcap-dev
```

## 3 编译、运行

### 3.1 依赖于ROS-catkin编译

解压驱动包后进入 vanjee_lidar_v1.10.2 工作空间目录执行以下命令即可编译、运行。

```sh
catkin_make
source devel/setup.bash
roslaunch vanjee_lidar vanjee.launch
```

## 4 参数介绍

### 4.1 config.yaml 文件参数介绍 文件路径：/vanjee_lidar_v1.10.2/src/config/config.yaml

```sh
  lidar:
  - driver:
      lidartype: wlr720                                      #雷达型号 支持 wlr720 wlr721 （选填）
      frame_id: wlr_720                                      #驱动frame_id，支持修改
      min_angle: 0                                           #点云显示起始角度， value="0" （0-360）支持起始角度＞终止角
      max_angle: 360                                         #点云显示终止角度， value="0" （0-360）支持起始角度＞终止角
      min_distance: 0.2                                      #点云显示最近距离，不支持修改
      max_distance: 200                                      #点云显示最远距离，不支持修改
      pcap: nothing                                          #pcap数据包播包路径（直连雷达显示点云时此处需必须默认为nothing）
      read_fast: true                                        #读包速率：true时，pcap包正常速率播包；false时，pcap包慢速率播包
      read_once: false                                       #读PCAP包：true时，pcap包只读一次；false时，pcap包循环读取
      repeat_delay: 0.0                                      #读包延时，不建议修改
      rmp: 0                                                 #电机转速，不支持修改（0.1°=5HZ=300rmp；0.2°=10HZ=600rmp；0.4°=20HZ=1200rmp）
      data_length: 1200                                      #后期删掉
      time_mode: true                                        #时间模式：true时，使用雷达时间；false时，使用系统时间（精确到微秒级）
      axisinversion: false                                   #坐标系翻转：true时，点云Z轴翻转；false时，点云Z轴不翻转
      axisoffset: 0                                          #坐标系旋转：绕+Z轴顺时针旋转（0-360 单位/度）
      calibration: Vanjee16-carside3455.csv                  #雷达垂直角分辨率表，运行驱动自动读取，不支持修改
      calibutfipath: SN720C223400030055.txt                  #雷达IMU温漂表，运行驱动自动读取，不支持修改
    ros_topic:
      ros_recv_packet_topic: vanjee_packets                  #原始数据点云包
      ros_send_imu_topic: wlr_720/imu                        #雷达IMU话题名,支持修改
      ros_send_point_cloud_topic: wlr_720/cloud_points       #雷达点云话题名，支持修改
    proto:
      lidar_ip: 192.168.2.86                                 #雷达ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_ip: 192.68.2.88                                   #目标ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_port: 3001                                        #目标端口，支持修改（需使用点云软件修改雷达ip后再修改此处）

## vanjee_lidar_v1.10.2 驱动文件支持多雷达运行，运行多雷达需将/vanjee_lidar_v1.10.2/src/config/config.yaml中其它雷达信息补充完整##
```

## 5 使用进阶-在线多雷达使用（理论上最多支持8台雷达同时解析）

### 5.1 config.yaml 文件参数介绍 文件路径：/vanjee_lidar_v1.10.2/src/config/config.yaml

```sh
  lidar:
  - driver:
      lidartype: wlr720                                      #雷达型号 支持 wlr720 wlr721 （选填）
      frame_id: wlr_720                                      #驱动frame_id，支持修改
      min_angle: 0                                           #点云显示起始角度， value="0" （0-360）支持起始角度＞终止角
      max_angle: 360                                         #点云显示终止角度， value="0" （0-360）支持起始角度＞终止角
      min_distance: 0.2                                      #点云显示最近距离，不支持修改
      max_distance: 200                                      #点云显示最远距离，不支持修改
      pcap: nothing                                          #pcap数据包播包路径（直连雷达显示点云时此处需必须默认为nothing）
      read_fast: true                                        #读包速率：true时，pcap包正常速率播包；false时，pcap包慢速率播包
      read_once: false                                       #读PCAP包：true时，pcap包只读一次；false时，pcap包循环读取
      repeat_delay: 0.0                                      #读包延时，不建议修改
      rmp: 0                                                 #电机转速，不支持修改（0.1°=5HZ=300rmp；0.2°=10HZ=600rmp；0.4°=20HZ=1200rmp）
      data_length: 1200                                      #后期删掉
      time_mode: true                                        #时间模式：true时，使用雷达时间；false时，使用系统时间（精确到微秒级）
      axisinversion: false                                   #坐标系翻转：true时，点云Z轴翻转；false时，点云Z轴不翻转
      axisoffset: 0                                          #坐标系旋转：绕+Z轴顺时针旋转（0-360 单位/度）
      calibration: Vanjee16-carside3455.csv                  #雷达垂直角分辨率表，运行驱动自动读取，不支持修改
      calibutfipath: SN720C223400030055.txt                  #雷达IMU温漂表，运行驱动自动读取，不支持修改
    ros_topic:
      ros_recv_packet_topic: vanjee_packets                  #原始数据点云包
      ros_send_imu_topic: wlr_720/imu                        #雷达IMU话题名,支持修改
      ros_send_point_cloud_topic: wlr_720/cloud_points       #雷达点云话题名，支持修改
    proto:
      lidar_ip: 192.168.2.86                                 #雷达ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_ip: 192.68.2.88                                   #目标ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_port: 3001                                        #目标端口，支持修改（需使用点云软件修改雷达ip后再修改此处）

  - driver:                                                  ###多台雷达显示点云在需文本后继续增添- driver:...即可（不同雷达ip需设置不同）
     ...
     ...
     ...
    proto:
      lidar_ip: 192.168.2.85                                 #雷达ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_ip: 192.68.2.88                                   #目标ip，支持修改（需使用点云软件修改雷达ip后再修改此处）
      dest_port: 3000                                        #目标端口，支持修改（需使用点云软件修改雷达ip后再修改此处）

## 注：需要解析多台雷达在线显示时仅需将雷达配置参数- driver:...在文本继续往下粘贴复制即可（一台雷达对应一个- driver:...）##
```