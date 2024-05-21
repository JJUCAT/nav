# 2 参数介绍

```vanjee_lidar_sdk```读取配置文件 ```config.yaml```，得到所有的参数。```config.yaml```在```vanjee_lidar_sdk/config```文件夹中。 

**config.yaml遵循YAML格式。该格式对缩进有严格要求。修改config.yaml之后，请确保每行开头的缩进仍保持一致！**

```config.yaml```包括两部分：```common```部分 和 ```lidar```部分。 

```vanjee_lidar_sdk```支持多个雷达。```common```部分为所有雷达共享。```lidar```部分，每一个子节点对应一个雷达，针对这个雷达的实际情况分别设置。



## 2.1 common部分

```common```部分设置雷达消息的源（Packet或点云从哪来）和目标（Packet或点云发布到哪去）。

```yaml
common:
  msg_source: 1                                   # 1: 消息来源于在线雷达
                                                  # 2: 消息来源于 pcap
  send_point_cloud_ros: true                      # true: 将点云发送到ROS以便查看      
  send_imu_packet_ros: true                       # true: 将IMU发送到ROS以便查看       
  send_laser_scan_ros: true                       # true: 发送极坐标点云的话题到ROS以便查看,只支持单线雷达             
```

- ```msg_source```

  - 1 -- 连接在线雷达。更多使用细节，请参考[连接在线雷达并发送点云到ROS](../howto/06_how_to_decode_online_lidar_CN.md)。

  - 2-- 离线解析PCAP包。更多使用细节，请参考[离线解析PCAP包并发送点云到ROS](../howto/08_how_to_decode_pcap_file_CN.md)。

- ```send_laser_scan_ros```

   - true -- 雷达极坐标点云的消息将通过ROS发出 

     *只支持单线雷达。
   
- ```send_point_cloud_ros```

   - true -- 雷达点云消息将通过ROS发出 

   *点云消息的类型为ROS官方定义的点云类型sensor_msgs/PointCloud2, 用户可以使用Rviz直接查看点云。用户可以录制ROS的点云包，但点云包的体积非常大，所以不建议这么做。更好的方式是录制Packet包，请参考send_packet_ros=true的情况。*

 



## 2.2 ```lidar```部分

```lidar```部分根据每个雷达的实际情况进行设置。

```yaml
lidar:
  - driver:
      lidar_type: vanjee_720                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                          # 接收点云数据的主机端口号
      lidar_msop_port: 3333                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      wait_for_difop: true                          # 是否等角度标定表参数导入
      min_distance: 0.5                             # 点云最小距离
      max_distance: 120                             # 点云最大距离
      use_lidar_clock: true                         # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径
      pcap_repeat: true                             # 是否循环播放pcap
      pcap_rate: 10                                 # pcap播放速度
      config_from_file: false                       # 从配置文件内获取参数
      angle_path_ver:                               # 垂直角度配置文件地址
      angle_path_hor:                               # 水平角度配置文件地址
      imu_param_path:                               # imu参数配置文件地址
      dense_points: false                           # 输出的点云中是否剔除标记为NAN的点
      ts_first_point: false                         # 点云的时间戳是否第一个点的时间 true=第一个点的时间，false-最后一个点的时间
      publish_mode: 0                               # 回波模式 0-发布第一重，1-发布第二重；2-发布两重；
      group_address: 0.0.0.0                        # 组播地址
      host_address: 192.168.2.88                    # 接收点云数据的主机IP地址
      lidar_address: 192.168.2.86                   # 雷达IP地址
      x: 0                                          # x方向偏移量
      y: 0                                          # y方向偏移量
      z: 0                                          # z方向偏移量
      roll: 0                                       # 横滚角偏移量 
      pitch: 0                                      # 俯仰角偏移量
      yaw: 0                                        # 航向角偏移量
    use_vlan: false                                 # PCAP文件中的Packet是否包含VLAN层
    ros:
      ros_frame_id: vanjee_lidar                            # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720         # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets  # Topic used to send imu through ROS
   
```

- ```lidar_type```

  支持的雷达型号在```vanjee_lidar_sdk```的```README```文件中列出。

- ```host_msop_port```

  接收扫描数据Packet的端口号。 *若收不到消息，请优先确认这个参数是否配置正确。*

   ```lidar_msop_port```

  发送扫描数据Packet的端口号,也即激光雷达接收查询指令的端口号。 *若收不到消息，请优先确认这个参数是否配置正确。*

- ```start_angle```, ```end_angle```

  点云消息的起始角度和结束角度。这个设置是软件屏蔽，将区域外的点设置为0。 start_angle和end_angle的范围是0~360°，**起始角可以大于结束角**.

- ```wait_for_difop```
  当连接在线雷达时，如果设置为true，在收到激光雷达的角度表之前，不解析点云数据；当读取PCAP文件时，该参数默认为false。**719/719c无角度标定表 默认false**

- ```min_distance```, ```max_distance```

  点云的最小距离和最大距离。这个设置是软件屏蔽，会将区域外的点设置为0，不会减小每帧点云的体积。

- ```use_lidar_clock```

  - true -- 使用雷达时间作为消息时间戳。
  - false -- 使用电脑主机时间作为消息时间戳。 

- ```dense_points ```

  输出的点云中是否剔除标记为NAN的点,默认值为false。
  - true   剔除标记为NAN的点,
  - false  不剔除标记为NAN的点。

- ```pcap_path```

   pcap包的路径。当 msg_source=3 时有效。

- ```pcap_repeat``` -- 默认值为true， 用户可将其设置为false来禁用pcap循环播放功能。

- ```pcap_rate``` -- 默认值为1，该值与频率无明确对应关系，但可通过调节参数值改变播放频率。 用户可调节此参数来控制pcap播放速度，设置的值越大，pcap播放速度越快。

- ```config_from_file``` -- 默认值为false, 是否从外参文件读入雷达配置信息。

- ```angle_path_ver``` -- 角度配置文件的路径,定义和保存激光雷达各通道(垂直角度)/(垂直角度和初始水平角度)。 文件路径:/src/vanjee_lidar_sdk/param

- ```angle_path_hor``` -- 角度配置文件的路径,定义和保存激光雷达各通道水平角度。 文件路径:/src/vanjee_lidar_sdk/param

- ```imu_param_path``` -- imu参数配置文件路径,定义和保存激光雷达imu配置参数。文件路径:/src/vanjee_lidar_sdk/param

- ```ts_first_point``` -- 默认值为false。点云的时间戳是否第一个点的时间。```true```为第一个点的时间，```false```为最后一个点的时间。

- ```publish_mode``` -- 回波模式 0-发布第一重，1-发布第二重；2-发布两重；

- ```group_address``` -- 如果雷达为组播模式，此参数需要被设置为组播的地址。具体使用方式可以参考[在线雷达 - 高级主题](../howto/07_online_lidar_advanced_topics_CN.md) 。

- ```host_address``` -- 如果主机上通过多个端口接收多个雷达的数据，则可以将此参数指定为雷达的目标IP；如果设置了group_address，那也需要设置host_address，以便将这个IP地址的网卡加入组播组
- ```lidar_address``` -- LiDAR端发送数据的IP地址。

- ```x, y, z, roll, pitch, yaw ``` -- 坐标变换参数，若启用了内核的坐标变换功能，将会使用此参数输出经过变换后的点云。x, y, z, 单位为```米```, roll, pitch, yaw, 单位为```角度```。具体使用方式可以参考 [坐标变换功能](../howto/10_how_to_use_coordinate_transformation_CN.md) 。

- ```use_vlan``` -- 如果PCAP文件中的Packet包含VLAN层，可以指定```use_vlan```=```true```，跳过这一层。


## 2.3 示例

### 2.3.1 单台雷达

在线连接1台```WLR-720```雷达，并发送点云到ROS。

```yaml
common:
  msg_source: 1                                   # 0: not use lidar 
                                                  # 1: 消息来源于在线雷达
                                                  # 2: 消息来源于 pcap
  send_point_cloud_ros: true                      # true: 将点云发送到ROS以便查看
  send_imu_packet_ros: true                       # true: 将IMU发送到ROS以便查看
lidar:
  - driver:
      lidar_type: vanjee_720                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                          # 接收点云数据的主机端口号
      lidar_msop_port: 3333                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      wait_for_difop: true                          # 是否等角度标定表参数导入
      min_distance: 0.5                             # 点云最小距离
      max_distance: 120                             # 点云最大距离
      use_lidar_clock: true                         # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径
      pcap_repeat: true                             # 是否循环播放pcap
      pcap_rate: 10                                 # pcap播放速度
      config_from_file: false                       # 从配置文件内获取参数
      angle_path_ver:                               # 垂直角度配置文件地址
      angle_path_hor:                               # 水平角度配置文件地址
      imu_param_path:                               # imu参数配置文件地址
      dense_points: false                           # 输出的点云中是否剔除标记为NAN的点
      ts_first_point: false                         # 点云的时间戳是否第一个点的时间 true=第一个点的时间，false-最后一个点的时间
      publish_mode: 0                               # 回波模式 0-发布第一重，1-发布第二重；2-发布两重；
      group_address: 0.0.0.0                        # 组播地址
      host_address: 192.168.2.88                    # 接收点云数据的主机IP地址
      lidar_address: 192.168.2.86                   # 雷达IP地址
      x: 0                                          # x方向偏移量
      y: 0                                          # y方向偏移量
      z: 0                                          # z方向偏移量
      roll: 0                                       # 横滚角偏移量 
      pitch: 0                                      # 俯仰角偏移量
      yaw: 0                                        # 航向角偏移量
    use_vlan: false                                 # PCAP文件中的Packet是否包含VLAN层
    ros:
      ros_frame_id: vanjee_lidar                            # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720         # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets  # Topic used to send imu through ROS 

     
```

### 2.3.2 两台雷达

在线连接1台```WLR-719```雷达和1台```WLR-720```雷达，发送点云到ROS。

*注意```lidar```部分参数的缩进*

```yaml
common:
  msg_source: 1                                   # 0: not use lidar 
                                                  # 1: 消息来源于在线雷达
                                                  # 2: 消息来源于 pcap
  send_point_cloud_ros: true                      # true: 将点云发送到ROS以便查看
  send_imu_packet_ros: true                       # true: 将IMU发送到ROS以便查看
  send_laser_scan_ros: false                      # true: 发送极坐标点云的话题到ROS以便查看,只支持单线雷达
lidar:
  - driver:
      lidar_type: vanjee_719                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3002                          # 接收点云数据的主机端口号
      lidar_msop_port: 6050                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      wait_for_difop: false                         # 是否等角度标定表参数导入
      min_distance: 0.2                             # 点云最小距离
      max_distance: 50                              # 点云最大距离
      use_lidar_clock: true                         # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径
      pcap_repeat: true                             # 是否循环播放pcap
      pcap_rate: 10                                 # pcap播放速度
      config_from_file: false                       # 从配置文件内获取参数
      angle_path_ver:                               # 垂直角度配置文件地址
      angle_path_hor:                               # 水平角度配置文件地址
      dense_points: false                           # 输出的点云中是否剔除标记为NAN的点
      ts_first_point: false                         # 点云的时间戳是否第一个点的时间 true=第一个点的时间，false-最后一个点的时间
      publish_mode: 0                               # 回波模式 0-发布第一重，1-发布第二重；2-发布两重；
      group_address: 0.0.0.0                        # 组播地址
      host_address: 192.168.0.110                   # 接收点云数据的主机IP地址
      lidar_address: 192.168.0.2                    # 雷达IP地址
      x: 0                                          # x方向偏移量 m
      y: 0                                          # y方向偏移量 m
      z: 0                                          # z方向偏移量 m
      roll: 0                                       # 横滚角偏移量 °
      pitch: 0                                      # 俯仰角偏移量 °
      yaw: 0                                        # 航向角偏移量 °
      use_vlan: false                               # PCAP文件中的Packet是否包含VLAN层
    ros:
      ros_frame_id: vanjee_lidar                       # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points719    # Topic used to send point cloud through ROS
      ros_send_laser_scan_topic: /vanjee_scan719       # Topic used to send laser scan through ROS
  - driver:
      lidar_type: vanjee_720                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                          # 接收点云数据的主机端口号
      lidar_msop_port: 3333                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      wait_for_difop: true                          # 是否等角度标定表参数导入
      min_distance: 0.5                             # 点云最小距离
      max_distance: 120                             # 点云最大距离
      use_lidar_clock: true                         # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径
      pcap_repeat: true                             # 是否循环播放pcap
      pcap_rate: 10                                 # pcap播放速度
      config_from_file: false                       # 从配置文件内获取参数
      angle_path_ver:                               # 垂直角度配置文件地址
      angle_path_hor:                               # 水平角度配置文件地址
      imu_param_path:                               # imu参数配置文件地址
      dense_points: false                           # 输出的点云中是否剔除标记为NAN的点
      ts_first_point: false                         # 点云的时间戳是否第一个点的时间 true=第一个点的时间，false-最后一个点的时间
      publish_mode: 0                               # 回波模式 0-发布第一重，1-发布第二重；2-发布两重；
      group_address: 0.0.0.0                        # 组播地址
      host_address: 192.168.2.88                    # 接收点云数据的主机IP地址
      lidar_address: 192.168.2.86                   # 雷达IP地址
      x: 0                                          # x方向偏移量 m
      y: 0                                          # y方向偏移量 m
      z: 0                                          # z方向偏移量 m
      roll: 0                                       # 横滚角偏移量 °
      pitch: 0                                      # 俯仰角偏移量 °
      yaw: 0                                        # 航向角偏移量 °
    use_vlan: false                                 # PCAP文件中的Packet是否包含VLAN层
    ros:
      ros_frame_id: vanjee_lidar                            # Frame id of packet message and point cloud message
      ros_send_point_cloud_topic: /vanjee_points720         # Topic used to send point cloud through ROS
      ros_send_imu_packet_topic: /vanjee_lidar_imu_packets_720  # Topic used to send imu through ROS 
```

