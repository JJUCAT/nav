

# 8 如何解码PCAP文件



## 8.1 简介

本文档展示如何解码PCAP文件, 并发送点云数据到ROS。 

在阅读本文档之前，请确保已阅读雷达用户手册和 [参数简介](../intro/02_parameter_intro_CN.md) 。



## 8.2 步骤

### 8.2.1 获取数据的端口号

请参考雷达用户手册，或者使用第三方工具（WireShark等）抓包，得到雷达的目标端口。端口的默认值分别为```3001```。

### 8.2.2 设置参数文件

设置参数文件```config.yaml```。

#### 8.2.2.1 common部分

```yaml
common:
  msg_source: 2                                   # 1: 消息来源于在线雷达
                                                  # 2: 消息来源于 pcap
  send_point_cloud_ros: true                      # true: 将点云发送到ROS以便查看                     
```

消息来自PCAP包，所以设置 ```msg_source = 2``` 。

将点云发送到ROS以便查看，所以设置 ```send_point_cloud_ros = true``` 。 

#### 8.2.2.2 lidar-driver部分

```yaml
lidar:
  - driver:
      lidar_type: vanjee_721                        # LiDAR类型
      connect_type: 1                               # 连接方式 1-udp  2-tcp
      host_msop_port: 3001                          # 接收点云数据的主机端口号
      lidar_msop_port: 3333                         # 雷达端口号
      start_angle: 0                                # 点云起始角度
      end_angle: 360                                # 点云结束角度
      min_distance: 0.3                             # 点云最小距离
      max_distance: 45                              # 点云最大距离
      use_lidar_clocks: false                       # True: 使用雷达时间作为消息时间戳
                                                    # False: 使用电脑主机时间作为消息时间戳
      pcap_path:                                    # pcap文件绝对路径  
```

将```pcap_path``` 设置为PCAP文件的全路径。
将 ```lidar_type``` 设置为LiDAR类型。
设置 ```host_msop_port``` 为雷达数据的目标端口号，这里是```3001```。

#### 8.2.2.3 lidar-ros部分

```yaml
    ros:
      ros_frame_id: vanjee_lidar                       # 坐标系
      ros_send_point_cloud_topic: /vanjee_points721    # 点云话题名
```

将 ```ros_send_point_cloud_topic``` 设置为发送点云的话题，这里是 ```/vanjee_points721 ```。 

### 8.2.3 运行

运行程序。
