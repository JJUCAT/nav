# 需要绑定的端口信息
配置新的机器人需要设置相应的端口
- RTK serial: /dev/ttyUSB* --> serial: /dev/rtk
- Front Livox LiDAR ip:192.168.1.164 --> ip:front_lidar
- Rear Livox LiDAR ip:192.168.1.166 --> ip:rear_lidar
- Top Livox LiDAR ip:192.168.1.168 --> ip:top_lidar

修改完配置文件后，ROS驱动程序使用的端口号可以固定下来，不需要重新配置。

# 修改方法
## 绑定串口
### 修改udev规则
udev规则文件在/etc/udev/rules.d/目录下，文件名以数字开头，数字越小，优先级越高，数字越大，优先级越低。所以我们可以在该目录下新建一个文件，命名为99-rtk.rules，内容如下：
```
RTK配置，这里是绑定了usb转串口模块的VID和PID
KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE="0666", SYMLINK+="rtk"
```

### 重启udev服务
```
sudo service udev reload
sudo service udev restart
```

## 绑定网口
### 修改hosts文件
hosts文件在/etc/目录下，文件名为hosts，内容如下：
```
192.168.1.164 front_lidar
192.168.1.166 rear_lidar
192.168.1.168 top_lidar
```
