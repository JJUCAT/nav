# the ray imu for LINS354
## Description
Raymond Robot imu driver for ROS.

## How to start 
1. Insert the IMU device, check the USB device id and set up the USB permissions, like:
```
sudo chmod 777 /dev/ttyUSB0
```

2. Install rosserial
```
sudo apt-get install ros-kinetic-serial
```

3. Create a new workspace and put the rm_imu package in your workspace, build your workspace and setting the environment variable,for example:
```
cd catkin_ws
catkin_make
source /devel/setup.bash
```

4. Remap the device serial port(ttyUSBX) to LINS354.
```
sudo cp /scripts/rm_imu.rules /etc/udev/rules.d
sudo service udev reload
sudo service udev restart
```

5. Set coreect configuration parameters in the launch file
```
<param name="port" value="/dev/rm_imu" />
<param name="imu_baudrate" value="115200" />
<param name="imu_frame_id" value="imu_base" />
<param name="imu_topic" value="imu" />
```
You can set these params to meet your needs.

6. Insert the IMU device and run the driver.
```
roslaunch rm_imu rm_imu.launch
```

7. Query the imu data
```
rostopic echo /imu
```

## How to simulation
1. rviz view to test imu
```
sudo apt-get install git-core
```
Download the stack from our repository into your catkin workspace
```
git clone https://github.com/ccny-ros-pkg/imu_tools.git
```
Compile the stack:
```
cd ~/catkin_ws
catkin_make
roslaunch rm_imu view_rm_imu.launch
```
The rviz_imu_plugin package is used to display sensor_msgs/Imu messages in rviz. Once you download and compile the package, it should be visible as a plugin.
It displays the orientation of the IMU using a box as well as and coordinate axes. The acceleration can be visualized using a vector.
