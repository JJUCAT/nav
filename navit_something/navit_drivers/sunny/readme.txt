# TOF DEV SDK ROS节点封装使用说明


## 1 安装ROS

关键事项：
    在Ubuntu 18.04系统下的实际安装的经验，安装ROS前，先要进行如下操作：
        【1】先将电脑的“软件和更新”-“Ubuntu 软件”-“下载自”下拉选项卡里的内容改到mirrors.aliyun.com，不然可能会提示“连接失败 [IP: 91.189.91.39 80]”；
        【2】 然后需要将电脑的“软件和更新”-“更新”选项卡的更新打开（不要去点“还原”按钮），让系统进行一次全面的更新；
        【3】执行完sudo apt install ros-melodic-desktop-full后，不需要执行sudo rosdep init，rosdep update，不然之前安装的ros命令又会被卸载掉；       


根据自己的操作系统安装指定的ROS版本：
    （1）Ubuntu 16.04系统下推荐安装ROS Kinetic Kame，详细教程见官网：http://wiki.ros.org/cn/kinetic/Installation/Ubuntu 
    （2）Ubuntu 18.04系统下推荐安装ROS Melodic Morenia，详细教程见官网：http://wiki.ros.org/cn/melodic/Installation/Ubuntu
    （3）Ubuntu 20.04系统下推荐安装ROS Noetic Ninjemys，详细教程见官网：http://wiki.ros.org/cn/noetic/Installation/Ubuntu


## 2 创建ROS工作空间

    详见教程见官网：http://wiki.ros.org/cn/ROS/Tutorials/InstallingandConfiguringROSEnvironment
    

## ==================================
## 以下步骤以工作空间“~/catkin_ws”为例。
## ==================================


将“source ~/catkin_ws/devel/setup.bash”添加到“~/.bashrc”文件末尾，这样可以避免每次开启终端都需要输入source命令。
添加后效果例如：
source /opt/ros/melodic/setup.bash
source ~/catkin_ws/devel/setup.bash


## 3 编译和安装demo

复制“dev-ros.tar.gz”到“~/catkin_ws/src”；
将“dev-ros.tar.gz”解压到“~/catkin_ws/src”目录下，如：tar -zxvf dev-ros.tar.gz；
必须根据实际系统账户情况，修改tof_dev_sdk_demo-ros-publisher.cpp的以下源码来指定SDK所需的parameter文件夹路径：
	"/home/sunny/catkin_ws/devel/lib/tof_dev_sdk_demo/parameter";
	
另外，可以根据实际系统账户情况，修改以下源码，来重新指定用于存放SDK日志的文件：
	"./tof_dev_sdk_log.txt"
	
	

通过终端进入“~/catkin_ws”目录；
依次执行以下命令：
catkin_make 
catkin_make install
此时可在“~/catkin_ws/devel/lib/tof_dev_sdk_demo”目录下看到生成的程序文件publisher_demo，libtof_dev_sdk.so，parameter


## 4 使用demo

### 4.1 运行roscore
新开一个终端1，执行以下命令：
roscore



### 4.2 运行demo
新开一个终端2，执行以下命令：
rosrun tof_dev_sdk_demo publisher_demo


#### 4.2.1 执行成功后应该可以看到如下输出：

        *********************start test*************************
        libusb_version: V1.0.24.11584-rc()-describe(http://libusb.info).
        SDK Version: V4.4.39 build 20220412 11:02:49.
        ....
        recved calib data again, skip.
        ...
        [1], one TOF frame, time=9144644, center depthZ = 2.530 m.
        [2], one TOF frame, time=9144750, center depthZ = 2.516 m.
        ...



#### 4.2.2 publisher_demo 软件会在两个topic下发布内容，使用“rostopic list”可以查看当前所有的topic。

        新开一个终端，执行以下命令：
        $ rostopic list
        /rosout
        /rosout_agg      
        /sunny_topic/camera_info                        # publisher_demo 软件发布的相机信息
        /sunny_topic/rgb_frame/rgb                   # publisher_demo 软件发布的RGB信息
        /sunny_topic/tof_frame/depthz             # publisher_demo 软件发布的点云Z值信息（垂线深度值）
        /sunny_topic/tof_frame/pointcloud    # publisher_demo 软件发布的点云信息
        /sunny_topic/tof_frame/rgbd                 # publisher_demo 软件发布的RgbD信息

执行以下命令,可看到camerainfo信息：
        $ rostopic echo /sunny_topic/camera_info




## 5 验证demo

### 5.1 查看实时图像方式

使用rviz查看publisher_demo 软件发布的图像。

新开一个终端3，执行以下命令：
rviz

### 5.1.1 查看实时深度图像
依次点击“Add”->“By topic”->“sunny_topic”->“tof_frame”->“depthz”->“Image”->“OK”
如此，可在左下角看到深度图像；

### 5.1.2 查看实时点云图像
依次点击“Add”->“By topic”->“sunny_topic”->“tof_frame”->“pointcloud”->“PointCloud2”->“OK”
修改“Display”->“Global Options”->“Fixed Frame”的值为“camera_point_cloud_frame”
如此，可在中间画面看到点云图像；

