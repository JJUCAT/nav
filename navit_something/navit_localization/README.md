navit_localization
==================

navit_localization is a package of nonlinear state estimation nodes. The package was developed by Charles River Analytics, Inc.

Please see documentation here: http://docs.ros.org/noetic/api/navit_localization/html/index.html

启动ekf节点定位服务

roslaunch navit_localization online_localization_server.launch
启动后默认使用rkt融合定位，如若需要切换到融合激光定位则需调用以下服务

rosservice call /switch_localization "use_lidar: true
use_rtk: false"

需要使用哪种定位就把切换服务中对应的标志位设为true，另一个设为false，都设为true时融合激光和rtk定位（未实现）

使用激光融合定位时，需要开启激光定位，流程如下

roslaunch hdl_localization online_mid360_hdl_localization.launch

启动后需要切图和重定位来进行激光定位，重定位依赖rtk数据，需要再较为空旷的地方进行
依次调用下面两个服务

rosservice call /hdl_map_server "time:
  data:
    secs: 0
    nsecs: 0
command: ''
map_path: '/home/robot/Public/data/'
map_name: 'shenzhen_wanke'"

rosservice call /relocalize