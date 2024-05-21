# navit_collision_monitor

### 说明
navit_collision_monitor 包包含两个功能：
1. CollisionDetector 障碍物检测
2. CollisionMonitor 检测障碍物接管cmd_vel以防止碰撞。

### CollisionDetector
1. CollisionDetector 用于从各中传感器如 LaserScan、PointCloud2、Range获取障碍物数据。
2. 用户可以定义一块或多块区域，CollisionDetector将检测区域是否有障碍物。

### CollisionMonitor
1. CollisionMonitor也会进行区域的障碍物检测。
2. 检测到障碍物后，CollisionMonitor会接管cmd_vel以防止碰撞。

