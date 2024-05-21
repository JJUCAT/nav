# 用于室外作业的机器人任务地图

# 地图的原点坐标，UTM坐标系
geometry_msgs/PoseStamped map_origin

# 充电桩的位置，UTM坐标系
geometry_msgs/PoseStamped dock_pose

# 任务区域
GeoJsonArea[] work_areas

# 障碍物区域
GeoJsonArea[] no_go_zones

# 任务区域之间的连接路线, 用于连接不同的任务区域
GeoJsonPath[] channels

# 任务路径, 用于指定机器人的任务路径
GeoJsonPath[] work_paths
