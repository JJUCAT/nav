# 定义模块
define laser ranger
(
  sensor
  (
    # 雷达距离
    range_max 8 
    # 雷达范围
    fov 360.0 
    # 采样数
    samples 720 
  )
  color "black"
  size [ 0.06 0.15 0.03 ]
)

# 定义机器，带定位功能
define carlike_robot position
(
  # 
  pose [ 0.0 0.0 0.0 0.0 ]

  # odom 定位，[x误差，y误差，航向协方差]
  odom_error [0.03 0.03 999999 999999 999999 0.02]

  size [ 0.6 0.25 0.40 ]

  # correct center of rotation:
  origin [ 0.0 0.0 0.0 0.0]

  gui_nose 1

  #颜色
  color "red"

  # 驱动模型 @diff:差速轮 @car:阿克曼 @omni:全向轮
  drive "car"

  # 前后轮距
  wheelbase 1.07
  
  # 搭载模块，传感器
  laser(pose [ -0.1 0.0 -0.11 0.0 ])
)
