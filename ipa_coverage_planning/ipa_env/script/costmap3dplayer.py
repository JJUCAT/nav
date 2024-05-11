#! /usr/bin/python3
# code: UTF-8

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped

import copy
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

import plotly.graph_objs as go
import plotly.io as pio
# -------------------- global --------------------

g_recv_map = False
g_costmap_topic = OccupancyGrid()



# -------------------- 3dplayer --------------------
def get_height(map_topic, X, Y):
  """将 2d 地图话题数据转为 3d 曲面
  地图话题数据中, -1 表示未知区域, [0, 100] 表示到障碍物距离越来越近
  这里需要把 -1 转为 100 以上

  Args:
      map_topic (OccupancyGrid): 地图话题数据
      X (list): 地图栅格 x 坐标矩阵
      Y (list): 地图栅格 y 坐标矩阵

  Returns:
      list: Z 坐标矩阵
  """  
  print("<----- get height ----->")
  # 初始化 Z 维度大小
  # shape 返回矩阵的行列 (n,m)：n 行 m 列
  Z = np.zeros((X.shape[0], X.shape[1]))
  print("get height, Z (w %d, h %d), Z (w %d, h %d), Z (w %d, h %d)" %
        (X.shape[0], X.shape[1], Y.shape[0], Y.shape[1], Z.shape[0], Z.shape[1]))

  # 遍历 X 和 Y 中的每个元素
  for i in range(Z.shape[1]):
    for j in range(Z.shape[0]):
      x = X[i, j]
      y = Y[i, j]
      index = x * map_topic.info.width + y
      value = map_topic.data[index]
      # print("get height, [%f, %f], index:%d, value:%d" % (x, y, index, value))   
      if value < 0:
        value = 100
      Z[i,j] = value
      # print("Z :%f, raw:%f" % (Z[i,j], value/100))
  return Z



def display_test():
  xx = np.arange(0, 6, 1)
  yy = np.arange(0, 6, 1)
  X, Y = np.meshgrid(xx, yy)
  Z = np.sin(X)+np.cos(Y) # Z 轴是关于 x , y 的函数
  print("get height, Z (w %d, h %d), Z (w %d, h %d), Z (w %d, h %d)" %
        (X.shape[0], X.shape[1], Y.shape[0], Y.shape[1], Z.shape[0], Z.shape[1]))
  print('X:')
  print(X)
  print('Y:')
  print(Y)
  print('Z:')
  print(Z)
  return X, Y, Z



def display_map(map_topic):
  xx = np.arange(0, map_topic.info.width, 1)
  yy = np.arange(0, map_topic.info.height, 1)
  X, Y = np.meshgrid(xx, yy)
  Z = get_height(map_topic, X, Y)
  return X, Y, Z



def matplotlib_display(map_topic):
  """matplotlib 绘制 3d 曲面，不可交互

  Args:
      map_topic (OccupancyGrid): 地图话题数据
  """  
  fig = plt.figure()
  ax3 = plt.axes(projection='3d')
  plt.rcParams['font.sans-serif']=['FangSong'] # 用来正常显示中文标签
  plt.rcParams['axes.unicode_minus']=False # 用来正常显示负号

  # X, Y, Z = display_test() # test
  X, Y, Z = display_map(map_topic) # 地图数据
  ax3.plot_surface(X,Y,Z,cmap='rainbow') # 设置曲面颜色 @viridis @rainbow @autumn @afmhot @jet @nipy_spectral
  plt.show()



def plotlylib_display(map_topic):
  """plotlylib 绘制 3d 曲面，网页端显示，可交互

  Args:
      map_topic (OccupancyGrid): 地图话题数据
  """  
  X, Y, Z = display_map(map_topic) # 地图数据
  surface = go.Surface(x=X, y=Y, z=Z, colorscale='Rainbow', showscale=True)
  data = [surface]
  layout = go.Layout(title='3D Surface Plot', autosize=True, margin=dict(l=0, r=0, b=0, t=0),
                     scene=dict(aspectmode='data', # 坐标轴比例用正常数据的大小显示
                                # hover 鼠标悬停数据显示的配置
                                hovermode=False,
                                xaxis=dict(title='X Axis'),
                                yaxis=dict(title='Y Axis'),
                                zaxis=dict(title='Z Axis')
                                ))
  fig = go.Figure(data=data, layout=layout)
  pio.show(fig)



# -------------------- ros --------------------

def callback(data):
  global g_recv_map, g_costmap_topic
  if g_recv_map == False:
    g_recv_map = True
    g_costmap_topic = data
    print("Received costmap2d message")
    # matplotlib_display(g_costmap_topic)
    plotlylib_display(g_costmap_topic)



def listener():
  rospy.init_node('costmap_listener', anonymous=True)
  rospy.Subscriber("/ipa_test_node/test_costmap/costmap", OccupancyGrid, callback)
  rospy.spin()



if __name__ == '__main__':
  listener()


