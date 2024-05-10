#! /usr/bin/python3
# code: UTF-8

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from nav_msgs.srv import GetMap
from geometry_msgs.msg import PoseStamped
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import matplotlib.pyplot as plt

def display():
  fig = plt.figure()
  ax3 = plt.axes(projection='3d')

  plt.rcParams['font.sans-serif']=['FangSong'] # 用来正常显示中文标签
  plt.rcParams['axes.unicode_minus']=False # 用来正常显示负号

  #定义三维数据
  xx = np.arange(-5,5,0.5)
  yy = np.arange(-5,5,0.5)
  X, Y = np.meshgrid(xx, yy)
  Z = np.sin(X)+np.cos(Y)

  #作图
  ax3.plot_surface(X,Y,Z,cmap='rainbow') 
  # 改变cmap参数可以控制三维曲面的颜色组合, 一般我们见到的三维曲面就是 rainbow 的

  plt.show()


recv_map = False
costmap_topic=OccupancyGrid()

def callback(data):
  if recv_map == False:
    recv_map = True
    costmap_topic = data
    print("Received costmap2d message")
    display()

def listener():
    # 初始化节点
    rospy.init_node('costmap_listener', anonymous=True)
    # 订阅costmap2d话题
    rospy.Subscriber("/ipa_test_node/test_costmap/costmap", OccupancyGrid, callback)
    # 进入循环，等待消息
    rospy.spin()

if __name__ == '__main__':
    listener()
