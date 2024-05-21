#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy, math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class PidPlot:

  def __init__(self, error_topic_name, cmd_vel_topic_name, smooth_cmd_vel_topic_name, odom_topic_name, reset_topic_name):

    self.initialized = False
    self.error = 0.0
    self.x1 = 0
    self.x1_axes = []
    self.y1_axes = []

    self.result = 0.0
    self.y2_axes = []
    self.x2 = 0
    self.x2_axes = []

    self.y3_axes = []
    self.x3 = 0
    self.x3_axes = []

    self.y4_axes = []
    self.x4 = 0
    self.x4_axes = []

    self.y5_axes = []
    self.x5 = 0
    self.x5_axes = []

    self.y6_axes = []
    self.x6 = 0
    self.x6_axes = []

    self.y7_axes = []
    self.x7 = 0
    self.x7_axes = []

    self.zero1_axes = []
    self.zero2_axes = []
    self.zero3_axes = []
    self.zero4_axes = []
    self.zero5_axes = []
    self.zero6_axes = []
    self.zero7_axes = []
    self.zero = 0.0
    self.error_sub = rospy.Subscriber(error_topic_name, Float64, self.pid_error_callback, queue_size = 1)
    self.reset_sub = rospy.Subscriber(reset_topic_name, Float64, self.pid_reset_callback, queue_size = 1)
    self.cmd_vel_sub = rospy.Subscriber(cmd_vel_topic_name, Twist, self.cmd_vel_callback, queue_size = 1)
    self.smooth_cmd_vel_sub = rospy.Subscriber(smooth_cmd_vel_topic_name, Twist, self.smooth_cmd_vel_callback, queue_size = 1)
    self.odom_sub = rospy.Subscriber(odom_topic_name, Odometry, self.odom_callback, queue_size = 1)

    rospy.loginfo("Plotting pid results published on '%s'.",error_topic_name)

  def pid_error_callback(self, data):
    self.initialized = True
    rospy.loginfo_once("First message received.")
    self.error = data.data
    self.y1_axes.append(self.error)
    self.x1 += 1
    self.x1_axes.append(self.x1)
    self.zero1_axes.append(self.zero)

  def cmd_vel_callback(self,data):
    self.initialized = True
    self.y2_axes.append(data.angular.z)
    self.x2 += 1
    self.x2_axes.append(self.x2)
    self.zero2_axes.append(self.zero)

    self.y4_axes.append(data.linear.x)
    self.x4 += 1
    self.x4_axes.append(self.x4)
    self.zero4_axes.append(self.zero)

  def smooth_cmd_vel_callback(self,data):
    self.initialized = True
    self.y3_axes.append(data.angular.z)
    self.x3 += 1
    self.x3_axes.append(self.x3)
    self.zero3_axes.append(self.zero)

    self.y5_axes.append(data.linear.x)
    self.x5 += 1
    self.x5_axes.append(self.x5)
    self.zero5_axes.append(self.zero)

  def odom_callback(self,data):
    if (data.twist.twist.linear.x != 0.000 and data.twist.twist.linear.x != 0.000 and self.initialized == True):
      self.y6_axes.append(data.twist.twist.linear.x)
      self.x6 += 1
      self.x6_axes.append(self.x6)
      self.zero6_axes.append(self.zero)

      self.y7_axes.append(data.twist.twist.angular.z)
      self.x7 += 1
      self.x7_axes.append(self.x7)
      self.zero7_axes.append(self.zero)

  def pid_reset_callback(self, data):
    if (data.data == 1.0):
      self.reset()
    elif(data.data == 2.0):
      self.keep_plot()

  def keep_plot(self):
    self.initialized = False

  def reset(self):
    self.error = 0.0
    self.x1 = 0
    self.x1_axes = []
    self.y1_axes = []

    self.result = 0.0
    self.y2_axes = []
    self.x2 = 0
    self.x2_axes = []

    self.y3_axes = []
    self.x3 = 0
    self.x3_axes = []

    self.y4_axes = []
    self.x4 = 0
    self.x4_axes = []

    self.y5_axes = []
    self.x5 = 0
    self.x5_axes = []

    self.y6_axes = []
    self.x6 = 0
    self.x6_axes = []

    self.y7_axes = []
    self.x7 = 0
    self.x7_axes = []

    self.zero1_axes = []
    self.zero2_axes = []
    self.zero3_axes = []
    self.zero4_axes = []
    self.zero5_axes = []
    self.zero6_axes = []
    self.zero7_axes = []
    self.zero = 0.0
    plt.ioff()

  def plot(self, rate):
    plt.clf()
    plt.suptitle("Stanley Control", fontsize=20)
    agraphic=plt.subplot(5,1,1)
    agraphic.set_title("Trajectory Offset")
    agraphic.set_xlabel("Time",fontsize=10)
    agraphic.set_ylabel("Trajectory Offset",fontsize=10)
    plt.plot(self.x1_axes, self.y1_axes,"r-")
    plt.plot(self.x1_axes, self.zero1_axes,"g-")

    # bgraphic=plt.subplot(4,1,2)
    # bgraphic.set_title("Cmd Angular Z")
    # bgraphic.set_xlabel("Time",fontsize=5)
    # bgraphic.set_ylabel("Cmd Angular Z",fontsize=5)
    # plt.plot(self.x2_axes, self.y2_axes,"r-")
    # plt.plot(self.x2_axes, self.zero2_axes,"g-")

    bgraphic=plt.subplot(5,1,2)
    bgraphic.set_title("Cmd Angular Z")
    bgraphic.set_xlabel("Time",fontsize=10)
    bgraphic.set_ylabel("Cmd Angular Z",fontsize=10)
    plt.plot(self.x3_axes, self.y3_axes,"r-")
    plt.plot(self.x3_axes, self.zero3_axes,"g-")

    bgraphic=plt.subplot(5,1,3)
    bgraphic.set_title("Odom Angular Z")
    bgraphic.set_xlabel("Time",fontsize=10)
    bgraphic.set_ylabel("Odom Angular Z",fontsize=10)
    plt.plot(self.x7_axes, self.y7_axes,"r-")
    plt.plot(self.x7_axes, self.zero7_axes,"g-")

    # bgraphic=plt.subplot(4,1,3)
    # bgraphic.set_title("Cmd Linear X")
    # bgraphic.set_xlabel("Time",fontsize=5)
    # bgraphic.set_ylabel("Cmd Linear X",fontsize=5)
    # plt.plot(self.x4_axes, self.y4_axes,"r-")
    # plt.plot(self.x4_axes, self.zero4_axes,"g-")

    bgraphic=plt.subplot(5,1,4)
    bgraphic.set_title("Cmd Linear X")
    bgraphic.set_xlabel("Time",fontsize=10)
    bgraphic.set_ylabel("Cmd Linear X",fontsize=10)
    plt.plot(self.x5_axes, self.y5_axes,"r-")
    plt.plot(self.x5_axes, self.zero5_axes,"g-")

    bgraphic=plt.subplot(5,1,5)
    bgraphic.set_title("Odom Linear X")
    bgraphic.set_xlabel("Time",fontsize=10)
    bgraphic.set_ylabel("Odom Linear X",fontsize=10)
    plt.plot(self.x6_axes, self.y6_axes,"r-")
    plt.plot(self.x6_axes, self.zero6_axes,"g-")

    plt.subplots_adjust(wspace = 0, hspace = 0.5)
    plt.pause(0.05)


    
  def start(self, rate):
    r = rospy.Rate(rate) # define rate here
    plt.ion()
    while not rospy.is_shutdown():
      # if(self.initialized == True):
      self.plot(rate)
      r.sleep()


if __name__ == '__main__': 
  try:
    rospy.init_node("pid_result_plotter", anonymous=True)
    error_topic_name = "/error_value"
    reset_topic_name = "/pid_reset"
    cmd_vel_topic_name = "/move_base_cmd_vel"
    smooth_cmd_vel_topic_name = "/cmd_vel"
    odom_topic_name = "/odom"
    plt.rcParams['figure.figsize'] = (8, 16)        # 图像显示大小
    plt.rcParams['axes.unicode_minus'] = False
    plt.rcParams['lines.linewidth'] = 1.0   #设置曲线线条宽度

    result_plotter = PidPlot(error_topic_name, cmd_vel_topic_name, smooth_cmd_vel_topic_name, odom_topic_name, reset_topic_name)
    rate = 10
    result_plotter.start(rate)
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
