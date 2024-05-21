#!/usr/bin/env python
#-*-coding:utf-8-*-

import rospy, math
import numpy as np
import os
from geometry_msgs.msg import PoseWithCovarianceStamped

class BuildNodes:

  def __init__(self):
    self.error_sub = rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, self.NodesCallback, queue_size = 1)
    self.this_node = PoseWithCovarianceStamped()
    self.this_node_id = 0
    self.this_node_map_id = 0
    self.this_node_characteristic = 0
    self.yaw = 0

  def NodesCallback(self, msg):
    rospy.loginfo("Received a new goal.")
    self.ReSet()
    self.GenerateNewNodeId()
    self.this_node = msg
    self.yaw = self.GetYaw(msg)
    self.WriteToNode()
    self.WriteToNodePub()
    rospy.loginfo("Write over.")

  def GenerateNewNodeId(self):
    node_id_list = []
    data = np.loadtxt('../cfg/nodes.txt', usecols=0)
    data = data.tolist()

    if isinstance(data,(str, float, int)) == True:
      node_id_list.append(int(data))
    else:
      node_id_list = data

    if len(node_id_list) != 0:
      self.this_node_id = int(node_id_list[-1]) + 1
      Flag = True
      while Flag:
        if len(node_id_list) == 1:
          Flag = False
        for i in range(0, len(node_id_list) - 1):
          if self.this_node_id == node_id_list[i]:
            self.this_node_id += 1
            break
          else:
            Flag = False
    else:
      self.this_node_id +=1

  def WriteToNode(self):
    file_handle = open('../cfg/nodes.txt',mode='a+')
    file_handle.write('\n')
    file_handle.write(str(self.this_node_id) + " " + str(self.this_node_map_id) + " " + str(self.this_node_characteristic) + " " + str(self.this_node.pose.pose.position.x) + " " + str(self.this_node.pose.pose.position.y) + " " + str(self.yaw))
    file_handle.write('\n')
    file_handle.close()

  def WriteToNodePub(self):
    file_handle = open('../cfg/nodes_pub.txt',mode='a+')
    file_handle.write('\n')
    file_handle.write(str(self.this_node.pose.pose.position.x) + " " + str(self.this_node.pose.pose.position.y))
    file_handle.write('\n')
    file_handle.close()

  def GetYaw(self, data):
    w = data.pose.pose.orientation.w
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z

    y = math.atan2(2*(w*z+x*y),1-2*(z*z+y*y))
    return y

  def ReSet(self):
    self.this_node = PoseWithCovarianceStamped()
    self.this_node_id = 0
    self.this_node_map_id = 0
    self.this_node_characteristic = 0
    self.yaw = 0

  def GenerateNewNode(self):
    rospy.loginfo("Received a new goal.")
    self.GenerateNewNodeId()
    self.WriteToFile()
    rospy.loginfo("Write over.")



if __name__ == '__main__': 
  try:
    rospy.init_node("build_nodes", anonymous=True)
    build_nodes = BuildNodes()
    rospy.loginfo("build nodes begin!")
    #build_nodes.GenerateNewNode()
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
