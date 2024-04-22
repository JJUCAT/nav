#!/usr/bin/env python3
import sys
import rospy
import actionlib
import tf
import math
import copy
import numpy as np
from navit_msgs.msg import NavigateThroughPathAction, NavigateThroughPathGoal, NavigateThroughPathFeedback
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

client = actionlib.SimpleActionClient("/navigate_through_path", NavigateThroughPathAction)
goal = NavigateThroughPathGoal()

def genSinglePosePath(p):
    path = Path()
    path.header = p.header
    path.poses.append(p)
    return path

def genLinearPath(p):
    q = p.pose.orientation
    (r, pp, y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    path = Path()
    path.header = p.header    
    for i in np.arange(0.0, 10.0, 0.5):
      # pose = p # 这样是引用
      pose = copy.deepcopy(p) # 这样是赋值
      rospy.loginfo("linear pose [%.3f,%.3f], p [%.3f,%.3f]" %(pose.pose.position.x, pose.pose.position.y, p.pose.position.x, p.pose.position.y))
      pose.pose.position.x += i * math.cos(y)
      pose.pose.position.y += i * math.sin(y)
      path.poses.append(pose)
      rospy.loginfo("linear path cos:%.3f, sin:%.3f" %(math.cos(y), math.sin(y)))
      rospy.loginfo("linear path size:%u, [%.3f][%.3f,%.3f]" %(len(path.poses), i, pose.pose.position.x, pose.pose.position.y))
    return path

def genZigzagPath(p):
    q = p.pose.orientation
    (r, pp, y) = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
    path = Path()
    path.header = p.header
    length = 10.0
    step = 0.5
    intervel = 2.0    
    row_num = 5
    pose = copy.deepcopy(p)
    # path.poses.append(pose) # 这里还是引用，最后列表所有元素都是同一个 pose
    path.poses.append(copy.deepcopy(pose)) # 放起点
    for i in range(0, row_num):
      dir = 0 if (i&1)==0 else math.pi
      turn = math.pi/2 # if (i&1)==0 else -math.pi/2
      rospy.loginfo("row[%u], dir %.3f, turn %.3f" %(i, dir, turn))
      rospy.loginfo("length %.3f, step %.3f, intervel %.3f" %(length, step, intervel))
      for j in np.arange(step, length+step, step): # row
        row_y = y+dir
        pose.pose.position.x += step * math.cos(row_y)
        pose.pose.position.y += step * math.sin(row_y)
        orient = tf.transformations.quaternion_from_euler(0, 0, row_y)
        pose.pose.orientation.x = orient[0]
        pose.pose.orientation.y = orient[1]
        pose.pose.orientation.z = orient[2]
        pose.pose.orientation.w = orient[3]
        # path.poses.append(pose) # 这里还是引用，最后列表所有元素都是同一个 pose
        path.poses.append(copy.deepcopy(pose))
        rospy.loginfo("row step [%.3f], [%.3f, %.3f]" %(j, pose.pose.position.x, pose.pose.position.y))
      for k in np.arange(step, intervel+step, step): # column
        col_y = y+turn
        pose.pose.position.x += step * math.cos(col_y)
        pose.pose.position.y += step * math.sin(col_y)
        orient = tf.transformations.quaternion_from_euler(0, 0, col_y)
        pose.pose.orientation.x = orient[0]
        pose.pose.orientation.y = orient[1]
        pose.pose.orientation.z = orient[2]
        pose.pose.orientation.w = orient[3]
        path.poses.append(copy.deepcopy(pose))
        rospy.loginfo("col step [%.3f], [%.3f, %.3f]" %(k, pose.pose.position.x, pose.pose.position.y))

    for p in path.poses:
      rospy.loginfo("path p [%.3f, %.3f]" %(p.pose.position.x, p.pose.position.y))
    return path

def navigateThroughPathCallback(state, result):
    rospy.loginfo("approach dock is Done")

def navigateThroughPathFeedbackCallback(fb):
    rospy.loginfo("approach dock feedback ...")
    rospy.loginfo("status_code %u, status_msg %s" %(fb.status_code, fb.status_msg))
    rospy.loginfo("distance2goal %f, current vel %f" %(fb.distance_to_goal, fb.current_vel))
    rospy.loginfo("completion percentage %f, num of recoveries %u" %(fb.completion_percentage, fb.num_of_recoveries))

def recvGoal(p):
    rospy.loginfo("recv goal [%.3f, %.3f]", p.pose.position.x, p.pose.position.y)
    # goal.path = genSinglePosePath(p)
    # goal.path = genLinearPath(p)
    goal.path = genZigzagPath(p)
    rospy.loginfo("send <navigate through path> goal, path size %u ..." %(len(goal.path.poses)))
    client.send_goal(goal, done_cb=navigateThroughPathCallback, active_cb=None, feedback_cb=navigateThroughPathFeedbackCallback)

if __name__ == "__main__":
    rospy.init_node("navigate_through_path_sim_node")
    rospy.loginfo("navigate through path sim node start.")
    rospy.loginfo("Connecting to server </navigate_through_path> ...")
    client.wait_for_server()
    rospy.loginfo("Connected.")
    goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, recvGoal, queue_size=1)
    path_pub = rospy.Publisher("/navigate_through_sim_path", Path, queue_size=1)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if len(goal.path.poses)!=0:
          path_pub.publish(goal.path)
        rate.sleep()
    rospy.spin()
