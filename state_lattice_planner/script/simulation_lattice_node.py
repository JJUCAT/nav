#! /usr/bin/env python3
# -*- coding: UTF-8 -*-

import rospy
import actionlib
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from navit_msgs.msg import ComputePathAction, ComputePathGoal 
from navit_msgs.msg import FollowPathAction, FollowPathGoal 

planner_path_pub = rospy.Publisher("lattice_path", Path, queue_size=1)
flag_recv_goal = False

# ComputePath 数据处理
class ComputePathData:
    def __init__(self):
        self.goal = ComputePathGoal()
        self.path = Path()

    def setGoal(self, p, plugin):
        self.goal.start = PoseStamped()
        self.goal.use_start = False
        self.goal.goal = p
        self.goal.planner_plugin = plugin
    
    def getGoal(self):
        return self.goal.goal

    def reachGoal(self, px, py):
        # rospy.loginfo("reach goal, goal:[%.3f, %.3f] pose:[%.3f, %.3f]",
        #     self.goal.goal.pose.position.x, self.goal.goal.pose.position.y, px, py)
        if self.goal.goal.pose.position.x == px and\
           self.goal.goal.pose.position.y == py:
          return True
        else:
          return False

    def active_callback(self):
        rospy.loginfo("ComputePath active !")

    def feedback_callback(self):
        rospy.loginfo("ComputePath feedback ...")

    def result_callback(state, result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("ComputePath result succeeded !")
            if result.path_found == True:
                rospy.loginfo("ComputePath find path !")
                global planner_path_pub
                planner_path_pub.publish(result.path)
                # self.path = result.path
            else:
                rospy.loginfo("ComputePath can't find path !")
        else:
            rospy.loginfo("ComputePath result failed !")

cpdata = ComputePathData()
planner_cli = actionlib.SimpleActionClient("/compute_path", ComputePathAction)


def recvGoal(p):
    rospy.loginfo("recv goal [%.3f, %.3f]", p.pose.position.x, p.pose.position.y)
    global flag_recv_goal
    flag_recv_goal = True
    cpdata.setGoal(p, "lattice")
    # global planner_cli
    # planner_cli.send_goal(cpdata.goal, cpdata.result_callback ,cpdata.active_callback, cpdata.active_callback)

robot = PoseWithCovarianceStamped()
def recvRobot(p):
    # rospy.loginfo("recv robot [%.3f, %.3f]", p.pose.pose.position.x, p.pose.pose.position.y)
    global robot
    robot = p


def fp_active_callback():
    rospy.loginfo("FollowPath active ...")

def fp_feedback_callback(feedback):
    rospy.loginfo("FollowPath feedback %f", feedback.completion_percentage)

def fp_result_callback(state, result):
    if state == actionlib.GoalStatus.SUCCEEDED:
        rospy.loginfo("FollowPath result succeeded !")
        rospy.loginfo("code: %s", result.error_code)
    else:
        rospy.loginfo("FollowPath result failed !")

controller_cli = actionlib.SimpleActionClient("/follow_path", FollowPathAction)
def recvLatticePath(p):
    rospy.loginfo("recv lattice path !")
    fp_goal = FollowPathGoal()
    fp_goal.path = p
    fp_goal.controller_plugin = "avoidance"
    controller_cli.send_goal(fp_goal, fp_result_callback ,fp_active_callback, fp_feedback_callback)

if __name__ == "__main__":
    rospy.init_node("simulation_lattice_node")
    rospy.loginfo("planner server wait for server ...")
    # global planner_cli
    planner_cli.wait_for_server()
    controller_cli.wait_for_server()
    rospy.loginfo("servers are ready.")
    goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, recvGoal, queue_size=1)
    robot_sub = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, recvRobot, queue_size=1)
    lattice_sub = rospy.Subscriber("/state_lattice_planner/best_trajectory", Path, recvLatticePath, queue_size=1)
    sim_goal_pub = rospy.Publisher("/sim_goal", PoseStamped, queue_size=1)
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if flag_recv_goal == True:
          if cpdata.reachGoal(robot.pose.pose.position.x, robot.pose.pose.position.y):
              # rospy.loginfo("reach goal !")
              flag_recv_goal = False
          else:
              # rospy.loginfo("pub goal !")
              sim_goal_pub.publish(cpdata.getGoal())
        rate.sleep()
    rospy.spin()
