#!/usr/bin/env python3
import sys
import rospy
import actionlib
from navit_msgs.msg import ApproachDockAction, ApproachDockGoal, ApproachDockFeedback, FinalDockAction, FinalDockGoal
from geometry_msgs.msg import PoseStamped

approach_cnt = 0
final_finish = False
dock = PoseStamped()

# yaw 0.17 is [0.000000,0.000000,0.084898,0.996390]
# yaw 0.34 is [0.000000,0.000000,0.169182,0.985585]
# yaw 0.51 is [0.000000,0.000000,0.252245,0.967663]
# yaw -0.17 is [0.000000,0.000000,-0.084898,0.996390]
# yaw -0.34 is [0.000000,0.000000,-0.169182,0.985585]
# yaw -0.51 is [0.000000,0.000000,-0.252245,0.967663]


ACTION_NAME = ["/approach_dock", "/final_dock"]
client = actionlib.SimpleActionClient(ACTION_NAME[0], ApproachDockAction)
final_dock_client = actionlib.SimpleActionClient(ACTION_NAME[1], FinalDockAction)
goal = ApproachDockGoal()
goal.controller_plugin_name = "FgController"
goal.perception_plugin_name = "fake_perception" # fake_perception # 2d_lidar
goal.filter_plugin_name = "lkf" # lkf # average
goal.percepted_pose_offset = 2.0 # 预回充点的充电桩的负方向多远 [m] # 车头对桩 3.0 # 车尾对桩 2.0
goal.expected_dock_pose.header.frame_id = "odom"
goal.expected_dock_pose.pose.position.x = 8.0
goal.expected_dock_pose.pose.position.y = 5.0
goal.expected_dock_pose.pose.orientation.x = 0.0
goal.expected_dock_pose.pose.orientation.y = 0.0
goal.expected_dock_pose.pose.orientation.z = 0.0
goal.expected_dock_pose.pose.orientation.w = 0.1

goal_ = FinalDockGoal()
goal_.dock_pose.header.frame_id = "odom"
goal_.dock_pose.pose.position.x = 8.0
goal_.dock_pose.pose.position.y = 5.0
goal_.dock_pose.pose.orientation.w = 1.0
goal_.rotate_in_place = False


def approachDoneCallback(state, result):
    rospy.loginfo("approach dock is Done")
    # final_dock_client.send_goal_and_wait(goal_) # ???
    global approach_cnt
    approach_cnt += 1
    rospy.loginfo("approach cnt %d", approach_cnt)
    return

def approachFeedbackCallback(fb):
    # rospy.loginfo("approach dock feedback ...")
    goal_.dock_pose = fb.dock_pose
    # goal_.dock_pose.header.frame_id = "odom"

def finalDoneCallback(state, result):
    rospy.loginfo("final dock is Done")
    final_finish = True
    return

if __name__ == "__main__":
    rospy.init_node("dock_script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[0])
    client.wait_for_server()
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[1])
    final_dock_client.wait_for_server()

    rospy.loginfo("Sending approach dock goal...")
    client.send_goal(goal, done_cb=approachDoneCallback, active_cb=None, feedback_cb=approachFeedbackCallback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rospy.loginfo("approach cnt %d", approach_cnt)
        if approach_cnt == 1:
          approach_cnt += 1
          client.send_goal(goal, done_cb=approachDoneCallback, active_cb=None, feedback_cb=approachFeedbackCallback)
        if approach_cnt == 3:
          rospy.loginfo("Sending final dock goal...")
          approach_cnt = 0
          final_dock_client.send_goal(goal_, done_cb=finalDoneCallback, active_cb=None, feedback_cb=None)
        rate.sleep()
    rospy.spin()
