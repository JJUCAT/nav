#!/usr/bin/env python3
import math
import rospy
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navit_msgs.msg import ApproachDockAction, ApproachDockGoal, FinalDockAction, FinalDockGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion


ACTION_NAME = ["/approach_dock", "/final_dock"]
client = actionlib.SimpleActionClient(ACTION_NAME[0], ApproachDockAction)
final_dock_client = actionlib.SimpleActionClient(ACTION_NAME[1], FinalDockAction)
goal = ApproachDockGoal()
# goal.controller_plugin_name = "FgController"
goal.controller_plugin_name = "DiffPurepursuitController"
goal.perception_plugin_name = "fake_perception"
# goal.perception_plugin_name = "2d_lidar"
# goal.perception_plugin_name = "visual_perception"
goal.filter_plugin_name="average"
# goal.filter_plugin_name="ekf_1"
goal.expected_dock_pose.header.frame_id = "odom"
 

goal_ = FinalDockGoal()
feed_back=PoseStamped()
goal_.dock_pose.header.frame_id = "odom"
ori_=Quaternion()
def feedback(msg):
    global feed_back
    feed_back.pose=msg.dock_pose.pose
    feed_back.header.frame_id=msg.dock_pose.header.frame_id

def DockCallback(msg):
    global goal
    msg_odom=msg
    goal.expected_dock_pose.header.frame_id=msg_odom.header.frame_id
    goal.expected_dock_pose.pose.position.x = msg_odom.pose.position.x
    goal.expected_dock_pose.pose.position.y = msg_odom.pose.position.y
    goal.expected_dock_pose.pose.orientation.w = msg_odom.pose.orientation.w
    goal.expected_dock_pose.pose.orientation.x = msg_odom.pose.orientation.x
    goal.expected_dock_pose.pose.orientation.y = msg_odom.pose.orientation.y
    goal.expected_dock_pose.pose.orientation.z = msg_odom.pose.orientation.z

    rospy.loginfo("Sending dock goal...")
    client.send_goal(goal, done_cb=doneCallback, active_cb=None, feedback_cb=feedback)
    return
    
def doneCallback(state, result):
    global goal_
    global feed_back
    approach_res=client.get_result()
    if approach_res.reached:
       rospy.loginfo("approach dock is Done")
    else :
       rospy.loginfo("approach dock failed, cancel the final action")
       return
    orientation=(feed_back.pose.orientation.x,
                feed_back.pose.orientation.y,
                feed_back.pose.orientation.z,
                feed_back.pose.orientation.w)
    (roll,pitch,theta) = euler_from_quaternion(orientation)
    theta_=theta

    rospy.logwarn("the orientation is: %0.6f",theta)
    goal_.dock_pose.header.frame_id= feed_back.header.frame_id
    goal_.dock_pose.pose.position.x = feed_back.pose.position.x+(math.cos(theta)* -0.3)#0.3 Prechagrging point
    goal_.dock_pose.pose.position.y = feed_back.pose.position.y+(math.sin(theta)* +0.0)

    (x,y,z,w)=quaternion_from_euler(roll,pitch,theta_)
    goal_.dock_pose.pose.orientation.x=x
    goal_.dock_pose.pose.orientation.y=y
    goal_.dock_pose.pose.orientation.z=z
    goal_.dock_pose.pose.orientation.w=w
    goal_.rotate_in_place = False
    final_dock_client.send_goal_and_wait(goal_)
    result=final_dock_client.get_result()
    ## judge the result
    if result.docked :
      rospy.loginfo("all done!")
    else :
      rospy.loginfo("charge failed!")
    return
    

if __name__ == "__main__":
    rospy.init_node("dock_script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[0])
    client.wait_for_server()
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[1])
    final_dock_client.wait_for_server()

    rospy.loginfo("init rviz dock goal")
    rospy.Subscriber("/dock_point",PoseStamped,DockCallback)

    rospy.spin()