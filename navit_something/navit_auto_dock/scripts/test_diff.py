#!/usr/bin/env python3
import math
import rospy
import actionlib
import tf2_ros
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from navit_msgs.msg import ApproachDockAction, ApproachDockGoal, FinalDockAction, FinalDockGoal
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Bool


ACTION_NAME = ["/approach_dock", "/final_dock"]
controller_plugins = ["GracefulController","FgController","DiffPurepursuitController"]
perception_plugins = ["fake_perception","2d_lidar","visual_perception"]
filter_plugins = ["ekf_1","average"]

client = actionlib.SimpleActionClient(ACTION_NAME[0], ApproachDockAction)
final_dock_client = actionlib.SimpleActionClient(ACTION_NAME[1], FinalDockAction)

cout_success_num=0
dock_done_=False
goal_final = FinalDockGoal()
feed_back=PoseStamped()
goal_final.dock_pose.header.frame_id = "odom"
ori_=Quaternion()

def feedback(msg):
    global feed_back
    feed_back.pose=msg.dock_pose.pose
    feed_back.header.frame_id=msg.dock_pose.header.frame_id

    
def doneCallback(state, result):
    global goal_final
    global feed_back
    approach_res=client.get_result()
    if approach_res.reached:
       rospy.loginfo("approach dock is Done")
    else :
       rospy.loginfo("approach dock failed, cancel the final action")
       dock_done_=True
       return
    orientation=(feed_back.pose.orientation.x,
                feed_back.pose.orientation.y,
                feed_back.pose.orientation.z,
                feed_back.pose.orientation.w)
    (roll,pitch,theta) = euler_from_quaternion(orientation)
    theta_=theta+math.pi

    rospy.logwarn("the orientation is: %0.6f",theta)
    goal_final.dock_pose.header.frame_id= feed_back.header.frame_id
    goal_final.dock_pose.pose.position.x = feed_back.pose.position.x+(math.cos(theta)* -0.6)
    goal_final.dock_pose.pose.position.y = feed_back.pose.position.y+(math.sin(theta)* -0.6)

    (x,y,z,w)=quaternion_from_euler(roll,pitch,theta_)
    goal_final.dock_pose.pose.orientation.x=x
    goal_final.dock_pose.pose.orientation.y=y
    goal_final.dock_pose.pose.orientation.z=z
    goal_final.dock_pose.pose.orientation.w=w
    goal_final.rotate_in_place = False
    final_dock_client.send_goal_and_wait(goal_final)
    result=final_dock_client.get_result()
    ## judge the result
    if result.docked :
      rospy.loginfo("all done!")
      cout_success_num+=1
      dock_done_=True
    else :
      rospy.loginfo("charge failed!")
      dock_done_=True

    return
    

if __name__ == "__main__":
    rospy.init_node("dock_script")
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[0])
    client.wait_for_server()
    rospy.loginfo("Connecting to %s..." % ACTION_NAME[1])

    auto_dock_result_pub = rospy.Publisher("/auto_dock_action_succeed",Bool,queue_size=10)

    cout_success_num=0
    final_dock_client.wait_for_server()

    goal_approach = ApproachDockGoal()
    count_should_success=0
    for i in range(len(controller_plugins)):
      for j in range(len(perception_plugins)):
        for k in range(len(filter_plugins)):
          count_should_success+=1
          dock_done_=False
          goal_approach.filter_plugin_name=filter_plugins[k]
          goal_approach.perception_plugin_name=perception_plugins[j]
          goal_approach.controller_plugin_name=controller_plugins[i]
          goal_approach.expected_dock_pose.header.frame_id = "odom"
          goal_approach.expected_dock_pose.pose.position.x=0.0
          goal_approach.expected_dock_pose.pose.position.y=0.0
          goal_approach.expected_dock_pose.pose.position.z=0.0
          goal_approach.expected_dock_pose.pose.orientation.x=0.0
          goal_approach.expected_dock_pose.pose.orientation.y=0.0
          goal_approach.expected_dock_pose.pose.orientation.z=0.0
          goal_approach.expected_dock_pose.pose.orientation.w=0.0
          client.send_goal(goal_approach, done_cb=doneCallback, active_cb=None, feedback_cb=feedback)
          while(not dock_done_):
            rospy.sleep(1.0)

          if(cout_success_num!=count_should_success):
            rospy.logerrer("auto_dock failed by %d,[%f,%f,%f]",count_should_success,)

    re_=Bool()
    re_.data=True

    auto_dock_result_pub.publish(re_)

    rospy.spin()
