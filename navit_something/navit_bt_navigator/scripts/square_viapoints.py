#!/usr/bin/env python
import rospy, math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def publish_via_points_msg():
  pub = rospy.Publisher('/via_points', Path, queue_size=1)
  rospy.init_node("via_points_node")

  length = 7.0
  num = 60
  offset = 20

  via_points_msg = Path() 
  via_points_msg.header.stamp = rospy.Time.now()
  via_points_msg.header.frame_id = "odom" # CHANGE HERE: odom/map

  inc = np.linspace(0, length, num)
  val = np.ones(num)

  square_x = []
  square_y = []

  square_x = np.concatenate((0*val, -inc, -length*val, inc-length) ,axis=0) 
  square_y = np.concatenate((inc, length*val, length-inc, 0*val) ,axis=0) 

  # print(square_x)
  # print(square_y)

  for i in range(len(square_x)-1) :
    point_tmp = PoseStamped()
    point_tmp.header.frame_id = "odom" # CHANGE HERE: odom/map

    point_tmp.pose.position.x = square_x[i] + offset;
    point_tmp.pose.position.y = square_y[i] + offset;
    point_tmp.pose.position.z = 0;



    # tf2::Quaternion q;
    # q.setRPY(0, 0, math.atan2( square_y[i+1] -  square_y[i], square_x[i+1] - square_x[i]));
    # tf2::convert(q, pose.pose.orientation);
        

    via_points_msg.poses.append(point_tmp);

  r = rospy.Rate(1) # 10hz
  t = 0.0
  while not rospy.is_shutdown():
        
    pub.publish(via_points_msg)
    
    r.sleep()



if __name__ == '__main__': 
  try:
    publish_via_points_msg()
  except rospy.ROSInterruptException:
    pass

