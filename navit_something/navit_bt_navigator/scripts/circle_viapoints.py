#!/usr/bin/env python
import rospy, math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


def publish_via_points_msg():
  pub = rospy.Publisher('/via_points', Path, queue_size=1)
  rospy.init_node("via_points_node")

  rad = (4.0, 20.0);
  dis = 5;
  xc = rad;
  yc = 0;
  offset = 20;  

  via_points_msg = Path() 
  via_points_msg.header.stamp = rospy.Time.now()
  via_points_msg.header.frame_id = "odom" # CHANGE HERE: odom/map

  for r in rad :
    for i in range(360) :
      theta = float(i) / 360.0 * 2.0 * math.pi;
      circle_x = r * math.cos(theta) - r;
      circle_y = r * math.sin(theta) - 0;

      point_tmp = PoseStamped()
      point_tmp.pose.position.x = circle_x + offset;
      point_tmp.pose.position.y = circle_y + offset;
      point_tmp.header.frame_id = "odom" # CHANGE HERE: odom/map

      # print("r: %f, theta: %f, point: [%f, %f]" %(r, theta, circle_x, circle_y) )

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

