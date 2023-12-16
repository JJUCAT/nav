#include "ros/ros.h"
#include "tf/tf.h"
#include "angles/angles.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseArray.h"
#include <limits>

geometry_msgs::Twist twist;
void callback(const geometry_msgs::TwistConstPtr& msg) {
  twist = *msg;
  ROS_INFO("twist [%f, %f]", twist.linear.x, twist.angular.z);
}

/**
 * @brief 差速轮和阿克曼运动模型预测位姿
 * @param  pose             当前位姿
 * @param  v                当前线速度
 * @param  ws               当前角速度或阿克曼的转向轮角度
 * @param  dt               运动所长时间
 * @param  wheel_base       阿克曼的轴距
 * @param  use_steering     是否将 ws 参数作为阿克曼转向轮角度
 * @return geometry_msgs::PoseStamped 返回运动 dt 时间后的位姿
 */
geometry_msgs::PoseStamped predict(const geometry_msgs::PoseStamped pose, const double v,
  const double ws, const double dt, const double wheel_base, const bool use_steering = false) {
  double dx = 0, dy = 0, alpha = 0;
  double epsilon = 0.001;
  if (fabs(ws) < epsilon) { // 认为是直线
    dx = v * dt;
  } else {
    double r, w = ws;
    if (use_steering) {
      r = fabs(wheel_base / tan(ws));
      r *= (ws > 0) ^ (v >= 0) ? -1 : 1;
      w = v / r;
    }
    double angle = fabs(w * dt);
    while (angle > 2*M_PI) angle -= 2*M_PI;
    int dx_sign = (angle > M_PI ? -1 : 1) * (v > 0 ? 1 : -1);
    alpha = angles::normalize_angle(w * dt);
    dy = v/w*(1-cos(w * dt));
    dx = std::sqrt(std::pow(v/w, 2) - std::pow(v/w-dy, 2)) * dx_sign;
  }
  ROS_INFO("alpha %f, dy %f, dx %f", alpha, dy, dx);
  geometry_msgs::PoseStamped next_pose = pose;
  double yaw = tf::getYaw(next_pose.pose.orientation);
  next_pose.pose.position.x += cos(yaw)*dx - sin(yaw)*dy;
  next_pose.pose.position.y += sin(yaw)*dx + cos(yaw)*dy;
  double next_yaw = angles::normalize_angle(yaw + alpha);
  next_pose.pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);
  return next_pose;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vw_node");
  ros::NodeHandle n("~");
  ros::Subscriber sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, callback);
  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose", 10);
  ros::Publisher poses_pub = n.advertise<geometry_msgs::PoseArray>("poses", 10);
  ros::Publisher aposes_pub = n.advertise<geometry_msgs::PoseArray>("aposes", 10);
  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.position.x = 2.45;
  pose.pose.position.y = -5.2;
  pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.83);
  
  double wheel_base = 1.0;
  double hz = 10.f;
  ros::Rate r(hz);
  geometry_msgs::PoseStamped next = pose;
  while(ros::ok()) {
    ROS_INFO("work");
    pose_pub.publish(pose);
    geometry_msgs::Twist tmp_twist = twist;
    next = predict(pose, tmp_twist.linear.x, tmp_twist.angular.z, 1/hz, wheel_base);
    geometry_msgs::PoseArray pa;
    geometry_msgs::PoseArray apa;
    pa.header.frame_id = pose.header.frame_id;
    pa.header.stamp = ros::Time::now();
    apa.header = pa.header;
    for (double t = 0; t < 5.0f; t += 0.5) {
      auto p = predict(pose, tmp_twist.linear.x, tmp_twist.angular.z, t, wheel_base, false).pose;
      auto ap = predict(pose, tmp_twist.linear.x, tmp_twist.angular.z, t, wheel_base, true).pose;
      pa.poses.push_back(p);
      apa.poses.push_back(ap);
    }
    poses_pub.publish(pa);
    aposes_pub.publish(apa);
    pose = next;
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}
