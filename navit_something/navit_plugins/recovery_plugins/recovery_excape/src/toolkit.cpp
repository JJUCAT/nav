#include <recovery_excape/toolkit.h>


namespace recovery_excape {

Eigen::Vector3d Predict(const Eigen::Vector3d pose, const double v,
  const double ws, const double dt, const double wheel_base, const bool use_steering)
{
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
  Eigen::Vector3d next_pose = pose;
  double yaw = next_pose[2];
  next_pose[0] += cos(yaw)*dx - sin(yaw)*dy;
  next_pose[1] += sin(yaw)*dx + cos(yaw)*dy;
  next_pose[2] = yaw + alpha;
  return next_pose;
}

geometry_msgs::Pose Predict(const geometry_msgs::Pose pose, const double v,
  const double ws, const double dt, const double wheel_base, const bool use_steering) {
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
  geometry_msgs::Pose next_pose = pose;
  double yaw = tf::getYaw(next_pose.orientation);
  next_pose.position.x += cos(yaw)*dx - sin(yaw)*dy;
  next_pose.position.y += sin(yaw)*dx + cos(yaw)*dy;
  double next_yaw = angles::normalize_angle(yaw + alpha);
  next_pose.orientation = tf::createQuaternionMsgFromYaw(next_yaw);
  return next_pose;
}

size_t GenPath(const std::vector<Eigen::Vector2d>* u_sequence,
  const geometry_msgs::PoseStamped& robot_pose,
  const double time_step, std::vector<Eigen::Vector3d>& x_sequence)
{
  Eigen::Vector3d pose(robot_pose.pose.position.x, robot_pose.pose.position.y, 
    tf::getYaw(robot_pose.pose.orientation));
  x_sequence.push_back(pose);
  for (size_t i = 0; i < u_sequence->size(); i ++) {
    pose = Predict(pose, u_sequence->at(i)[0], u_sequence->at(i)[1],
      time_step, 0.0);
    x_sequence.push_back(pose);
  }
  return x_sequence.size();
}

double W2S(const double v, const double w, const double wheel_base, const double s_limit)
{
  double s = atan(w*wheel_base/v);
  s = std::max(-s_limit, std::min(s_limit, s));
  return s;
}

int GetDir(const geometry_msgs::Pose p, const navit_costmap::Costmap2D& map, const double r)
{
  double yaw = tf::getYaw(p.orientation);
  double front_cost = 0.0f, back_cost = 0.0;
  unsigned int mx, my;
  double r_step = r / 3;
  for (double loopr = r_step; loopr <= r; loopr += r_step) {
    for (double y = -1.5; y <= 1.5; y += 0.1) {
      double wx = p.position.x + cos(yaw) * r;
      double wy = p.position.y + sin(yaw) * r;
      double base_cost = 255.0;
      if (map.worldToMap(wx, wy, mx, my)) {
        base_cost = map.getCost(mx, my);
      }
      front_cost += base_cost;
    }

    for (double y = -1.5; y <= 1.5; y += 0.3) {
      double wx = p.position.x + cos(yaw) * -r;
      double wy = p.position.y + sin(yaw) * -r;
      double base_cost = 255.0;
      if (map.worldToMap(wx, wy, mx, my)) {
        base_cost = map.getCost(mx, my);
      }
      back_cost += base_cost;
    }    
  }

  int dir = front_cost > back_cost ? -1 : 1;
  return dir;
}

} // namespace recovery_excape
