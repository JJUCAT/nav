#include <recovery_excape/environment.h>
#include <navit_costmap/costmap_2d_ros.h>
#include "navit_costmap/costmap_2d.h"
#include "tf/transform_datatypes.h"

namespace recovery_excape {

Environment::Environment() {}

Environment::Environment(const std::shared_ptr<navit_costmap::Costmap2DROS>& map_ros)
  : map_ros_{map_ros} {}

Environment::~Environment() {}

double Environment::Evaluate(const std::vector<geometry_msgs::Pose> poses,
  const geometry_msgs::Pose robot_pose)
{
  double fitness = 0.0f, average;
  fitness += EvaluateCostmap(poses, 1.0);
  fitness += EvaluateStraight(poses, 200);
  average = fitness / poses.size();
  // ROS_INFO("[RCVR][EXCAPE] total fitness %.3f, average %.3f.", fitness, average);
  return average;
}



// -------------------- protected --------------------

double Environment::EvaluateCostmap(const std::vector<geometry_msgs::Pose> poses, const double scale)
{
  navit_costmap::Costmap2D map;
  {
    auto map_mutex = map_ros_->getCostmap()->getMutex();
    boost::recursive_mutex::scoped_lock lock(*map_mutex);
    map = *map_ros_->getCostmap();
  }

  double danger = 230, head_danger = 210;
  double wb = 1.7;
  double fitness = 0.0, score;  
  unsigned int mx, my;
  for (size_t i = 0; i < poses.size(); i ++) {
    if (map.worldToMap(poses.at(i).position.x, poses.at(i).position.y, mx, my)) {
      score = map.getCost(mx, my);
      // score = score > danger ? 0.0 : score;
      fitness += (danger - score);
    }

    geometry_msgs::Pose hp = poses.at(i);
    double yaw = tf::getYaw(hp.orientation);
    hp.position.x += cos(yaw) * wb;
    hp.position.y += sin(yaw) * wb;
    if (map.worldToMap(hp.position.x, hp.position.y, mx, my)) {
      score = map.getCost(mx, my);
      // score = score > head_danger ? 0.0 : score;
      fitness += (head_danger - score);
    }
  }

  // ROS_INFO("[RCVR][EXCAPE] costmap fitness %.3f", fitness);
  return fitness;
}

double Environment::EvaluateStraight(const std::vector<geometry_msgs::Pose> poses, const double scale)
{
  double fitness = 0.0, base = 255.0;
  for (size_t i = 0, j = 0; i < poses.size(); j = i, i ++) {
    double i_yaw = tf::getYaw(poses.at(i).orientation);
    double j_yaw = tf::getYaw(poses.at(j).orientation);
    fitness += base - fabs(j_yaw - i_yaw) * scale;
  }
  // ROS_INFO("[RCVR][EXCAPE] straight fitness %.3f", fitness);
  return fitness;
}

// -------------------- private --------------------


} // namespace recovery_excape
