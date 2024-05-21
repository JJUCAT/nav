#ifndef RECOVERY_EXCAPE__ENVIRONMENT_H_
#define RECOVERY_EXCAPE__ENVIRONMENT_H_

#include <recovery_excape/individuals.h>
#include <navit_costmap/costmap_2d_ros.h>

namespace recovery_excape {

/**
 * @brief  环境评估个体的适应度，适应度高表示个体更优
 */
class Environment
{
 public:

  Environment();

  Environment(const std::shared_ptr<navit_costmap::Costmap2DROS>& map_ros);
  
  virtual ~Environment();

  double Evaluate(const std::vector<geometry_msgs::Pose> poses, const geometry_msgs::Pose robot_pose);

 protected:

  double EvaluateCostmap(const std::vector<geometry_msgs::Pose> poses, const double scale);

  double EvaluateStraight(const std::vector<geometry_msgs::Pose> poses, const double scale);

  std::shared_ptr<navit_costmap::Costmap2DROS> map_ros_;

}; // Environment

} // namespace recovery_excape

#endif // RECOVERY_EXCAPE__ENVIRONMENT_H_
