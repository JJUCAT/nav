#ifndef RECOVERY_EXCAPE__INDIVIDUALS_H_
#define RECOVERY_EXCAPE__INDIVIDUALS_H_

#include <recovery_excape/genotype.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include "navit_costmap/costmap_2d.h"

namespace recovery_excape {

/**
 * @brief 个体由多个基因序列组成
 */
class Individuals
{
 public:

  Individuals();
  
  Individuals(const HereditaryInformation info);

  explicit Individuals(const Individuals& indiv);

  Individuals& operator=(const Individuals& indiv);

  Individuals(Individuals&& indiv) noexcept;

  Individuals& operator=(const Individuals&& indiv) noexcept;

  void Reproduce();

  void Mutation();

  HereditaryInformation GetHereditaryInformation() const;

  Individuals Cross(const Individuals& indiv);

  std::vector<geometry_msgs::Pose> GenCharacter(const geometry_msgs::PoseStamped pose,
    const navit_costmap::Costmap2D& map, const double v, const double wheel_base,
    const bool use_steering = false);

  void Survive();

  void GetBehavior(const Chromosome& chro, const geometry_msgs::Pose p,
    const navit_costmap::Costmap2D& map, const double wheel_base, double& ws, int& v_signed);

  int GetDir(const geometry_msgs::Pose p, const navit_costmap::Costmap2D& map, const double r);

 protected:

  size_t SelectRandomGeno();

  HereditaryInformation hereditary_info_;
  Chromosome ancestor_;

 private:



}; // Individuals

using Population = std::vector<Individuals>;

// 按 key 从大到小排序
using PopulationWithFitness = std::multimap<double, Individuals, std::greater<double>>;

} // namespace recovery_excape

#endif // RECOVERY_EXCAPE__INDIVIDUALS_H_
