#ifndef RECOVERY_EXCAPE__TYPEDEF_HPP_
#define RECOVERY_EXCAPE__TYPEDEF_HPP_

namespace recovery_excape {

/**
 * @brief  优化参数配置
 */
struct ExcapeParams {
  double smooth_weight{100.0};
  double costmap_weight{1.0};
  double distance_weight{10.0};
  double curvature_weight{10.0};
  double max_curvature{1.0};
  double costmap_factor{10.0};
  double max_time;
}; // struct ExcapeParams



const double kFree = 0.0;






} // namespace recovery_excape

#endif // RECOVERY_EXCAPE_H
