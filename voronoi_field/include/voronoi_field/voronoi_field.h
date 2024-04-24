/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */
#ifndef _VORONOI_FIELD__VORONOI_FIELD_H_
#define _VORONOI_FIELD__VORONOI_FIELD_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <voronoi_field/VoronoiFieldPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <voronoi_field/nanoflann_port.h>

namespace costmap_2d
{

class VoronoiFieldLayer : public Layer
{
 public:

  VoronoiFieldLayer();

  virtual ~VoronoiFieldLayer();

  virtual void onInitialize();

  virtual void updateBounds(double robot_x, double robot_y, double robot_yaw,
    double* min_x, double* min_y, double* max_x, double* max_y);

  virtual void updateCosts(costmap_2d::Costmap2D& master_grid,
    int min_i, int min_j, int max_i, int max_j);

  virtual void matchSize();

  virtual void reset();

 protected:

  boost::recursive_mutex* voronoi_access_;

 private:

  dynamic_reconfigure::Server<costmap_2d::VoronoiFieldPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::VoronoiFieldPluginConfig &config, uint32_t level);

  double alpha_;
  double dist2O_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recompute_;

}; // class VoronoiFieldLayer

} // namespace costmap_2d

#endif // _VORONOI_FIELD__VORONOI_FIELD_H_
