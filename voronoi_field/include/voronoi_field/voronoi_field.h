/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */
#ifndef _VORONOI_FIELD__VORONOI_FIELD_H_
#define _VORONOI_FIELD__VORONOI_FIELD_H_

#include "costmap_2d/costmap_2d.h"
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <voronoi_field/VoronoiFieldPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>
#include <voronoi_field/nanoflann_port.h>
#include <voronoi_field/dynamic_voronoi_port.h>
#include <nav_msgs/GridCells.h>

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

  virtual void GetRange(const costmap_2d::Costmap2D& master_grid,
    int& min_range_i, int& min_range_j, int& max_range_i, int& max_range_j);

  virtual size_t GetObstacles(std::vector<costmap_2d::MapLocation>& obstacles,
    const costmap_2d::Costmap2D& master_grid, const unsigned char obs_cost,
    const int min_range_i, const int min_range_j, const int max_range_i, const int max_range_j);

  

 private:

  dynamic_reconfigure::Server<costmap_2d::VoronoiFieldPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::VoronoiFieldPluginConfig &config, uint32_t level);

  /**
   * @brief  发布 voronoi 图
   * @param  width  地图宽
   * @param  height  地图高
   * @param  vdiagram  voronoi 数据
   */
  void PubVoronoiDiagram(const int width, const int height, const std::vector<costmap_2d::MapLocation>& vdiagram);

  double alpha_;
  double dist2O_;
  double cell_dist2O_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recompute_;

  ros::Publisher voronoi_diagram_pub_;

}; // class VoronoiFieldLayer

} // namespace costmap_2d

#endif // _VORONOI_FIELD__VORONOI_FIELD_H_
