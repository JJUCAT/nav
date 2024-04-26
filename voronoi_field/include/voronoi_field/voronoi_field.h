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
#include <voronoi_field/dynamic_voronoi_port.h>
#include <voronoi_field/nanoflann_port.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>


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

 protected:

  boost::recursive_mutex* voronoi_access_;

  /**
   * @brief  计算更新边界，边界需要 >= 地图实际边界
   * @param  master_grid  master 地图
   * @param  min_range_i  地图边界 x 最小值
   * @param  min_range_j  地图边界 y 最小值
   * @param  max_range_i  地图边界 x 最大值
   * @param  max_range_j  地图边界 y 最大值
   */
  virtual void GetRange(const costmap_2d::Costmap2D& master_grid,
    int& min_range_i, int& min_range_j, int& max_range_i, int& max_range_j);

  /**
   * @brief  获取地图中的障碍物，基于地图坐标
   * @param  obstacles  障碍物
   * @param  master_grid  master 地图
   * @param  obs_cost  障碍物代价
   * @param  min_range_i  地图边界 x 最小值
   * @param  min_range_j  地图边界 y 最小值
   * @param  max_range_i  地图边界 x 最大值
   * @param  max_range_j  地图边界 y 最大值
   * @return size_t  障碍物数量
   */
  virtual size_t GetObstacles(std::vector<costmap_2d::MapLocation>& obstacles,
    const costmap_2d::Costmap2D& master_grid, const unsigned char obs_cost,
    const int min_range_i, const int min_range_j, const int max_range_i, const int max_range_j);

 private:

  dynamic_reconfigure::Server<costmap_2d::VoronoiFieldPluginConfig> *dsrv_;
  void reconfigureCB(costmap_2d::VoronoiFieldPluginConfig &config, uint32_t level);


  double GetFarestObstacleDistance(const nanoflann_port_ns::NanoflannPort& nanoflann_obs,
    const std::vector<geometry_msgs::Point>& world);

  /**
   * @brief  计算势场值
   * @param  x  坐标 x
   * @param  y  坐标 y
   * @param  dist_obs   [x,y] 最近障碍物距离
   * @param  dist_vdi   [x,y] 最近 voronoi 图距离
   * @param  dist_obs_max   地图中最远障碍物距离
   * @return double 
   */
  double ProjectFieldValue(const double x, const double y,
    const double dist_obs, const double dist_vdi, const double dist_obs_max);


  /**
   * @brief  vector 地图坐标转 vector 世界坐标
   * @param  obs_map  地图坐标
   * @param  obs_world   世界坐标
   */
  void VectorMap2World(const std::vector<costmap_2d::MapLocation>& map,
    std::vector<geometry_msgs::Point>& world);


  /**
   * @brief  发布 voronoi 图
   * @param  width  地图宽
   * @param  height  地图高
   * @param  vdiagram  voronoi 数据
   */
  void PubVoronoiDiagram(const int width, const int height, const std::vector<geometry_msgs::Point>& vdiagram);

  double alpha_{10.0};
  double dist2O_{3.0};
  double cell_dist2O_;
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  bool need_recompute_;

  ros::Publisher voronoi_diagram_pub_;

}; // class VoronoiFieldLayer

} // namespace costmap_2d

#endif // _VORONOI_FIELD__VORONOI_FIELD_H_
