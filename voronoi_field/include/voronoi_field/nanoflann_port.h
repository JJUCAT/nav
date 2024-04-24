/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */
#ifndef _VORONOI_FIELD__NANOFLANN_PORT_H_
#define _VORONOI_FIELD__NANOFLANN_PORT_H_

#include <costmap_2d/costmap_2d.h>
#include <nanoflann/nanoflann.hpp>
#include <nanoflann/utils.h>
#include <geometry_msgs/Point32.h>
#include <nav_msgs/Path.h>


namespace nanoflann_port_ns {

typedef struct
{
  size_t idx;
  double dist;
} KDTIndex;

/**
 * @brief port of 3rd party library 'nanoflann'
 */
class NanoflannPort
{
  using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
    PointCloud<double>, 3 /* dim */>;

 public:
  NanoflannPort() {}
  NanoflannPort(const std::vector<costmap_2d::MapLocation>& nl);
  ~NanoflannPort();

  void Init(const std::vector<costmap_2d::MapLocation>& nl);
  void Reset();

  KDTIndex FindClosestPoint(const costmap_2d::MapLocation p) const;

  size_t FindClosestPoint(const costmap_2d::MapLocation p,
    const size_t num, std::vector<KDTIndex>& idx_list) const;

  size_t FindPointsInRadius(const costmap_2d::MapLocation p,
    const double r, std::vector<KDTIndex>& idx_list) const;

 private:
  std::shared_ptr<PointCloud<double>> pc_;
  std::shared_ptr<kd_tree_t> kdtree_;
};


} // namespace nanoflann_port_ns


#endif // _VORONOI_FIELD__NANOFLANN_PORT_H_
