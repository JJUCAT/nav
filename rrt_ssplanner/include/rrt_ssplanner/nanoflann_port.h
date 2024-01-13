#ifndef NANOFLANN_PORT_H
#define NANOFLANN_PORT_H

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


class NanoflannPort
{
  using kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, PointCloud<double>>,
    PointCloud<double>, 3 /* dim */>;

 public:
  NanoflannPort() {}
  NanoflannPort(const std::vector<geometry_msgs::Point>& pl);
  ~NanoflannPort();

  void Init(const std::vector<geometry_msgs::Point>& pl);

  void Reset();

  KDTIndex FindClosestPoint(const geometry_msgs::Point p) const;

  size_t FindClosestPoint(const geometry_msgs::Point p,
    const size_t num, std::vector<KDTIndex>& idx_list) const;

  size_t FindPointsInRadius(const geometry_msgs::Point p,
    const double r, std::vector<KDTIndex>& idx_list) const;

 private:
  std::shared_ptr<PointCloud<double>> pc_;
  std::shared_ptr<kd_tree_t> kdtree_;
};


} // namespace nanoflann_port_ns


#endif // NANOFLANN_PORT_H
