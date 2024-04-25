/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */
#include <voronoi_field/nanoflann_port.h>
#include <ros/ros.h>

namespace nanoflann_port_ns
{

NanoflannPort::NanoflannPort(const std::vector<geometry_msgs::Point>& nl)
{
  Init(nl);
}

NanoflannPort::~NanoflannPort()
{
  Reset();
}

void NanoflannPort::Init(const std::vector<geometry_msgs::Point>& nl)
{
  if (nl.empty()) return;

  Reset();

  pc_ = std::make_shared<PointCloud<double>>();
  pc_->pts.resize(nl.size());
  for (size_t i = 0; i < nl.size(); ++i) {
    pc_->pts.at(i).x = nl.at(i).x;
    pc_->pts.at(i).y = nl.at(i).y;
    pc_->pts.at(i).z = 0;
  }

  kdtree_ = std::make_shared<kd_tree_t>(3, *pc_, 10);
}

void NanoflannPort::Reset()
{
  pc_.reset();
  kdtree_.reset();
}

KDTIndex NanoflannPort::FindClosestPoint(const geometry_msgs::Point p) const
{
  double query_pnt[3] = {p.x, p.y, p.z};
  size_t num_results = 1;
  std::vector<unsigned int> ret_index(num_results);
  std::vector<double> out_dist_sqr(num_results);

  num_results = kdtree_->knnSearch(
      &query_pnt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  ret_index.resize(num_results);
  out_dist_sqr.resize(num_results);

  KDTIndex kdti;
  kdti.idx = ret_index[0];
  kdti.dist = out_dist_sqr[0];

  // std::cout << "knnSearch(): idx=" << ret_index[0] << " dist=" << out_dist_sqr[0] << std::endl;
  return kdti;
}

size_t NanoflannPort::FindClosestPoint(const geometry_msgs::Point p,
  const size_t num, std::vector<KDTIndex>& idx_list) const
{
  if (pc_ == nullptr) return 0;
  double query_pnt[3] = {p.x, p.y, p.z};
  size_t num_results = num;
  std::vector<unsigned int> ret_index(num_results);
  std::vector<double> out_dist_sqr(num_results);

  num_results = kdtree_->knnSearch(
      &query_pnt[0], num_results, &ret_index[0], &out_dist_sqr[0]);

  ret_index.resize(num_results);
  out_dist_sqr.resize(num_results);

  KDTIndex kdti;
  for (size_t i = 0; i < num_results; ++i) {
    kdti.idx = ret_index[0];
    kdti.dist = out_dist_sqr[0];
    idx_list.push_back(kdti);
  }

  return idx_list.size();
}

size_t NanoflannPort::FindPointsInRadius(const geometry_msgs::Point p,
  const double r, std::vector<KDTIndex>& idx_list) const
{
  if (pc_ == nullptr) return 0;
  double query_pnt[3] = {p.x, p.y, p.z};
  std::vector<std::pair<uint32_t, double>> ret_matches;
  size_t num = kdtree_->radiusSearch(&query_pnt[0], r, ret_matches, nanoflann::SearchParams());

  KDTIndex kdti;
  for (size_t i = 0; i < num; ++i) {
    kdti.idx = ret_matches.at(i).first;
    kdti.dist = ret_matches.at(i).second;
    idx_list.push_back(kdti);
  }

  return idx_list.size();
}



} // namespace nanoflann_port_ns
