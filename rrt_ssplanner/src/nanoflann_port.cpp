#include <rrt_ssplanner/nanoflann_port.h>
#include <ros/ros.h>

namespace nanoflann_port_ns
{
    
  NanoflannPort::NanoflannPort(const std::vector<geometry_msgs::Point>& pl)
  {
    Init(pl);
  }

  NanoflannPort::~NanoflannPort()
  {
    Reset();
  }

  void NanoflannPort::Init(const std::vector<geometry_msgs::Point>& pl)
  {
    if (pl.empty()) return;

    Reset();

    pc_ = std::make_shared<PointCloud<double>>();
    pc_->pts.resize(pl.size());
    for (int i = 0; i < pl.size(); ++i) {
      pc_->pts.at(i).x = pl.at(i).x;
      pc_->pts.at(i).y = pl.at(i).y;
      pc_->pts.at(i).z = pl.at(i).z;
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
      kdti.idx = ret_matches.at(i).second;
      kdti.dist = ret_matches.at(i).first;
      idx_list.push_back(kdti);
    }

    return idx_list.size();
  }



} // namespace nanoflann_port_ns
