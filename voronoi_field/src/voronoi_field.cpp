#include <voronoi_field/voronoi_field.h>
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(costmap_2d::VoronoiFieldLayer, costmap_2d::Layer)


namespace costmap_2d {

VoronoiFieldLayer::VoronoiFieldLayer()
{

}

VoronoiFieldLayer::~VoronoiFieldLayer()
{

}

void VoronoiFieldLayer::onInitialize()
{
  {
    boost::unique_lock < boost::recursive_mutex > lock(*voronoi_access_);

    ros::NodeHandle nh("~/" + name_);
    need_recompute_ = false;
    dynamic_reconfigure::Server<costmap_2d::VoronoiFieldPluginConfig>::CallbackType cb =
      [this](auto& config, auto level){ reconfigureCB(config, level); };

    if (dsrv_ != nullptr) {
      dsrv_->clearCallback();
      dsrv_->setCallback(cb);
    } else {
      dsrv_ = new dynamic_reconfigure::Server<costmap_2d::VoronoiFieldPluginConfig>(
        ros::NodeHandle("~/" + name_));
      dsrv_->setCallback(cb);
    }
  }
}

void VoronoiFieldLayer::reconfigureCB(costmap_2d::VoronoiFieldPluginConfig &config, uint32_t level)
{
  if (enabled_ != config.enabled || alpha_ != config.alpha || dist2O_ != config.dist2O) {
    boost::unique_lock < boost::recursive_mutex > lock(*voronoi_access_);

    enabled_ = config.enabled;
    alpha_ = config.alpha;
    dist2O_ = config.dist2O;
    cell_dist2O_ = layered_costmap_->getCostmap()->cellDistance(dist2O_);
    need_recompute_ = true;
  }
}

void VoronoiFieldLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  if (need_recompute_) {
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;
    *min_x = -std::numeric_limits<float>::max();
    *min_y = -std::numeric_limits<float>::max();
    *max_x = std::numeric_limits<float>::max();
    *max_y = std::numeric_limits<float>::max();
    need_recompute_ = false;
  } else {
    double tmp_min_x = last_min_x_;
    double tmp_min_y = last_min_y_;
    double tmp_max_x = last_max_x_;
    double tmp_max_y = last_max_y_;
    last_min_x_ = *min_x;
    last_min_y_ = *min_y;
    last_max_x_ = *max_x;
    last_max_y_ = *max_y;

    *min_x = std::min(tmp_min_x, *min_x) - dist2O_;
    *min_y = std::min(tmp_min_y, *min_y) - dist2O_;
    *max_x = std::max(tmp_max_x, *max_x) + dist2O_;
    *max_y = std::max(tmp_max_y, *max_y) + dist2O_;
  }
}


void VoronoiFieldLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  int min_range_i=min_i, min_range_j=min_j, max_range_i=max_i, max_range_j=max_j;
  GetRange(master_grid, min_range_i, min_range_j, max_range_i, max_range_j);

  std::vector<costmap_2d::MapLocation> obstacles;
  size_t obs_size = GetObstacles(obstacles, master_grid, 254u, min_range_i, min_range_j, max_range_i, max_range_j);

  if (obs_size > 0) {
    
  }
}


// -------------------- protected --------------------


void VoronoiFieldLayer::GetRange(const costmap_2d::Costmap2D& master_grid,
  int& min_range_i, int& min_range_j, int& max_range_i, int& max_range_j)
{
  unsigned int size_x = master_grid.getSizeInCellsX();
  unsigned int size_y = master_grid.getSizeInCellsY();

  min_range_i -= cell_dist2O_;
  min_range_j -= cell_dist2O_;
  max_range_i += cell_dist2O_;
  max_range_j += cell_dist2O_;

  min_range_i = std::max(0, min_range_i);
  min_range_j = std::max(0, min_range_j);
  max_range_i = std::min(int(size_x), max_range_i);
  max_range_j = std::min(int(size_y), max_range_j);
}


size_t VoronoiFieldLayer::GetObstacles(std::vector<costmap_2d::MapLocation> obstacles,
  const costmap_2d::Costmap2D& master_grid, const unsigned char obs_cost,
  const int min_range_i, const int min_range_j, const int max_range_i, const int max_range_j)
{
  for (int j = min_range_j; j < max_range_j; j++) {
    for (int i = min_range_i; i < max_range_i; i++) {
      unsigned char cost = master_grid.getCost(i, j);
      if (cost >= obs_cost) {
        costmap_2d::MapLocation ml;
        ml.x = i; ml.y = j;
        obstacles.push_back(ml);
      }
    }
  }
  return obstacles.size();
}





void VoronoiFieldLayer::matchSize()
{

}

void VoronoiFieldLayer::reset()
{

}


} // namespace costmap_2d
