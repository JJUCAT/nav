#include "costmap_2d/cost_values.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/GridCells.h"
#include <voronoi_field/voronoi_field.h>
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(costmap_2d::VoronoiFieldLayer, costmap_2d::Layer)


namespace costmap_2d {

VoronoiFieldLayer::VoronoiFieldLayer()
{
  voronoi_access_ = new boost::recursive_mutex();
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
    voronoi_diagram_pub_ = nh.advertise<nav_msgs::GridCells>("voronoi_diagram", 1);
    pruned_voronoi_diagram_pub_ = nh.advertise<nav_msgs::GridCells>("pruned_voronoi_diagram", 1);
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
    ROS_INFO("[VFL] reconfigureCB, enable %d, alpha %f, dist2O %f", enabled_, alpha_, dist2O_);
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
  int range_i = max_range_i - min_range_i, range_j = max_range_j - min_range_j;

  std::vector<costmap_2d::MapLocation> obstacles;
  size_t obs_size = GetObstacles(obstacles, master_grid, costmap_2d::LETHAL_OBSTACLE,
    min_range_i, min_range_j, max_range_i, max_range_j);

  if (obs_size > 0) {
    std::vector<geometry_msgs::Point> obs_world;
    VectorMap2World(obstacles, obs_world);
    auto nanoflann_obs = new nanoflann_port_ns::NanoflannPort(obs_world);

    auto voronoi_port = new dynamic_voronoi_port_ns::DynamicVoronoiPort(range_i, range_j, obstacles, true);
    std::string savepath = std::getenv("HOME") + std::string("/voronoifield.pgm");
    voronoi_port->Save(savepath.c_str());
    std::vector<costmap_2d::MapLocation> vdiagram;
    voronoi_port->GetVoronoiDiagram(vdiagram);
    std::vector<geometry_msgs::Point> vdi_world;
    VectorMap2World(vdiagram, vdi_world);
    PubVoronoiDiagram(voronoi_diagram_pub_, range_i, range_j, vdi_world);
    // std::vector<costmap_2d::MapLocation> pruned_vdiagram = vdiagram;
    // voronoi_port->GetPruneVoronoiDiagram(pruned_vdiagram);
    // std::vector<geometry_msgs::Point> pvdi_world;
    // PubVoronoiDiagram(pruned_voronoi_diagram_pub_, range_i, range_j, pvdi_world);

    auto nanoflann_vdi = new nanoflann_port_ns::NanoflannPort(vdi_world);

    unsigned int mx = master_grid.getSizeInCellsX(), my = master_grid.getSizeInCellsY();
    double farest = GetFarestObstacleDistance(*nanoflann_obs, vdi_world);
    ROS_INFO("[VFL] farest distance %f", farest);
    for (unsigned int y = 0; y < my; y ++) {
      for (unsigned int x = 0; x < mx; x ++) {
        auto cost = master_grid.getCost(x, y);
        if (cost < costmap_2d::LETHAL_OBSTACLE) {
          geometry_msgs::Point wp;
          master_grid.mapToWorld(x, y, wp.x, wp.y);
          double dist_obs = nanoflann_obs->FindClosestPoint(wp).dist;          
          double dist_vdi = nanoflann_vdi->FindClosestPoint(wp).dist;
          double dist_obs_max = farest;
          dist_obs = std::min(dist_obs, dist_obs_max);
          double field_value = ProjectFieldValue(x, y, dist_obs, dist_vdi, dist_obs_max);
          cost = static_cast<unsigned char>(field_value * costmap_2d::LETHAL_OBSTACLE);
          // ROS_INFO("[VFL] map[%u, %u] world[%f,%f], field value %f, cost %u",
          //   x, y, wp.x, wp.y, field_value, cost);
          master_grid.setCost(x, y, cost);
        }
      }
    }
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


size_t VoronoiFieldLayer::GetObstacles(std::vector<costmap_2d::MapLocation>& obstacles,
  const costmap_2d::Costmap2D& master_grid, const unsigned char obs_cost,
  const int min_range_i, const int min_range_j, const int max_range_i, const int max_range_j)
{
  int min_j = min_range_j, max_j = max_range_j, min_i = min_range_i, max_i = max_range_i;
  for (int j = min_j; j < max_j; j++) {
    for (int i = min_i; i < max_i; i++) {
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


// -------------------- private --------------------


double VoronoiFieldLayer::GetFarestObstacleDistance(const nanoflann_port_ns::NanoflannPort& nanoflann_obs,
  const std::vector<geometry_msgs::Point>& world)
{
  double farest = std::numeric_limits<double>::min();
  for (auto p : world) {
    auto idx = nanoflann_obs.FindClosestPoint(p);
    if (idx.dist > farest)
      farest = idx.dist;
  }
  return farest;
}


double VoronoiFieldLayer::ProjectFieldValue(const double x, const double y,
  const double dist_obs, const double dist_vdi, const double dist_obs_max)
{
  double prefix = alpha_ / ( alpha_ + dist_obs);
  double middle = dist_vdi / (dist_obs + dist_vdi);
  double suffix = std::pow(dist2O_ - dist_obs_max, 2)/std::pow(dist_obs_max, 2);
  double field_value = prefix * middle * suffix;
  return field_value;
}


void VoronoiFieldLayer::VectorMap2World(const std::vector<costmap_2d::MapLocation>& obs_map,
  std::vector<geometry_msgs::Point>& obs_world)
{
  auto map = layered_costmap_->getCostmap();

  for (auto p : obs_map) {
    geometry_msgs::Point cell;
    map->mapToWorld(p.x, p.y, cell.x, cell.y);
    obs_world.push_back(cell);
  }
}

void VoronoiFieldLayer::PubVoronoiDiagram(const ros::Publisher& pub,
  const int width, const int height,
  const std::vector<geometry_msgs::Point>& vdiagram)
{
  auto map = layered_costmap_->getCostmap();

  nav_msgs::GridCells grid;
  grid.header.seq = 0;
  grid.header.stamp = ros::Time::now();
  grid.header.frame_id = layered_costmap_->getGlobalFrameID();
  grid.cell_width = map->getResolution();
  grid.cell_height = map->getResolution();
  grid.cells = vdiagram;
  pub.publish(grid);
}


} // namespace costmap_2d
