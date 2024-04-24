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

}

void VoronoiFieldLayer::updateBounds(double robot_x, double robot_y, double robot_yaw,
  double* min_x, double* min_y, double* max_x, double* max_y)
{
  
}

void VoronoiFieldLayer::updateCosts(costmap_2d::Costmap2D& master_grid,
  int min_i, int min_j, int max_i, int max_j)
{

}

void VoronoiFieldLayer::matchSize()
{

}

void VoronoiFieldLayer::reset()
{

}


} // namespace costmap_2d
