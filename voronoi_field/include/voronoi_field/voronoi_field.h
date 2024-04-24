#ifndef _VORONOI_FIELD_H_
#define _VORONOI_FIELD_H_

#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <voronoi_field/VoronoiFieldPluginConfig.h>
#include <dynamic_reconfigure/server.h>
#include <boost/thread.hpp>

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


 private:

}; // class VoronoiFieldLayer

} // namespace costmap_2d

#endif // _VORONOI_FIELD_H_
