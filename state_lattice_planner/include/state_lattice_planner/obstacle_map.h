#ifndef __OBSTACLE_MAP_H
#define __OBSTACLE_MAP_H

#include <iostream>
#include <vector>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace state_lattice_planner
{

 //
 //            x
 //            ^
 //            | height
 //            |
 //    y <- - -
 //       width
 //

template<typename ELEMENT_TYPE>
class ObstacleMap
{
public:
  ObstacleMap(void);
  unsigned int get_index_from_xy(double, double) const;
  void set_shape(unsigned int, unsigned int, double);
  void set_frame_id(const std::string);
  double get_resolution(void) const;
  std::string get_map_frame(void) const;
  bool in_map(
    const double base_link_x, const double base_link_y,
    double& map_x, double& map_y) const;
  void set_yaw_diff(const double yaw_diff);

  std::vector<ELEMENT_TYPE> data;
protected:
  std::string frame_id;
  unsigned int width;// cells
  unsigned int height;// cells
  double resolution;// m/cell
  double origin_x;// m
  double origin_y;// m
  double yaw_diff;// 当前机器航向角与地图坐标系 x 轴方向（地图的朝向）的角度差
};

template<typename ELEMENT_TYPE>
ObstacleMap<ELEMENT_TYPE>::ObstacleMap(void)
{
  width = 0;
  height = 0;
  resolution = 0.0;
  origin_x = 0.0;
  origin_y = 0.0;
  data.clear();
}

template<typename ELEMENT_TYPE>
unsigned int ObstacleMap<ELEMENT_TYPE>::get_index_from_xy(double x, double y) const
{
  unsigned int index = 0;
  index = std::round((x - origin_x) / resolution) + std::round((y - origin_y) / resolution) * height;
  return index;
}

template<typename ELEMENT_TYPE>
void ObstacleMap<ELEMENT_TYPE>::set_shape(unsigned int width_, unsigned int height_, double resolution_)
{
  if(width_ > 0 && height_ > 0){
    width = width_;
    height = height_;
  }else{
    std::cout << "\033[31mvalue error: width and height must be > 0\033[0m" << std::endl;
    return;
  }
  if(resolution_ > 0.0){
    resolution = resolution_;
  }else{
    std::cout << "\033[31mvalue error: resolution must be > 0.0 m\033[0m" << std::endl;
    return;
  }
  // origin is in the lower right-hand corner of the map.
  origin_x = -static_cast<double>(height) * 0.5 * resolution;
  origin_y = -static_cast<double>(width) * 0.5 * resolution;
}

template<typename ELEMENT_TYPE>
double ObstacleMap<ELEMENT_TYPE>::get_resolution(void) const
{
  return resolution;
}

template<typename ELEMENT_TYPE>
void ObstacleMap<ELEMENT_TYPE>::set_frame_id(const std::string frame_id)
{
  this->frame_id = frame_id;
}

template<typename ELEMENT_TYPE>
std::string ObstacleMap<ELEMENT_TYPE>::get_map_frame(void) const
{
  return frame_id;
}

template<typename ELEMENT_TYPE>
void ObstacleMap<ELEMENT_TYPE>::set_yaw_diff(const double yaw_diff)
{
  this->yaw_diff = yaw_diff;
}

// 传入 base_link 下的坐标，输出 map 下的坐标
template<typename ELEMENT_TYPE>
bool ObstacleMap<ELEMENT_TYPE>::in_map(
  const double base_link_x, const double base_link_y,
  double& map_x, double& map_y) const
{
  map_x = cos(yaw_diff) * base_link_x - sin(yaw_diff) * base_link_y;
  map_y = sin(yaw_diff) * base_link_x + cos(yaw_diff) * base_link_y;
  if (map_x > fabs(origin_x)) return false;
  if (map_y > fabs(origin_y)) return false;
  return true;
}

}

#endif// __OBSTACLE_MAP_H
