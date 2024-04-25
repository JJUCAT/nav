/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */

#include <voronoi_field/dynamic_voronoi_port.h>


namespace dynamic_voronoi_port_ns
{

DynamicVoronoiPort::DynamicVoronoiPort(const int sizeX, const int sizeY,
  const std::vector<costmap_2d::MapLocation>& obstacles)
{
  if (map_ != nullptr) {
    for (int x = 0; x < sizeX; x ++) {
      delete map_[x];
    }
    delete map_;
  }

  sizeX_ = sizeX, sizeY_ = sizeY;
  map_ = new bool* [sizeX_];
  for (int x = 0; x < sizeX; x ++) {
    map_[x] = new bool[sizeY_];
    memset(map_[x], false, sizeY_);
  }

  for (auto obs : obstacles)
    map_[obs.x][obs.y] = true;

  dv_ = std::make_shared<DynamicVoronoi>();
  dv_->initializeMap(sizeX, sizeY, map_);
  dv_->update(); 
}

DynamicVoronoiPort::~DynamicVoronoiPort()
{
  if (map_ != nullptr) {
    for (int x=0; x<sizeX_; x++) {
      delete map_[x];
    }
    delete map_;
  }
}

void DynamicVoronoiPort::GetVoronoiDiagram(std::vector<costmap_2d::MapLocation>& voronoi_diagram)
{
  for(int y = 0; y < sizeY_; y++) {      
    for(int x = 0; x < sizeX_; x++) {
      if (dv_->isVoronoi(x, y)) {
        costmap_2d::MapLocation ml;
        voronoi_diagram.push_back(ml);
      }
    }
  }
}

float DynamicVoronoiPort::GetDistance2Obstacle(const int x, const int y)
{
  return dv_->getDistance(x, y);
}


} // namespace dynamic_voronoi_port_ns
