/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */

#include <voronoi_field/dynamic_voronoi_port.h>
#include <ros/ros.h>

namespace dynamic_voronoi_port_ns
{

DynamicVoronoiPort::DynamicVoronoiPort(const int sizeX, const int sizeY,
  const std::vector<costmap_2d::MapLocation>& obstacles, const bool boundary)
{
  if (map_ != nullptr) {
    for (int x = 0; x < sizeX_; x ++) {
      delete map_[x];
    }
    delete map_;
  }

  boundary_ = boundary;
  sizeX_ = sizeX, sizeY_ = sizeY;
  if (boundary_) { sizeX_ += 2*boundary_size_; sizeY_ += 2*boundary_size_; }  
  map_ = new bool* [sizeX_];
  for (int x = 0; x < sizeX_; x ++) {
    map_[x] = new bool[sizeY_];
    memset(map_[x], false, sizeY_);
  }
  if (boundary_) {
    for (int y = 0; y < sizeY_; y ++) {
      for (int x = 0; x < sizeX_; x ++) {
        if ( y<boundary_size_ || y>=sizeY_-boundary_size_ ||
          x<boundary_size_ || x>=sizeX_-boundary_size_)
          map_[x][y] = true;
      }
    }
  }

  for (auto obs : obstacles) {
    if (boundary_) { obs.x+=boundary_size_; obs.y+=boundary_size_; }
    map_[obs.x][obs.y] = true;    
  }

  dv_ = std::make_shared<DynamicVoronoi>();
  dv_->initializeMap(sizeX_, sizeY_, map_);
  dv_->update(); 
  // dv_->prune();
  dv_->updateAlternativePrunedDiagram();
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
      // if (dv_->isVoronoi(x, y)) {
      if (dv_->isVoronoiAlternative(x, y)) {
        costmap_2d::MapLocation ml;
        ml.x = x; ml.y = y;
        if (boundary_) { ml.x-=boundary_size_; ml.y-=boundary_size_; }
        voronoi_diagram.push_back(ml);
      }
    }
  }
}

float DynamicVoronoiPort::GetDistance2Obstacle(const int x, const int y)
{
  return dv_->getDistance(x, y);
}


void DynamicVoronoiPort::Save(const char* filename)
{
  dv_->visualize(filename);
}

} // namespace dynamic_voronoi_port_ns
