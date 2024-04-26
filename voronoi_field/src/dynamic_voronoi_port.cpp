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
        if (IsBoundary(x, y)) {
          map_[x][y] = true;
        }
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
  dv_->prune();
  // dv_->updateAlternativePrunedDiagram();
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
      // if (dv_->isVoronoiAlternative(x, y)) {
        costmap_2d::MapLocation ml;
        ml.x = x; ml.y = y;
        if (boundary_) { ml.x-=boundary_size_; ml.y-=boundary_size_; }
        voronoi_diagram.push_back(ml);
      }
    }
  }
}

void DynamicVoronoiPort::GetPruneVoronoiDiagram(std::vector<costmap_2d::MapLocation>& voronoi_diagram)
{
  std::vector<size_t> line_junctions;
  std::vector<size_t> line_ends;
  FindJunctionsAndEnd(voronoi_diagram, line_junctions, line_ends);
  std::sort(line_ends.begin(), line_ends.end(), std::greater<int>()); // 从大到小排
  ROS_INFO("[VFL] first prune, ends:%lu, junctions:%lu", line_ends.size(), line_junctions.size());

  auto IsHit = [&](const size_t e, const std::vector<size_t>& line_junctions) {
    for (auto j : line_junctions) {
      if (e == j) return true;
    }
    return false;
  };

  size_t loop = 0;
  while (!line_ends.empty()) {
    loop ++;
    for (auto e : line_ends) {
      if (!IsHit(e, line_junctions)) voronoi_diagram.erase(voronoi_diagram.begin()+e);
    }
    line_junctions.clear();
    line_ends.clear();
    FindJunctionsAndEnd(voronoi_diagram, line_junctions, line_ends);
    std::sort(line_ends.begin(), line_ends.end(), std::greater<int>());
    ROS_INFO("[VFL] [%lu] prune, ends:%lu, junctions:%lu", loop, line_ends.size(), line_junctions.size());
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


// -------------------- protected --------------------

bool DynamicVoronoiPort::IsBoundary(const int x, const int y)
{
  int boundary_size = boundary_ ? boundary_size_ : 0;
  if ( y<boundary_size_ || y>=sizeY_-boundary_size_ ||
       x<boundary_size_ || x>=sizeX_-boundary_size_) {
    return true;
  }
  return false;
}

bool DynamicVoronoiPort::IsNearBoundary(const int x, const int y)
{
  int boundary_size = boundary_ ? boundary_size_+1 : 1;
  if ( y<boundary_size || y>=sizeY_-boundary_size ||
       x<boundary_size || x>=sizeX_-boundary_size) {
    return true;
  }
  return false;
}


bool DynamicVoronoiPort::IsLineEnd(const int x, const int y)
{
  if (IsNearBoundary(x, y)) return false;

  auto IsEnd0 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y)) &&
        (!dv_->isVoronoi(x-1, y+1) && !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x-1, y-1) &&
         !dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd1 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x, y-1)) &&
        (!dv_->isVoronoi(x-1, y+1) && !dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x+1, y+1) &&
         !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y))) {
      return true;
    }
    return false;
  };

  auto IsEnd2 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x-1, y)) &&
        (!dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x+1, y+1) && !dv_->isVoronoi(x+1, y) &&
         !dv_->isVoronoi(x+1, y-1) && !dv_->isVoronoi(x, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd3 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x, y+1)) &&
        (!dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y) && !dv_->isVoronoi(x+1, y-1) &&
         !dv_->isVoronoi(x, y-1) && !dv_->isVoronoi(x-1, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd4 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y-1)) &&
        (!dv_->isVoronoi(x-1, y+1) && !dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x+1, y+1) &&
         !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y) && 
         !dv_->isVoronoi(x-1, y-1) && !dv_->isVoronoi(x, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd5 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x-1, y-1)) &&
        (!dv_->isVoronoi(x-1, y+1) && !dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x+1, y+1) &&
         !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y) && 
         !dv_->isVoronoi(x, y-1) && !dv_->isVoronoi(x+1, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd6 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x-1, y+1)) &&
        (!dv_->isVoronoi(x, y+1) && !dv_->isVoronoi(x+1, y+1) &&
         !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y) && 
         !dv_->isVoronoi(x-1, y-1)&&!dv_->isVoronoi(x, y-1) && !dv_->isVoronoi(x+1, y-1))) {
      return true;
    }
    return false;
  };

  auto IsEnd7 = [&](const int x, const int y) {
    if ((dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y+1)) &&
        (!dv_->isVoronoi(x-1, y+1) && !dv_->isVoronoi(x, y+1) &&
         !dv_->isVoronoi(x-1, y) && !dv_->isVoronoi(x+1, y) && 
         !dv_->isVoronoi(x-1, y-1)&&!dv_->isVoronoi(x, y-1) && !dv_->isVoronoi(x+1, y-1))) {
      return true;
    }
    return false;
  };

  if (IsEnd0(x, y) || IsEnd1(x, y) || IsEnd2(x, y) || IsEnd3(x, y) || 
      IsEnd4(x, y) || IsEnd5(x, y) || IsEnd6(x, y) || IsEnd7(x, y)) {
    return true;
  }
  return false;
}

bool DynamicVoronoiPort::IsLineJunctions(const int x, const int y)
{
  if (IsNearBoundary(x, y)) return false;

  auto IsJunctions0 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x-1, y+1) && dv_->isVoronoi(x+1, y+1) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions1 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x+1, y) && dv_->isVoronoi(x-1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions2 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x+1, y+1) && dv_->isVoronoi(x-1, y) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions3 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x-1, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x+1, y) && dv_->isVoronoi(x, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions4 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x-1, y-1) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions5 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x+1, y+1) && dv_->isVoronoi(x-1, y) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions6 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x+1, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x+1, y) && dv_->isVoronoi(x-1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions7 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x, y+1) && dv_->isVoronoi(x-1, y) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions8 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x-1, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x-1, y-1) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions9 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x-1, y+1) && dv_->isVoronoi(x+1, y+1) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x-1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions10 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x-1, y+1) && dv_->isVoronoi(x+1, y+1) &&
      dv_->isVoronoi(x, y) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };

  auto IsJunctions11 = [&](const int x, const int y) {
    if (dv_->isVoronoi(x+1, y+1) && dv_->isVoronoi(x, y) &&
      dv_->isVoronoi(x-1, y-1) && dv_->isVoronoi(x+1, y-1)) {
      return true;
    }
    return false;
  };
  
  if (IsJunctions0(x, y) || IsJunctions1(x, y) || IsJunctions2(x, y) || IsJunctions3(x, y) ||
    IsJunctions4(x, y) || IsJunctions5(x, y) || IsJunctions6(x, y) || IsJunctions7(x, y) ||
    IsJunctions8(x, y) || IsJunctions9(x, y) || IsJunctions10(x, y) || IsJunctions11(x, y)) {
    return true;   
  }
  return false;
}

void DynamicVoronoiPort::FindJunctionsAndEnd(
  const std::vector<costmap_2d::MapLocation>& voronoi_diagram,
  std::vector<size_t>& line_junctions, std::vector<size_t>& line_ends)
{
  for (size_t i = 0; i < voronoi_diagram.size(); i ++) {
    const auto& v = voronoi_diagram.at(i);
    if (IsNearBoundary(v.x, v.y)) {
      line_ends.push_back(i);
    } else if (IsLineJunctions(v.x, v.y)) {
      line_junctions.push_back(i);
    } else if (IsLineEnd(v.x, v.y)) {
      line_ends.push_back(i);
    }
  }
}






// -------------------- protected --------------------




} // namespace dynamic_voronoi_port_ns
