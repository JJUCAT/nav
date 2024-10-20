/**
 * @copyright Copyright (c) {2022} LMR
 * @author LMR (lmr2887@163.com)
 * @date 2024-04-24
 * @brief 
 */

#ifndef _VORONOI_FIELD__DYNAMIC_VORONOI_PORT_H_
#define _VORONOI_FIELD__DYNAMIC_VORONOI_PORT_H_

#include <costmap_2d/costmap_2d.h>
#include <dynamic_voronoi/dynamicvoronoi.h>

namespace dynamic_voronoi_port_ns
{

class DynamicVoronoiPort
{
 public:

  DynamicVoronoiPort(const int sizeX, const int sizeY,
    const std::vector<costmap_2d::MapLocation>& obstacles, const bool boundary=false);

  ~DynamicVoronoiPort();

  void GetVoronoiDiagram(std::vector<costmap_2d::MapLocation>& voronoi_diagram);

  void GetRawVoronoiDiagram(std::vector<costmap_2d::MapLocation>& voronoi_diagram);

  void GetRawJunctions(std::vector<size_t>& junctions);

  void GetJunctions(std::vector<costmap_2d::MapLocation>& junctions);

  void GetPruneVoronoiDiagram(std::vector<costmap_2d::MapLocation>& branch);

  float GetDistance2Obstacle(const int x, const int y);

  void Save(const char* filename);

 protected:

  /**
   * @brief  是否是边界
   * @param  x  栅格 x
   * @param  y  栅格 y
   * @return true 
   * @return false 
   */
  bool IsBoundary(const int x, const int y);

  /**
   * @brief  是否靠近边界
   * @param  x  栅格 x
   * @param  y  栅格 y
   * @return true 
   * @return false 
   */
  bool IsNearBoundary(const int x, const int y);

  /**
   * @brief  是否是端点，需要检查邻近 8 个栅格，使用前需要排除 [x,y] 不靠近边界
   * @param  gvd  voronoi 网络
   * @param  x  栅格 x
   * @param  y  栅格 y
   * @return true 
   * @return false 
   */
  bool IsLineEnd(const std::vector<costmap_2d::MapLocation>& gvd, const int x, const int y);

  /**
   * @brief  是否是交叉点，需要检查邻近 8 个栅格，使用前需要排除 [x,y] 不靠近边界
   * @param  gvd  voronoi 网络
   * @param  x  栅格 x
   * @param  y  栅格 y
   * @return true 
   * @return false 
   */
  bool IsLineJunctions(const std::vector<costmap_2d::MapLocation>& gvd, const int x, const int y);

  /**
   * @brief  找交点和端点
   * @param  voronoi_diagram  voronoi 网络
   * @param  line_junctions  交点
   * @param  line_ends  端点
   */
  void FindJunctionsAndEnds(
    const std::vector<costmap_2d::MapLocation>& voronoi_diagram,
    std::vector<size_t>& line_junctions, std::vector<size_t>& line_ends);

  /**
   * @brief  找端点
   * @param  voronoi_diagram  voronoi 网络
   * @param  ignore  排除的点
   * @param  line_ends  端点
   */
  void FindEnds(const std::vector<costmap_2d::MapLocation>& voronoi_diagram,
    const std::vector<size_t>& ignore, std::vector<size_t>& line_ends);


 private:

  std::shared_ptr<DynamicVoronoi> dv_;
  int sizeX_, sizeY_;
  bool **map_ = nullptr;
  bool boundary_{false};
  int boundary_size_{1};
  std::vector<costmap_2d::MapLocation> gvd_;
  std::vector<size_t> junctions_;

}; // class DynamicVoronoiPort

} // dynamic_voronoi_port_ns

#endif // _VORONOI_FIELD__DYNAMIC_VORONOI_PORT_H_
