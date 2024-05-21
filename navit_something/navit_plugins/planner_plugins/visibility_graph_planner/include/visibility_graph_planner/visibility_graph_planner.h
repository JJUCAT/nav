#ifndef VISIBILITY_GRAPH_H_
#define VISIBILITY_GRAPH_H_

#include <utility>
#include <vector>
#include <iostream>
#include <chrono>
// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/buffer.hpp>

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>::ring_type Ring;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
typedef boost::geometry::model::multi_polygon<Polygon> Polygons;

namespace navit_planner {

using MapInfo = std::map<int, std::tuple<Polygon, std::vector<Polygon>, std::vector<std::pair<Point, Point>>, std::map<uint64_t, Point>>>;

using Graph = std::unordered_map<int, std::vector<std::tuple<int, float, uint16_t, std::pair<Point, Point>>>>;

//using Graph = adjacency_list<vecS, vecS, undirectedS, no_property, property<edge_weight_t, double>>;

struct PointHash {
    std::size_t operator()(const Point& p) const {
        std::size_t h1 = std::hash<double>()(p.x());
        std::size_t h2 = std::hash<double>()(p.y());
        return h1 ^ h2;  // Combine hashes
    }
};

struct PointEqual {
    bool operator()(const Point& p1, const Point& p2) const {
        return std::abs(p1.x() - p2.x()) < std::numeric_limits<double>::epsilon() && std::abs(p1.y() - p2.y()) < std::numeric_limits<double>::epsilon();
    }
};
class VisibilityGraph {
    public:
    // 使用智能指针管理对象
    using Ptr = std::shared_ptr<VisibilityGraph>;
    /*
    * @brief Construct a new Visibility Graph object 
    * 
    * @param outer_boundary_radius       外边界半径, 用于生成外边界的内接多边形
    * @param internal_collision_radius   内部碰撞半径，用于生成不可行驶区域的外接多边形     
    * @param map_info                    地图信息
    */
    VisibilityGraph(double outer_boundary_radius, double internal_collision_radius, MapInfo& map_info);

    /*
    * @brief 设置外边界半径
    * 
    * @param radius 外边界半径 单位 m
    * @return bool
    */  
    bool SetOuterBoundaryRadius(double radius) {
        outer_boundary_radius_ = radius;
        return true;
    };

    /*
    * @brief 获取外边界半径
    * 
    * @return double 外边界半径 单位 m
    */
    double GetOuterBoundaryRadius() const {
        return outer_boundary_radius_;
    };

    /*
    * @brief 设置内部碰撞半径
    * 
    * @param radius 内部碰撞半径 单位 m
    * 
    * @return bool
    */
    bool SetInternalCollisionRadius(double radius) {
        internal_collision_radius_ = radius;
        return true;
    };

    /*
    * @brief 获取内部碰撞半径
    * 
    * @return double 内部碰撞半径 单位 m
    */
    double GetInternalCollisionRadius() const {
        return internal_collision_radius_;
    };

    /*
    * @brief 设置地图信息
    * 
    * @param map_info 地图信息
    * 
    * @return bool 
    */
    bool setMapInfo(MapInfo& map_info) {
            map_info_ = map_info;
            //TODO(czk): check map_info 
            return true;
    }
    /*
    * @brief 获取地图信息
    * 
    * @return MapInfo 地图信息
    */
    MapInfo getMapInfo() const{
            return map_info_;
    }
        
    /* @brief 设置起点和终点
    * 
    * @param start_point 起点
    * @param goal_point 终点
    * 
    * @return bool
    */
    bool setStartGoalPoint(const Point& start_point, const Point& goal_point) {
        start_point_ = start_point;
        goal_point_ = goal_point;
        return true;
    }
    MapInfo processMapInfo();

    Graph getGraph() const {
        return g_;
    }

    /* @brief 构建可见图
     *
     * @param processed_map_info 处理后的地图信息
     * 
     * @return Graph 可见图
     */
    Graph constructVisibilityGraph(const MapInfo& processed_map_info);
 private:
  
    /*
    * @brief 生成外边界的内接多边形
    * 
    * @param outer_boundary_radius 外边界半径
    * 
    * @return Polygon 内接多边形
    */
    Polygon generateInnerPolygonForOuterBoundary(const Polygon& inner_polygon, double outer_boundary_radius);

    /* @brief 生成内边界的外接多边形
     *
     * @param internal_collision_radius 内接半径
     * 
     * return Polygon 外接多边形
     */
    Polygons generateInnerPolygonForInternalBoundary(const Polygons& inner_polygon, double internal_collision_radius);

    /* @brief 简化多边形 
     *
     * @param input 多边形
    */
    Polygon simplifyPolygon(const Polygon& input, double tolerance);
    
    struct PointCollector {
        std::vector<Point>& points;

        PointCollector(std::vector<Point>& points) : points(points) {}

        void operator()(const Point& p) const {
            points.push_back(p);
        }
    };

    /* @brief 获取多边形的顶点
     *
     * @param polygon 多边形
     * 
     * @return std::vector<Point> 多边形的顶点
     */
    std::vector<Point> getPolygonVertices(const Polygon& polygon);

    double outer_boundary_radius_ = 0.0;     //init 0.0
    double internal_collision_radius_ = 0.0; //init 0.0
    
    MapInfo map_info_;
    Point start_point_, goal_point_;
    Graph g_;
};
} // navit_planner
#endif  // VISIBILITY_GRAPH_H_