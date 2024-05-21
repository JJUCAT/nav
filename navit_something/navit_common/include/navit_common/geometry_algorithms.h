#pragma once
// boost
#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/linestring.hpp>
// opencv
#include <opencv2/opencv.hpp>

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>::ring_type Ring;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
typedef boost::geometry::model::multi_polygon<Polygon> Polygons;
typedef boost::geometry::model::multi_polygon<Polygon>::const_iterator PolygonIterator;
typedef boost::geometry::model::linestring<Point> LineString;

namespace navit_common {

namespace geometry {
    /*
     * @brief 调整多边形边界
     *
     * @param input_polygon 输入多边形
     * @param expand_distance 扩展距离，正值为扩展，负值为收缩
     * @return Polygon 调整后的多边形
     */
    Polygon adjustPolygonBoundary(const Polygon& input_polygon, double expand_distance);

    /*
     * @brief 调整多边形集合的边界
     *
     * @param input_polygon 输入多边形集合
     * @param expand_distance 扩展距离，正值为扩展，负值为收缩
     * @return Polygons 调整后的多边形集合
     */
    Polygons adjustPolygonsBoundary(const Polygons& input_polygon, double expand_distance);
    /*
     * @brief 调整多边形集合的边界
     *
     * @param input_polygon 输入多边形集合
     * @param expand_distance 扩展距离，正值为扩展，负值为收缩
     * @return Polygons 调整后的多边形集合
     */
    std::vector<Polygon> adjustPolygonsBoundary(const std::vector<Polygon>& input_polygon, double expand_distance);
    /*
     * @brief 简化多边形
     *
     * @param input 输入多边形
     * @param tolerance 简化容差
     * @return Polygon 简化后的多边形
     */

    Polygon simplifyPolygon(const Polygon& input, const double tolerance);

     /*
     * @brief 修正多边形
     *
     * @param input 输入多边形
     * @return Polygon 修正后的多边形
     */
    void correctPolygon(Polygons& input) {
        boost::geometry::correct(input);
    }
    /*
     * @brief 修正轨迹
     *
     * @param input 输入轨迹
     * @return Polygon 修正后的轨迹
     */
    void correctPolygon(LineString& input) {
        boost::geometry::correct(input);
    }

    /*
     * @brief 获取多边形的顶点
     *
     * @param polygon 多边形
     * @return std::vector<Point> 多边形的顶点
     */
    std::vector<Point> getPolygonVertices(const Polygon& polygon);

    /*
     * @brief 将多边形转换为图, 我们认为multipolygons的第一个多边形为外边界，其余的为内部不可行驶区域
     *
     * @param multipolygons 外围边界+内部不可行驶区域
     * @return Graph 图
     */
    cv::Mat converGeometryToImage(const Polygons& multipolygons);
    /*
     * @brief 获取多边形的面积
     * @parm coverage_area 多边形, holes_polygon 多边形的洞
     * @return float 多边形的面积
     */
    float getPolygonWithHolesArea(const Polygon& coverage_area, const std::vector<Polygon>& holes_polygon);
    /*
     * @brief 获取多边形的面积
     * @parm coverage_area 多边形
     * @return float 多边形的面积
     */
    float getPolygonArea(const Polygon& coverage_area) {
        return boost::geometry::area(coverage_area);
    };

    /*
     * @brief 判断点是否在多边形内
     * @parm pose 点, polygon 多边形
     * @return bool 点是否在多边形内
     */
    bool isPoseInPolygon(const Point& pose, const Polygon& polygon);

    /*
     * @brief 判断点是否在多边形集合内
     * @parm pose 点, polygons 多边形集合
     * @return bool 点是否在多边形集合内
     */
    bool isPoseInPolygons(const Point& pose, const std::vector<Polygon>& polygons);

    struct PointCollector {
        std::vector<Point>& points;

        PointCollector(std::vector<Point>& points) : points(points) {}

        void operator()(const Point& p) const {
            points.push_back(p);
        }
    };
    /**/
    float distance(const Point& p1, const Point& p2) {
        return boost::geometry::distance(p1, p2);
    }
} //namespace geometry
} //namesapce navit_algrithms