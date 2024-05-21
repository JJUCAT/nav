#pragma once

#include <ros/ros.h>
#include <map_msgs/TotalMapInfo.h>
#include <nav_msgs/OccupancyGrid.h>

#include <boost/graph/adjacency_list.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/algorithms/buffer.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <opencv2/opencv.hpp>

namespace navit_map_server {

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;
typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>::ring_type Ring;
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
typedef boost::geometry::model::multi_polygon<Polygon> Polygons;
using GeometryGraph = std::map<int, std::tuple<std::vector<Polygon>, std::vector<Polygon>, std::vector<Ring>>>;

class OgmGenerator
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber total_map_sub_;
    ros::Publisher map_pub_;

    Polygon convertAreaToPolygon(const map_msgs::MapArea &area);
    Ring convertLineToRing(const map_msgs::MapLine &line);
    std::pair<Point, Point> convertLineToPointPair(const map_msgs::MapLine &line, const std::map<long unsigned int, Point> &points_map);

public:
    OgmGenerator() {
        total_map_sub_ = nh_.subscribe("/total_map", 1000, &OgmGenerator::totalMapCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("map", 1);
    };
    ~OgmGenerator() {};

    void totalMapCallback(const map_msgs::TotalMapInfo::ConstPtr& msg);
    bool geometryPolygon2Ogm(const GeometryGraph& geometry_graph, nav_msgs::OccupancyGrid& grid);
    bool rosmsg2geometryPolygon(const map_msgs::TotalMapInfo& total_map_msg, GeometryGraph& geometry_graph);
};

} // namespace navit_map_server