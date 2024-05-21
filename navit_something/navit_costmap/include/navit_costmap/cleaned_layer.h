#ifndef CLEANED_LAYER_H
#define CLEANED_LAYER_H

#include <ros/ros.h>
#include <geometry_msgs/Polygon.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <vector>
#include <map>
#include <tf/transform_listener.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/graph/adjacency_list.hpp>

#include <opencv2/opencv.hpp>
namespace navit_costmap
{
    typedef boost::geometry::model::d2::point_xy<double> Point;
    typedef boost::geometry::model::polygon<Point> Polygon;
    typedef boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>>::ring_type Ring;
    typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph;
    typedef std::map<int, std::tuple<std::vector<Polygon>, std::vector<Polygon>, std::vector<Ring>>> GeometryGraph;
    typedef boost::geometry::model::multi_polygon<Polygon> multi_polygon;

    class CleanedLayer
    {
    public:
        CleanedLayer() {};
        ~CleanedLayer() {};
        void onInitialize(ros::NodeHandle& nh);
        Polygon convertToBoostPolygon(const geometry_msgs::Polygon& ros_polygon);
        void setPolygonMap(geometry_msgs::Polygon bounding_polygon, std::vector<geometry_msgs::Polygon> forbidden_zones_polygons);
        void updateBounds(double robot_x, double robot_y, double robot_yaw, Polygon& robot_polygon);
        double calCleanedArea(const std::vector<Polygon>& cleaned_areas);
        void mapUpdateLoop(double frequency);
        void publishFilledPolygon(const Polygon& polygon, int id, const std::string& ns, const std_msgs::ColorRGBA& color);
    private:
        bool haveIntersection(const Polygon& poly1, const Polygon& poly2);
        Polygon mergePolygons(const Polygon& poly1, const Polygon& poly2);
        nav_msgs::OccupancyGrid generateOccupancyGrid(double resolution);
        bool isPointInPolygon(Point point, Polygon polygon);
        bool isPointInAnyForbiddenZone(Point point, std::vector<Polygon> forbidden_zones);

        bool makePlan(const cv::Mat& map, std::vector<nav_msgs::Path>& edge_paths, std::vector<Polygon>& center_points);
        ros::ServiceServer enable_swept_layer_service_;
        ros::ServiceServer clear_swept_layer_service_;
        ros::ServiceServer get_swept_layer_service_;

        std::vector<geometry_msgs::Point> footprint_;
        Polygon footprint_polygon_;
        std::vector<Polygon> cleaned_polygons_;
        double whole_area_ = 0.0;
        double update_frequency_;
        bool enabled_;

        Polygon bounding_polygon_;
        std::vector<Polygon> forbidden_zones_polygons_;
        ros::Publisher marker_pub_, map_pub_, cleaned_map_pub_, edge_paths_pub_;
        ros::NodeHandle nh_;
        tf::TransformListener tf_listener_;
        Polygon result_polygons_;
        multi_polygon multi_polygon_result_;
        boost::geometry::model::multi_polygon<Polygon> result_polygon_;
        boost::geometry::model::multi_polygon<Polygon> result_cleaned_polygon_;

        nav_msgs::OccupancyGrid cleaned_grid_;
        int dilation_size_ = 1;
        double map_resolution_ = 0.05;
    };
}

#endif // CLEANED_LAYER_H