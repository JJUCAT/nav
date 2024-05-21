
#include <navit_map_server/ogm_generator.h>

namespace navit_map_server {

Polygon OgmGenerator::convertAreaToPolygon(const map_msgs::MapArea &area) {
    Polygon polygon;
    for (const auto &pose_msg : area.path.poses) {
        double x = pose_msg.pose.position.x;
        double y = pose_msg.pose.position.y;
        polygon.outer().push_back(Point(x, y));
    }
    return polygon;
}
// 
Ring OgmGenerator::convertLineToRing(const map_msgs::MapLine &line) {
    Ring ring;
    for (const auto &line : line.path.poses) {
        double x = line.pose.position.x;
        double y = line.pose.position.y;
        ring.push_back(Point(x, y));
    }
    return ring;
}
// 
std::pair<Point, Point> OgmGenerator::convertLineToPointPair(const map_msgs::MapLine &line, const std::map<long unsigned int, Point> &points_map) {
    Point start_point = points_map.at(line.start_point_id);
    Point end_point = points_map.at(line.end_point_id);

    return std::make_pair(start_point, end_point);
}

void OgmGenerator::totalMapCallback(const map_msgs::TotalMapInfo::ConstPtr& msg)
{
    GeometryGraph geometry_graph;
    if (!rosmsg2geometryPolygon(*msg, geometry_graph)) {
        ROS_WARN("Failed to convert TotalMapInfo to AreasInfo");
    }
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = "map";
    if (geometryPolygon2Ogm(geometry_graph, grid)) {
        ROS_INFO("Success to convert AreasInfo to OGM");
    }
    else {
        ROS_WARN("Failed to convert AreasInfo to OGM");
    }
}
//TODO:(czk) add expand distance config
bool OgmGenerator::geometryPolygon2Ogm(const GeometryGraph& geometry_graph, nav_msgs::OccupancyGrid& grid) {
    std::vector<std::vector<cv::Point2f>> boundary_polygons, forbidden_polygons, lines_polygons;
    std::vector<cv::Point2f> all_points;
    const int points_per_circle = 1;
    double expand_distance = 0.2;

    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> out_polygon;

    for (const auto& graph : geometry_graph) {
        const auto& [boundary_areas, forbidden_areas, lines] = graph.second;
        for (const auto& polygon : boundary_areas) {
            auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(expand_distance);
            boost::geometry::buffer(polygon, out_polygon, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
            std::vector<cv::Point2f> points;
            for (const auto& point : out_polygon[0].outer()) {
                points.push_back(cv::Point2f(point.x(), point.y()));
                all_points.push_back(cv::Point2f(point.x(), point.y()));
            }
            boundary_polygons.push_back(points);
        }
        for (const auto& polygon : forbidden_areas) {
            auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(-expand_distance);
            boost::geometry::buffer(polygon, out_polygon, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
            std::vector<cv::Point2f> points;
            for (const auto& point : out_polygon[0].outer()) {
                points.push_back(cv::Point2f(point.x(), point.y()));
                all_points.push_back(cv::Point2f(point.x(), point.y()));
            }
            forbidden_polygons.push_back(points);
        }
        for (const auto& ring : lines) {
            std::vector<cv::Point2f> points;
            for (const auto& point : ring) {
                points.push_back(cv::Point2f(point.x(), point.y()));
                all_points.push_back(cv::Point2f(point.x(), point.y()));
            }
            lines_polygons.push_back(points);
        }
    }

    cv::Rect2f bounding_box = cv::boundingRect(all_points);
    float resolution = 0.05;
    cv::Size image_size(std::ceil(bounding_box.width / resolution), std::ceil(bounding_box.height / resolution));

    cv::Mat image = cv::Mat::ones(image_size, CV_8UC1) * 0;

    std::vector<std::vector<cv::Point>> boundary_pixels, forbidden_pixels, lines_pixels;
    for (const auto& polygons : {std::make_pair(boundary_polygons, &boundary_pixels),
                                std::make_pair(forbidden_polygons, &forbidden_pixels),
                                std::make_pair(lines_polygons, &lines_pixels)}) {
        for (const auto& polygon : polygons.first) {
            std::vector<cv::Point> pixel_points;
            for (const auto& point : polygon) {
                pixel_points.push_back(cv::Point(std::round((point.x - bounding_box.x) / resolution),
                                                std::round((point.y - bounding_box.y) / resolution)));
            }
            polygons.second->push_back(pixel_points);
        }
    }

    cv::fillPoly(image, boundary_pixels, cv::Scalar(255));
    cv::fillPoly(image, forbidden_pixels, cv::Scalar(0));

    double buffer_distance = 1.0 * resolution;
    for (const auto& ring : lines_polygons) {
        std::vector<cv::Point2f> points;
        boost::geometry::model::linestring<Point> boost_line;
        for (const auto& point : ring) {
            points.push_back(cv::Point2f(point.x, point.y));
            boost_line.push_back(Point(point.x, point.y));
        }

        const int points_per_circle = 1;
        auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(buffer_distance);
        boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
        boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
        boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
        boost::geometry::strategy::buffer::side_straight side_strategy;
        boost::geometry::model::multi_polygon<Polygon> result;

        boost::geometry::buffer(boost_line, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

        for (const auto& poly : result) {
            std::vector<cv::Point> buffer_pixels;
            for (const auto& point : poly.outer()) {
                buffer_pixels.push_back(cv::Point(std::round((point.x() - bounding_box.x) / resolution), 
                                                std::round((point.y() - bounding_box.y) / resolution)));
            }
            cv::fillPoly(image, std::vector<std::vector<cv::Point>>{buffer_pixels}, cv::Scalar(255));
        }
    }

    // Create a nav_msgs/OccupancyGrid message
    grid.header.frame_id = "map";
    grid.info.resolution = resolution;
    grid.info.width = image_size.width;
    grid.info.height = image_size.height;
    grid.info.origin.position.x = bounding_box.x;
    grid.info.origin.position.y = bounding_box.y;
    grid.info.origin.orientation.w = 1.0; // Identity quaternion
    grid.data.resize(grid.info.width * grid.info.height);
    for (int i = 0; i < image_size.height; ++i) {
        for (int j = 0; j < image_size.width; ++j) {
            grid.data[i * grid.info.width + j] = image.at<uint8_t>(i, j) > 0 ? 0 : 255;
        }
    }

    sleep(2);
    // Publish the grid
    map_pub_.publish(grid);
    return true;
}

bool OgmGenerator::rosmsg2geometryPolygon(const map_msgs::TotalMapInfo& total_map_msg, GeometryGraph& geometry_graph) {
    int i = 0;
    for (const auto &map_info_msg : total_map_msg.map_infos) {
        std::vector<Polygon> boundary_areas;
        std::vector<Polygon> forbidden_areas;
        std::vector<Ring> lines;

        // convert ros map areas to geometry polygons
        for (const auto &area_msg : map_info_msg.map_areas) {
            if(area_msg.type == map_msgs::MapArea::AREA_TYPE_FULL_COVERAGE) {
                boundary_areas.push_back(convertAreaToPolygon(area_msg));
            }
            else if(area_msg.type == map_msgs::MapArea::AREA_TYPE_FORBIDDEN_AREA) {
                forbidden_areas.push_back(convertAreaToPolygon(area_msg));
            }
        }
        
        for (const auto &line_msg : map_info_msg.map_lines) {
            if (line_msg.type == map_msgs::MapLine::CONNECTION_LINE) {
                lines.push_back(convertLineToRing(line_msg));
            }
        }
        geometry_graph[i] = std::tuple(boundary_areas, forbidden_areas, lines);
        i++;
    }
    return true;
}
} // namespace navit_map_server

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ogm_generator_node");

    navit_map_server::OgmGenerator ogm_generator;

    ros::spin();

    return 0;
}