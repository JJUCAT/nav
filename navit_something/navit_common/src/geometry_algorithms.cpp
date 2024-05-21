#include <navit_common/geometry_algorithms.h>

namespace navit_common {
namespace geometry {

Polygon adjustPolygonBoundary(const Polygon& input_polygon, double expand_distance) {
    const int points_per_circle = 1;
    auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(expand_distance);

    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> out_polygon;
    out_polygon.clear();

    try {
        boost::geometry::buffer(input_polygon, out_polygon, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
    } catch (const std::exception& e) {
        for (const Point &p : input_polygon.outer()) {
            std::cout << "x: " << p.x() << " y: " << p.y() << std::endl;
        }
        std::cout << "Error " << e.what() << std::endl;
    }

    if (out_polygon.empty()) {
        return Polygon();
    } else {
        return out_polygon.at(0);
    }
    return out_polygon.at(0);
}


Polygons adjustPolygonsBoundary(const Polygons& input_polygon, double expand_distance) {
    const int points_per_circle = 1;
    auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(expand_distance);

    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> outer_polyons;

    boost::geometry::buffer(input_polygon, outer_polyons, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

    return outer_polyons;
}
std::vector<Polygon> adjustPolygonsBoundary(const std::vector<Polygon>& input_polygon, double expand_distance) {
    const int points_per_circle = 1;
    auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(expand_distance);


    //convert input polygon to
    Polygons polyons;
    for (const auto& poly : input_polygon) {
        polyons.push_back(poly);
    }
    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> outer_polyons;

    boost::geometry::buffer(polyons, outer_polyons, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

    for (auto& poly : outer_polyons) {
        boost::geometry::correct(poly);
    }
    std::vector<Polygon> outer_polyons_vec;
    for (auto& poly : outer_polyons) {
        outer_polyons_vec.push_back(poly);
    }
    return outer_polyons_vec;
}
Polygon simplifyPolygon(const Polygon& input_polygon, const double tolerance) {
    Polygon output_polygon;
    boost::geometry::simplify(input_polygon, output_polygon, tolerance);
    boost::geometry::correct(output_polygon);
    return output_polygon;
}

std::vector<Point> getPolygonVertices(const Polygon& polygon) {
    std::vector<Point> vertices;
    boost::geometry::for_each_point(polygon, PointCollector(vertices));
    return vertices;
}

cv::Mat converGeometryToImage(const Polygons& multipolygons) {
    std::vector<std::vector<cv::Point2f>> polygons;
    for (const auto& poly : multipolygons) {
        std::vector<cv::Point2f> polygon_points;
        for (const auto& point : boost::geometry::exterior_ring(poly)) {
            polygon_points.push_back(cv::Point2f(boost::geometry::get<0>(point), boost::geometry::get<1>(point)));
        }
        polygons.push_back(polygon_points);
    }

    // Calculate the bounding box for all polygons
    std::vector<cv::Point2f> all_points;
    for (const auto& polygon : polygons) {
        all_points.insert(all_points.end(), polygon.begin(), polygon.end());
    }
    cv::Rect2f bounding_box = cv::boundingRect(all_points);

    float resolution = 0.05;
    cv::Size image_size(std::ceil(bounding_box.width / resolution), std::ceil(bounding_box.height / resolution));
    cv::Mat image = cv::Mat::zeros(image_size, CV_8UC1);
    for (size_t i = 0; i < polygons.size(); ++i) {
        std::vector<cv::Point> pixel_points;
        for (const auto& point : polygons[i]) {
            pixel_points.push_back(cv::Point(static_cast<int>((point.x - bounding_box.x) / resolution + 0.5),
                                                static_cast<int>((point.y - bounding_box.y) / resolution + 0.5)));
        }

        if (!pixel_points.empty()) {
            std::vector<std::vector<cv::Point>> polys(1, pixel_points); // create a vector of polygon
            cv::fillPoly(image, polys, cv::Scalar(i == 0 ? 254 : 0)); // fill the polygons, 255 for outer boundary, 0 for inner boundaries
        } else {
            std::cout << "Warning: pixel_points is empty for polygon " << i << std::endl;
        }
    }
    return image;
}

float getPolygonWithHolesArea(const Polygon& coverage_area, const std::vector<Polygon>& holes_polygon) {
    std::vector<Polygon> output {coverage_area};
    for (const auto& hole : holes_polygon) {
        std::vector<Polygon> temp_output;
        for (const auto& out_poly : output) {
            std::vector<Polygon> difference_output;
            boost::geometry::difference(out_poly, hole, difference_output);
            temp_output.insert(temp_output.end(), difference_output.begin(), difference_output.end());
        }
        output = temp_output;
    }

    double total_area = 0.0;
    for (const auto& p : output) {
        total_area += boost::geometry::area(p);
    }
    return total_area;
}

bool isPoseInPolygon(const Point& pose, const Polygon& polygon) {
    return boost::geometry::within(pose, polygon);
}

bool isPoseInPolygons(const Point& pose, const std::vector<Polygon>& polygons) {
    for (const auto& polygon : polygons) {
        if (isPoseInPolygon(pose, polygon)) {
            return true;
        }
    }
    return false;
}
} // namespace geometry
} // namespace navit_common