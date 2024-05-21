#include "visibility_graph_planner/visibility_graph_planner.h"
namespace navit_planner{

VisibilityGraph::VisibilityGraph(double outer_boundary_radius, double internal_collision_radius, MapInfo& map_info)
    : outer_boundary_radius_(outer_boundary_radius), internal_collision_radius_(internal_collision_radius), map_info_(map_info) {}


Polygon VisibilityGraph::generateInnerPolygonForOuterBoundary(const Polygon& inner_polygon, double outer_boundary_radius) {
    const int points_per_circle = 1;
    auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(-outer_boundary_radius);

    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> result;

    boost::geometry::buffer(inner_polygon, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);
    return result.at(0);
}

Polygons VisibilityGraph::generateInnerPolygonForInternalBoundary(const Polygons& inner_polygon, double internal_collision_radius) {
    const int points_per_circle = 1;
    auto distance_strategy = boost::geometry::strategy::buffer::distance_symmetric<double>(internal_collision_radius);

    boost::geometry::strategy::buffer::join_round join_strategy(points_per_circle);
    boost::geometry::strategy::buffer::end_round end_strategy(points_per_circle);
    boost::geometry::strategy::buffer::point_circle circle_strategy(points_per_circle);
    boost::geometry::strategy::buffer::side_straight side_strategy;
    boost::geometry::model::multi_polygon<Polygon> result;

    boost::geometry::buffer(inner_polygon, result, distance_strategy, side_strategy, join_strategy, end_strategy, circle_strategy);

    return result;
}

Polygon VisibilityGraph::simplifyPolygon(const Polygon& input, double tolerance) {
    Polygon output;
    boost::geometry::simplify(input, output, tolerance);

    boost::geometry::correct(output);
    return output;
}

std::vector<Point> VisibilityGraph::getPolygonVertices(const Polygon& polygon) {
    std::vector<Point> vertices;
    boost::geometry::for_each_point(polygon, PointCollector(vertices));
    return vertices;
}

MapInfo VisibilityGraph::processMapInfo() {
    MapInfo processed_map_info;
    for (const auto& [key, value] : map_info_) {

        Polygon outer_polygon;
        std::vector<Polygon> inner_polygons;
        std::vector<std::pair<Point, Point>> points_pairs;
        std::map<uint64_t, Point> workpoints;
        std::tie(outer_polygon, inner_polygons, points_pairs, workpoints) = value;

        // Check, correct and simplify outer_polygon
        if (!boost::geometry::is_valid(outer_polygon)) {
            boost::geometry::correct(outer_polygon);
        }
        Polygon simplified_outer_polygon;
        boost::geometry::simplify(outer_polygon, simplified_outer_polygon, 0.2); // second argument is the tolerance

        Polygon processed_outer_polygon = generateInnerPolygonForOuterBoundary(simplified_outer_polygon, outer_boundary_radius_);

        std::vector<Polygon> processed_inner_polygons;
        for (const auto& inner_polygon : inner_polygons) {
            // Check, correct and simplify each inner_polygon
            if (!boost::geometry::is_valid(inner_polygon)) {
                Polygon corrected_inner_polygon = inner_polygon;
                boost::geometry::correct(corrected_inner_polygon);

                Polygon simplified_inner_polygon;
                boost::geometry::simplify(corrected_inner_polygon, simplified_inner_polygon, 0.2); // second argument is the tolerance

                Polygons single_polygon_multi = {simplified_inner_polygon};
                Polygons processed_single_polygon_multi = generateInnerPolygonForInternalBoundary(single_polygon_multi, internal_collision_radius_);

                for (const auto& processed_inner_polygon : processed_single_polygon_multi) {
                    processed_inner_polygons.push_back(processed_inner_polygon);
                }
            }
        }

        processed_map_info[key] = std::make_tuple(processed_outer_polygon, processed_inner_polygons, points_pairs, workpoints);
    }
    return processed_map_info;
}

Graph VisibilityGraph::constructVisibilityGraph(const MapInfo& processed_map_info) {
    int next_id = 0;
    std::unordered_map<Point, int, PointHash, PointEqual> vertex_to_ids;
    for (const auto& [key, value] : processed_map_info) {
        std::unordered_map<Point, int, PointHash, PointEqual> vertex_to_id;
       
        Polygon outer_polygon;
        std::vector<Polygon> inner_polygons;
        std::vector<Polygon> handled_polygons;
        std::vector<std::pair<Point, Point>> points_pairs;
        std::map<uint64_t, Point> workpoints;

        std::tie(outer_polygon, inner_polygons, points_pairs, workpoints) = value;
        // for (const auto& point : outer_polygon.outer()) {
        //     if (!vertex_to_id.count(point)) {
        //         vertex_to_id[point] = next_id++;
        //     }
        // }                   
        for (const auto& [wp_id, wp_point] : workpoints) {
            if (!vertex_to_id.count(wp_point)) {
                vertex_to_id[wp_point] = next_id++;
                vertex_to_ids[wp_point] = next_id++;
            }
        }

        for (const auto& inner_polygon : inner_polygons) {
            for (const auto& point : inner_polygon.outer()) {
                if (!vertex_to_id.count(point)) {
                    vertex_to_id[point] = next_id++;
                    vertex_to_ids[point] = next_id++;
                }
            }
        }
        uint16_t edge_id = 0;
        for (const auto& inner_polygon : inner_polygons) {
            Polygon temp_polygon = generateInnerPolygonForOuterBoundary(inner_polygon, 0.1);
            handled_polygons.push_back(temp_polygon);
        }
   
        for (const auto& [point1, id1] : vertex_to_id) {
            for (const auto& [point2, id2] : vertex_to_id) {
                bool is_inner = false;
                bool is_outer = false;
                if (id1 != id2) {
                    boost::geometry::model::linestring<Point> line({point1, point2});
                    if (boost::geometry::crosses(line, outer_polygon)) {
                        is_outer = true;
                    }

                    for (const auto& inner_polygon : handled_polygons) {
                        if (boost::geometry::intersects(line, inner_polygon)) {
                            is_inner = true;
                        } 
                    }

                    if (!is_outer && !is_inner) {
                        float weight = boost::geometry::distance(point1, point2);
                        std::pair<Point, Point> edge_points = std::make_pair(point1, point2);
                        g_[id1].push_back(std::make_tuple(id2, weight, edge_id++, edge_points));
                    }
                }
            }
        }
        for (auto& [pointx, idx] : vertex_to_id) {
            std::cout << "idx " << idx  << "  pointx x = " << pointx.x() << "pointx y = " << pointx.y() << std::endl;
        }
        for (const auto& point_pair: points_pairs) {
            const auto& point1 = point_pair.first;
            const auto& point2 = point_pair.second;

            // Ensure point1 and point2 exist in the mapping
            if (vertex_to_ids.count(point1) == 0 || vertex_to_ids.count(point2) == 0) {
                // Log an error, throw an exception, or handle this case as needed
                continue;
            }
                
            auto point1_id = vertex_to_ids[point1];
            auto point2_id = vertex_to_ids[point2];

            // // This will mark the pair of points as having a connection relationship
            // connections_map[std::make_pair(point1_id, point2_id)] = true;

            // Also add the edge to the graph, as in your original code
            float weight = boost::geometry::distance(point1, point2);
            std::pair<Point, Point> edge_points = std::make_pair(point1, point2);
            g_[point1_id -1].push_back(std::make_tuple(point2_id - 1, weight, edge_id++, edge_points));
        }
    }   
      
    return g_;
}
} // navit_planner