#ifndef PRM_PLANNER_LOAD_JSON_H_
#define PRM_PLANNER_LOAD_JSON_H_

#include <fstream>
#include <jsoncpp/json/json.h>
#include <visibility_graph_planner/visibility_graph_planner.h>

namespace navit_planner {
class PlanningSceneLoader {
public:
    PlanningSceneLoader() {}

    /* @brief Read map info from json file
     * @param filename The json file name
     * @param map_info The map info
     * @return void
    */
void ReadMapInfoFromJson(const std::string& filename, MapInfo* map_info) {
    std::ifstream file(filename, std::ifstream::binary);
    Json::Value root;
    file >> root;
    uint64_t wp_id = 0;    
    // First loop: build map_info without workpoints
    for (const auto& item : root) {
        if (item["id"].isNull() || item["boundary"].isNull()) continue;
        int id = item["id"].asInt();

        Polygon boundary;
        for (const auto& point : item["boundary"]) {
            boost::geometry::append(boundary, Point(point[0].asDouble(), point[1].asDouble()));
        }
      
        std::vector<Polygon> obstacles;
        if (!item["obstacles"].isNull()) {
            for (const auto& obs : item["obstacles"]) {
                if (obs.isNull()) continue; // Skip if the obstacle is null
                Polygon polygon;
                for (const auto& point : obs) {
                    boost::geometry::append(polygon, Point(point[0].asDouble(), point[1].asDouble()));
                }
                obstacles.push_back(polygon);
            }
        }

        std::vector<std::pair<Point, Point>> connections;
        if (!item["connections"].isNull()) {
            for (const auto& connection : item["connections"]) {
                connections.emplace_back(Point(connection[0][0].asDouble(), connection[0][1].asDouble()),
                                        Point(connection[1][0].asDouble(), connection[1][1].asDouble()));
            }
        }
        std::map<uint64_t, Point> workpoints;  // This will be empty for now

        map_info->emplace(id, std::make_tuple(boundary, obstacles, connections, workpoints));

    }

    // Second loop: go through workpoints and assign them to the correct boundary
    for (const auto& item : root) {
       
        if (!item["work_points"].isNull()) {
            for (const auto& wp : item["work_points"]) {
                Point location(wp[0].asDouble(), wp[1].asDouble());
                for(auto& [boundary_id, boundary_data] : *map_info) {
                    Polygon& boundary_polygon = std::get<0>(boundary_data);
                    boost::geometry::correct(boundary_polygon);
                    if(boost::geometry::within(location, boundary_polygon)) {
                        std::get<3>(boundary_data)[wp_id++] = location;  // Add workpoint to the correct boundary
                        break;
                    }
                }
            }
        }
        
        // Add connections to the workpoints
        if (!item["connections"].isNull()) {
            for (const auto& connection : item["connections"]) {
                Point start_conn(connection[0][0].asDouble(), connection[0][1].asDouble());
                Point end_conn(connection[1][0].asDouble(), connection[1][1].asDouble());
                for(auto& [boundary_id, boundary_data] : *map_info) {
                    Polygon& boundary_polygon = std::get<0>(boundary_data);
                    boost::geometry::correct(boundary_polygon);
                    if(boost::geometry::within(start_conn, boundary_polygon)) {
                        std::get<3>(boundary_data)[wp_id++] = start_conn;  // Add start connection to the correct boundary
                    }

                    if(boost::geometry::within(end_conn, boundary_polygon)) {
                        std::get<3>(boundary_data)[wp_id++] = end_conn;  // Add end connection to the correct boundary
                    }
                    if (boundary_id == 0) {
                        std::get<2>(boundary_data).emplace_back(start_conn, end_conn);  // Add connection to the correct boundary 
                    }
                }
                
                
            }
        }

    }
}
   
private:
};
} // namespace navit_planner
#endif // NAVIT_PLANNER_MAP_H