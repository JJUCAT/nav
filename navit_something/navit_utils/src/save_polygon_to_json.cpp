#include <ros/ros.h>
#include <navit_utils/GetSelection.h>
#include <navit_utils/GetPolylines.h>
#include <navit_utils/GetPoints.h>
#include <fstream>
#include <jsoncpp/json/json.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "save_polygon_node");
    ros::NodeHandle nh;

    ros::ServiceClient clientSelection = nh.serviceClient<navit_utils::GetSelection>("get_selection");
    ros::ServiceClient clientConnectionLine = nh.serviceClient<navit_utils::GetPolylines>("get_connection_line");
    ros::ServiceClient clientWorkPoints = nh.serviceClient<navit_utils::GetPoints>("get_work_points");

    navit_utils::GetSelection srvSelection;
    navit_utils::GetPolylines srvConnectionLine;
    navit_utils::GetPoints srvWorkPoints;

    if (clientSelection.call(srvSelection) && clientConnectionLine.call(srvConnectionLine) && clientWorkPoints.call(srvWorkPoints))
    {
        Json::Value root;
        int id = 1;

        for (const auto& poly_array : srvSelection.response.selection)
        {
            Json::Value json_poly;
            json_poly["id"] = id++;
            Json::Value json_boundary;
            Json::Value json_obstacles;

            for (size_t i = 0; i < poly_array.polygons.size(); ++i)
            {
                const auto& polygon = poly_array.polygons[i];
                Json::Value json_polygon;

                for (const auto& point : polygon.polygon.points)
                {
                    Json::Value json_point;
                    json_point.append(point.x);
                    json_point.append(point.y);
                    json_polygon.append(json_point);
                }

                if (i == 0)
                    json_boundary = json_polygon;
                else
                    json_obstacles.append(json_polygon);
            }

            json_poly["boundary"] = json_boundary;
            json_poly["obstacles"] = json_obstacles;

            // Add connections 
            Json::Value json_connections;
            for (const auto& polyline : srvConnectionLine.response.polylines) {
                Json::Value json_polyline;
                for (const auto& point : polyline.polyline) {
                    Json::Value json_point;
                    json_point.append(point.x);
                    json_point.append(point.y);
                    json_polyline.append(json_point);
                }
                json_connections.append(json_polyline);
            }
            json_poly["connections"] = json_connections;
            // Add work points 
            Json::Value json_workpoints;
            for (const auto& point : srvWorkPoints.response.points)
            {
                Json::Value json_point;
                json_point.append(point.x);
                json_point.append(point.y);
                json_workpoints.append(json_point);
            }
            json_poly["work_points"] = json_workpoints;

            root.append(json_poly);
        }

        Json::StreamWriterBuilder writerBuilder;
        std::unique_ptr<Json::StreamWriter> writer(writerBuilder.newStreamWriter());
        std::ofstream outputFileStream("/home/yjh/test/polygons.json");
        writer->write(root, &outputFileStream);
    }
    else
    {
        ROS_ERROR("Failed to call service get_selection or get_connection_line or get_work_points");
        return 1;
    }

    return 0;
}