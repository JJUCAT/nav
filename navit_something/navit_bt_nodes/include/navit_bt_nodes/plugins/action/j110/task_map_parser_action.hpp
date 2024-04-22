#ifndef TASK_MAP_PARSER_ACTION_H_
#define TASK_MAP_PARSER_ACTION_H_

#include <string>
#include <google/protobuf/util/json_util.h>
#include <navit_common/log.h>
#include <tf/tf.h>
#include "navit_bt_nodes/bt_action_node.h"
#include "navit_bt_nodes/bt_conversions.hpp"
#include "geometry_msgs/PolygonStamped.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"

#include "message_navit_map.pb.h"
namespace navit_bt_nodes
{

class TaskMapParserAction : public BT::ActionNodeBase
{
public:
    TaskMapParserAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {
        task_map_sub_ = nh_.subscribe<std_msgs::String>("/navit/map_info_update", 1, &TaskMapParserAction::taskMapCallback, this);
    }

    static BT::PortsList providedPorts()
    {
        return {
            BT::OutputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
            BT::OutputPort<std::vector<navit::protocol::map_info::MapLine>>("task_paths", "paths"),
            BT::OutputPort<std::vector<navit::protocol::map_info::MapPoint>>("task_points", "points"),
        };
    }

    void taskMapCallback(const std_msgs::StringConstPtr& msg) {
        ROS_INFO("Task map parser callback.");
        received_task_map_ = true;
        navit::protocol::map_info::MapInfo task_map_info;
        std::string json_string = msg->data;
        google::protobuf::util::JsonParseOptions options;
        options.ignore_unknown_fields = true;
        
        google::protobuf::util::JsonStringToMessage(json_string, &task_map_info, options);

        // 将task_map_info 中的内容转换为 polygons, paths, points
        std::vector<navit::protocol::map_info::MapArea> polygons;
        std::vector<navit::protocol::map_info::MapLine> paths;
        std::vector<navit::protocol::map_info::MapPoint> points;

        for (const auto& map_point : task_map_info.map_points()) {
            points.push_back(map_point);
        }

        for (const auto& map_line : task_map_info.map_lines()) {
            paths.push_back(map_line);
        }

        for (const auto& map_area : task_map_info.map_areas()) {
            polygons.push_back(map_area);
        }

        // std::vector<geometry_msgs::PolygonStamped> polygons;
        // std::vector<nav_msgs::Path> paths;
        // std::vector<geometry_msgs::PoseStamped> points;

        // for (const auto& map_point : task_map_info.map_points()) {
        //     geometry_msgs::PoseStamped pose_stamped;
        //     pose_stamped.header.stamp = ros::Time::now();
        //     pose_stamped.header.frame_id = "map";
        //     pose_stamped.header.seq = map_point.id();
        //     pose_stamped.pose.position.x = map_point.point().x();
        //     pose_stamped.pose.position.y = map_point.point().y();
        //     pose_stamped.pose.position.z = map_point.point().z();

        //     pose_stamped.pose.orientation.w = 1.0;
        //     points.push_back(pose_stamped);
        // }

        // for (const auto& map_line : task_map_info.map_lines()) {
        //     nav_msgs::Path path;
        //     path.header.stamp = ros::Time::now();
        //     path.header.frame_id = "map";
        //     path.header.seq = map_line.id();
        //     if (map_line.path().size() < 2) {
        //         continue;
        //     }

        //     for (size_t i = 0; i < map_line.path().size() - 1; ++i) {
        //         const auto& start = map_line.path(i);
        //         const auto& end = map_line.path(i + 1);

        //         geometry_msgs::PoseStamped start_pose;
        //         start_pose.pose.position.x = start.x();
        //         start_pose.pose.position.y = start.y();
        //         start_pose.pose.position.z = start.z();

        //         geometry_msgs::PoseStamped end_pose;
        //         end_pose.pose.position.x = end.x();
        //         end_pose.pose.position.y = end.y();
        //         end_pose.pose.position.z = end.z();

        //         double yaw = atan2(end.y() - start.y(), end.x() - start.x());
        //         geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);

        //         start_pose.pose.orientation = orientation;
        //         path.poses.push_back(start_pose);
        //     }

        //     const auto& last_point = map_line.path().end()[-1];
        //     geometry_msgs::PoseStamped last_pose;
        //     last_pose.pose.position.x = last_point.x();
        //     last_pose.pose.position.y = last_point.y();
        //     last_pose.pose.position.z = last_point.z();
        //     last_pose.pose.orientation = path.poses.back().pose.orientation;
        //     path.poses.push_back(last_pose);
        //     paths.push_back(path);
        // }

        // for (const auto& map_area : task_map_info.map_areas()) {
        //     geometry_msgs::PolygonStamped polygon_stamped;
        //     polygon_stamped.header.stamp = ros::Time::now();
        //     polygon_stamped.header.frame_id = "map";
        //     polygon_stamped.header.seq = map_area.id();

        //     for (const auto& point : map_area.path()) {
        //         geometry_msgs::Point32 point32;
        //         point32.x = static_cast<float>(point.x());
        //         point32.y = static_cast<float>(point.y());
        //         point32.z = static_cast<float>(point.z());
        //         polygon_stamped.polygon.points.push_back(point32);
        //     }
        //     polygons.push_back(polygon_stamped);
        // }
        setOutput("task_polygons", polygons);
        setOutput("task_paths", paths);
        setOutput("task_points", points);
    }
private:
    void halt() override
    {
    }

    BT::NodeStatus tick() override {
        if (received_task_map_) {
            // received_task_map_ = false;
            return BT::NodeStatus::SUCCESS;
        } else {
            return BT::NodeStatus::RUNNING;
        }

    };

    ros::NodeHandle nh_;
    ros::Subscriber task_map_sub_;
    bool received_task_map_ = false;
};

}  // namespace navit_bt_nodes
#endif  // TASK_MAP_PARSER_ACTION_H_
