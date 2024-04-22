#ifndef NAVIT_BT_NODES__PLUGINS__ACTION__TASK_COMMAND_SERVICE_ACTION_HPP_
#define NAVIT_BT_NODES__PLUGINS__ACTION__TASK_COMMAND_SERVICE_ACTION_HPP_
#include <navit_msgs/TaskCommand.h>
#include <nav_msgs/Path.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>
#include <navit_common/path_handle.h>
#include "navit_bt_nodes/bt_service_node.h"
#include "geometry_msgs/PolygonStamped.h"
#include <tf2_ros/transform_listener.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include "message_navit_map.pb.h"

namespace navit_bt_nodes
{

class TaskCommandAction : public BT::ActionNodeBase
{
public:
    TaskCommandAction(const std::string& xml_tag_name, const BT::NodeConfiguration& conf) : BT::ActionNodeBase(xml_tag_name, conf)
    {
        task_com_sub_ = nh_.subscribe<navit_msgs::TaskCommand>("/navit/task_command", 1, &TaskCommandAction::taskCmdCallback, this);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf2_ros::TransformListener tf_listener_(*tf_buffer_);
    }

  static BT::PortsList providedPorts()
  {
      return {
          BT::InputPort<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", "polygons"),
          BT::InputPort<std::vector<navit::protocol::map_info::MapLine>>("task_paths", "paths"),
          BT::InputPort<std::vector<navit::protocol::map_info::MapPoint>>("task_points", "points"),
          BT::InputPort<float>("final_dock_distance", "final_dock_dis"),

          BT::OutputPort<bool>("task_cmd_updated"),
          BT::OutputPort<navit::protocol::map_info::MapArea>("task_polygon"),
          BT::OutputPort<navit::protocol::map_info::MapLine>("task_path"),
          BT::OutputPort<nav_msgs::Path>("task_path_ros"),
          BT::OutputPort<navit::protocol::map_info::MapPoint>("task_point"),
          BT::OutputPort<geometry_msgs::PoseStamped>("task_pose_ros"),
          BT::OutputPort<std::string> ("task_cmd"),
          BT::OutputPort<geometry_msgs::PoseStamped>("nearby_charge_pose_ros"),
          BT::OutputPort<geometry_msgs::PoseStamped>("charge_pose_ros"),
          BT::OutputPort<geometry_msgs::PoseStamped>("final_dock_pose_ros"),
          BT::OutputPort<int>("repeat_times"),
          BT::OutputPort<std::vector<std::string>>("task_polygon_ids"),
          BT::OutputPort<std::string>("task_goal_area_id"),
          BT::OutputPort<std::vector<std::string>>("task_via_area_ids"),
          BT::OutputPort<int>("task_nums"),
          BT::OutputPort<std::vector<std::string>>("via_indexes"),
          BT::OutputPort<nav_msgs::Path>("multi_teaching_path_ros"),

        };
  }

private:
    void taskCmdCallback(const navit_msgs::TaskCommandConstPtr& msg) {
        task_cmd_updated_ = true;
          geometry_msgs::PoseStamped nearby_charge_pose, charge_pose, final_dock_pose;

            std::vector<navit::protocol::map_info::MapPoint> task_points;
            float final_dock_distance;
            getInput<std::vector<navit::protocol::map_info::MapPoint>>("task_points", task_points);
            getInput<float>("final_dock_distance", final_dock_distance);
            bool find_nearby_charge_pose = false;
            bool find_charge_pose = false;
            for(int i = 0; i < task_points.size(); i++) {
                if (task_points[i].type() == navit::protocol::map_info::MapPoint::POINT_TYPE_NEARBY_CHARGE_PILE_STATION) {
                    ROS_WARN("We find nearby charge pose. %f, %f", task_points[i].point().x(), task_points[i].point().y());
                    nearby_charge_pose.header.frame_id = "map";
                    nearby_charge_pose.header.stamp = ros::Time::now();
                    nearby_charge_pose.pose.position.x = task_points[i].point().x();
                    nearby_charge_pose.pose.position.y = task_points[i].point().y();
                    nearby_charge_pose.pose.position.z = 0;

                    double yaw = task_points[i].point().rz();

                    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);
                    nearby_charge_pose.pose.orientation = orientation;
                    find_nearby_charge_pose = true;
                } else if (task_points[i].type() == navit::protocol::map_info::MapPoint::POINT_TYPE_CHARGE_PILE_STATION) {
                    ROS_WARN("We find charge pose in map. %f, %f", task_points[i].point().x(), task_points[i].point().y());
                 
                    // do transform from map to odom 
                    // geometry_msgs::TransformStamped trans;
                    // geometry_msgs::PoseStamped charge_pose_map;
                    
                    // try {
                    //     if (!tf_buffer_->canTransform("map", "odom", ros::Time(0))) {
                    //         ROS_WARN("Waiting for transform");
                    //         return;
                    //     }

                    //     trans = tf_buffer_->lookupTransform("map", "odom", ros::Time(0)); // Use the latest available transform
                    // } catch (tf2::TransformException &ex) {
                    //     ROS_WARN("%s",ex.what());
                    //     return;
                    // }
                    charge_pose.header.frame_id = "map";
                    charge_pose.pose.position.x = task_points[i].point().x();
                    charge_pose.pose.position.y = task_points[i].point().y();
                    charge_pose.pose.position.z = 0;
                    //TODO(CZK) : set orientation, now just for test.
                    double yaw = task_points[i].point().rz();

                    geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);
                    charge_pose.pose.orientation = orientation;
                    // try {
                    //     tf2::doTransform(charge_pose_map, charge_pose , trans); // Perform the transformation
                    // } catch (tf2::TransformException &ex) {
                    //     ROS_WARN("Failed to do transform: %s", ex.what());
                    //     // Handle the exception appropriately
                    // }
                    
                    ROS_WARN("We find charge pose. %f, %f", charge_pose.pose.position.x, charge_pose.pose.position.y);
                    
                    final_dock_pose.header.frame_id = "base_link";
                    final_dock_pose.header.stamp = ros::Time::now();

                    final_dock_pose.pose.position.x = final_dock_distance;
                    find_charge_pose = true;
                }
            }
            // set task_pose_ros_
            if (find_nearby_charge_pose && find_charge_pose) {
              setOutput<geometry_msgs::PoseStamped>("nearby_charge_pose_ros", nearby_charge_pose);
              setOutput<geometry_msgs::PoseStamped>("charge_pose_ros", charge_pose);
              setOutput<geometry_msgs::PoseStamped>("final_dock_pose_ros", final_dock_pose);
            } else {
              ROS_ERROR("We cannot find both nearby charge pose and charge pose.");
            }
        if (msg->cmd == navit_msgs::TaskCommand::CMD_POLYGON) {
            ROS_INFO("Task command service polygon");
            std::vector<navit::protocol::map_info::MapArea> task_polygons;
            getInput<std::vector<navit::protocol::map_info::MapArea>>("task_polygons", task_polygons);
            for (int i = 0; i < task_polygons.size(); i++) {
                if (msg->seq[0] == task_polygons[i].name()) {
                    task_polygon_ = task_polygons[i];
                }
            }
            task_cmd_ = "polygon";
        } else if (msg->cmd == navit_msgs::TaskCommand::CMD_PATH) {
            ROS_INFO("Task command service path");
            std::vector<navit::protocol::map_info::MapLine> task_paths;
            getInput<std::vector<navit::protocol::map_info::MapLine>>("task_paths", task_paths);
            task_path_ros_.header.frame_id = "map";
            task_path_ros_.header.stamp = ros::Time::now();
            task_path_ros_.poses.clear();
            for(int i = 0; i < task_paths.size(); i++) {
                if (msg->seq[0] == task_paths[i].name()) {
                    task_path_ = task_paths[i];
                }
            }

            for (int i = 0; i < task_path_.path().size(); i++) {
              geometry_msgs::PoseStamped pose;
              pose.header.frame_id = "map";
              pose.pose.position.x = task_path_.path()[i].x();
              pose.pose.position.y = task_path_.path()[i].y();
              pose.pose.position.z = 0;
              
              pose.pose.orientation.x = 0;
              pose.pose.orientation.y = 0;
              pose.pose.orientation.z = 0;
              pose.pose.orientation.w = 1;
              task_path_ros_.poses.push_back(pose);
            }

            navit_common::checkPath(task_path_ros_);

            
            task_cmd_ = "path";
        } else if (msg->cmd == navit_msgs::TaskCommand::CMD_POINT) {
            ROS_INFO("Task command service point");
            std::vector<navit::protocol::map_info::MapPoint> task_points;
            getInput<std::vector<navit::protocol::map_info::MapPoint>>("task_points", task_points);

            for(int i = 0; i < task_points.size(); i++) {
                if (msg->seq[0] == task_points[i].name()) {
                    task_point_ = task_points[i];
                }
            }
             // set task_pose_ros_
            task_pose_ros_.header.frame_id = "map";
            task_pose_ros_.header.stamp = ros::Time::now();

            task_pose_ros_.pose.position.x = task_point_.point().x();
            task_pose_ros_.pose.position.y = task_point_.point().y();
            task_pose_ros_.pose.position.z = 0;

            //欧拉角转四元数
            double yaw = task_point_.point().rz();
            geometry_msgs::Quaternion orientation = tf::createQuaternionMsgFromYaw(yaw);
            task_pose_ros_.pose.orientation = orientation;
            //set task_point_

            task_cmd_ = "point";
        } else if (msg->cmd == navit_msgs::TaskCommand::CMD_CHARGE || msg->cmd == navit_msgs::TaskCommand::CMD_MuiltPolygons_WITH_AUTODOCK) {
            ROS_INFO("Task command service charge");
          
        
            task_cmd_ = "charge";
        } else if (msg->cmd == navit_msgs::TaskCommand::CMD_ABORT_CHARGE) {
            task_cmd_ = "undock";
        }
        
        if (msg->cmd == navit_msgs::TaskCommand::CMD_MuiltPolygons_WITH_AUTODOCK) {
            ROS_INFO("Task command service muilt polygons");
            std::vector<navit::protocol::map_info::MapArea> task_polygons;
            std::vector<std::string> task_polygon_ids, task_via_area_ids;
            std::string task_goal_area_id;
            task_polygon_ids = msg->seq;
            setOutput<std::vector<std::string>>("task_polygon_ids", task_polygon_ids);

            for (int i = 0; i < task_polygon_ids.size() - 1; i++) {
                task_via_area_ids.push_back(task_polygon_ids[i]);
            }
            
            task_goal_area_id = task_polygon_ids[task_polygon_ids.size() - 1];
            
            setOutput<std::string>("task_goal_area_id", task_goal_area_id);
            setOutput<std::vector<std::string>>("task_via_area_ids", task_via_area_ids);
            setOutput<std::vector<std::string>>("task_polygon_ids", task_polygon_ids);
            task_cmd_ = "muilt_polygons";
        } else {
            ROS_INFO("Task command service unknown");
        }
        setOutput<int>("repeat_times", msg->repeat_times);
        setOutput("task_cmd", task_cmd_);
        setOutput<int>("task_nums", 0);
        std::vector<std::string> via_indexes;
        nav_msgs::Path multi_teaching_path_ros;
        setOutput<std::vector<std::string>>("via_indexes", via_indexes);
        setOutput<nav_msgs::Path>("multi_teaching_path_ros", multi_teaching_path_ros);
    }

    void halt() override
    {
      ROS_INFO("Task command service halt");
    }

    BT::NodeStatus tick() override {
        if (task_cmd_updated_) {
            setOutput<bool>("task_cmd_updated", true);
            setOutput<navit::protocol::map_info::MapArea>("task_polygon", task_polygon_);
            setOutput<navit::protocol::map_info::MapLine>("task_path", task_path_);
            setOutput<navit::protocol::map_info::MapPoint>("task_point", task_point_);
            setOutput<nav_msgs::Path>("task_path_ros", task_path_ros_);
            setOutput<geometry_msgs::PoseStamped>("task_pose_ros", task_pose_ros_);
            task_cmd_updated_ = false;
            return BT::NodeStatus::SUCCESS;
        }
        return BT::NodeStatus::RUNNING;
    };
    ros::NodeHandle nh_;
    ros::Subscriber task_com_sub_;
    bool task_cmd_updated_ = false;
    std::string task_cmd_;
    navit::protocol::map_info::MapArea task_polygon_;
    navit::protocol::map_info::MapLine task_path_;
    navit::protocol::map_info::MapPoint task_point_;
    nav_msgs::Path task_path_ros_;
    geometry_msgs::PoseStamped task_pose_ros_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    tf::TransformListener listener;

};
}  // namespace navit_bt_nodes

#endif // NAVIT_BT_NODES__PLUGINS__ACTION__TASK_COMMAND_SERVICE_ACTION_HPP_