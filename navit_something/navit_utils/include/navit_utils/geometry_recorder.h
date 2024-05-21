// GeometryRecorder.h

#ifndef GEOMETRY_RECORDER_H
#define GEOMETRY_RECORDER_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <navit_msgs/StatusGeometryPoints.h>
#include <navit_msgs/CmdRecordGeometry.h>
#include <navit_common/geometry_algorithms.h>
class GeometryRecorder {
public:
    GeometryRecorder();

private:
    void timer1(const ros::TimerEvent &event);
    bool commandCallback(navit_msgs::CmdRecordGeometry::Request &req,
                         navit_msgs::CmdRecordGeometry::Response &res);
    void mappingPathCallback(const nav_msgs::Path::ConstPtr &path);
    void publishStatus();
    void recordGeometry();
    bool expandArea(const float expand_dis);
    ros::NodeHandle nh_ = ros::NodeHandle("~");
    ros::ServiceServer command_service_;
    ros::Publisher status_pub_, record_pub_;
    ros::Subscriber mapping_path_sub_;
    tf::TransformListener listener_;
    navit_msgs::StatusGeometryPoints status_msg_;
    bool is_recording_;
    uint16_t recording_type_;
    ros::Timer timer_;
    //parameters for path or polygon
    bool is_smooth_ = false;
    float smooth_step_ = 0.1f;
    float area_expand_dis_ = 0.0f;
    std::string golbal_frame_id_ = "map";
    uint16_t mode_ = 1;
};


#endif // GEOMETRY_RECORDER_H