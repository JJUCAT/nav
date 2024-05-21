#include <navit_utils/geometry_recorder.h>

GeometryRecorder::GeometryRecorder() : is_recording_(false), recording_type_(0) {

    //nh_.param("points_pub_frequency", monitor_frequency_, 1.0f);
    //TODO(czk): 增加打结处理，多边形异常错误返回
    nh_.param("is_smooth", is_smooth_, true);
    nh_.param("smooth_step", smooth_step_, 0.1f);
    nh_.param("area_expand_dis", area_expand_dis_, 0.0f); // > 0 means expand the area, < 0 means shrink the area

    command_service_ = nh_.advertiseService("/navit/cmd_record_geometry", &GeometryRecorder::commandCallback, this);
    status_pub_ = nh_.advertise<navit_msgs::StatusGeometryPoints>("/status_geometry_points", 10);
    record_pub_ = nh_.advertise<visualization_msgs::Marker>("/record_geometry", 10);
    mapping_path_sub_ = nh_.subscribe("/ms_lams/mapping/path", 10, &GeometryRecorder::mappingPathCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &GeometryRecorder::timer1, this);
}

void GeometryRecorder::timer1(const ros::TimerEvent &event) {
    recordGeometry();
    publishStatus();
}

bool GeometryRecorder::commandCallback(navit_msgs::CmdRecordGeometry::Request &req,
                                       navit_msgs::CmdRecordGeometry::Response &res) {

    ROS_INFO("Navit: cmd record geometry callback...");
    switch (req.cmd) {
        case navit_msgs::CmdRecordGeometryRequest::CMD_CLEAR:
            ROS_INFO("Clearing...");
            status_msg_.poses_stamped.clear();
            ROS_INFO("Clear Successfully!");
            // only clear the points, not the status
            // is_recording_ = false;
            break;
        case navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_POLYGON:
            recording_type_ = navit_msgs::StatusGeometryPoints::STATUS_RECORDING_POLYGON;
            is_recording_ = true;
            break;
        case navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_PATH:
            recording_type_ = navit_msgs::StatusGeometryPoints::STATUS_RECORDING_PATH;
            is_recording_ = true;
            break;
        case navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_POINT:
            recording_type_ = navit_msgs::StatusGeometryPoints::STATUS_RECORDING_POINT;
            is_recording_ = true;
            break;
        case navit_msgs::CmdRecordGeometryRequest::CMD_FINISH_RECORD:
            is_recording_ = false;
            publishStatus();
            if (recording_type_ == navit_msgs::CmdRecordGeometryRequest::CMD_START_RECORD_POLYGON) {
                if(!expandArea(area_expand_dis_)) {
                    ROS_ERROR("Expand area failed. Please check the polygon.");
                }
            }
            ROS_INFO("Expand area Successfully!");
            res.poses_stamped = status_msg_.poses_stamped;
            status_msg_.poses_stamped.clear();
            break;

        default:
            res.success = false;
            return false;
    }

    golbal_frame_id_ = req.frame_id.empty() ? "map" : req.frame_id;
    mode_ = req.mode;
    status_msg_.status = recording_type_;
    res.success = true;
    return true;
}
bool GeometryRecorder::expandArea(const float expand_dis) {
    if (status_msg_.poses_stamped.size() < 3) {
        ROS_ERROR("The points of polygon is less than 3.");
        return false;
    }
    Polygon polygon, expand_polygon, simplified_polygon;
    for (auto &pose_stamped : status_msg_.poses_stamped) {
        boost::geometry::append(polygon, Point(pose_stamped.pose.position.x, pose_stamped.pose.position.y));
    }
    boost::geometry::append(polygon, Point(status_msg_.poses_stamped[0].pose.position.x, status_msg_.poses_stamped[0].pose.position.y));
    boost::geometry::correct(polygon);
    boost::geometry::simplify(polygon, simplified_polygon, 0.15);

    expand_polygon = navit_common::geometry::adjustPolygonBoundary(simplified_polygon, expand_dis);

    if (expand_polygon.outer().size() == 0) {
        return false;
    }
    status_msg_.poses_stamped.clear();
    status_msg_.poses_stamped.resize(expand_polygon.outer().size());
    // update status_msg_.poses_stamped position with expand_polygon, orientation remains the same

    for (int i = 0; i < expand_polygon.outer().size(); ++i) {
        status_msg_.poses_stamped[i].pose.position.x = expand_polygon.outer()[i].x();
        status_msg_.poses_stamped[i].pose.position.y = expand_polygon.outer()[i].y();
        status_msg_.poses_stamped[i].pose.orientation.w = 1.0;
    }
    return true;
}

void GeometryRecorder::recordGeometry() {
    if (is_recording_ && mode_ == 1) {
        ROS_INFO("Recording...");
        try {
            tf::StampedTransform transform;
            listener_.lookupTransform(golbal_frame_id_, "base_link", ros::Time(0), transform);

            geometry_msgs::PoseStamped pose_stamped;
            pose_stamped.header.frame_id = golbal_frame_id_;
            pose_stamped.header.stamp = ros::Time::now();
            pose_stamped.pose.position.x = transform.getOrigin().x();
            pose_stamped.pose.position.y = transform.getOrigin().y();
            pose_stamped.pose.position.z = transform.getOrigin().z();

            pose_stamped.pose.orientation.x = transform.getRotation().x();
            pose_stamped.pose.orientation.y = transform.getRotation().y();
            pose_stamped.pose.orientation.z = transform.getRotation().z();
            pose_stamped.pose.orientation.w = transform.getRotation().w();

            LineString line_string;
            Point p(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
            if (status_msg_.poses_stamped.size() == 0) {
                status_msg_.poses_stamped.push_back(pose_stamped);
                boost::geometry::append(line_string, p);
            }
            // TODO(czk): use this method to smooth the path just for simple test.
            if (is_smooth_ && status_msg_.poses_stamped.size() > 0) {
                Point last_p(status_msg_.poses_stamped.back().pose.position.x, status_msg_.poses_stamped.back().pose.position.y);
                float dist = navit_common::geometry::distance(p, last_p);
                if (dist < smooth_step_) {
                    return;
                }
            }
            status_msg_.poses_stamped.push_back(pose_stamped);
        }
        catch (tf::TransformException &ex) {
            ROS_WARN("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
    //TODO(czk): if type is path, handle the path
    //TODO(czk): if type is polygon, handle the polygon
}

void GeometryRecorder::publishStatus() {
    if (status_msg_.poses_stamped.size() == 0) {
        return;
    }

    status_pub_.publish(status_msg_);
    visualization_msgs::Marker marker;
    marker.header.frame_id = golbal_frame_id_;
    marker.header.stamp = ros::Time::now();
    marker.ns = "navit";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    marker.points.resize(status_msg_.poses_stamped.size());
    for (int i = 0; i < status_msg_.poses_stamped.size(); ++i) {
        marker.points[i] = status_msg_.poses_stamped[i].pose.position;
    }

    record_pub_.publish(marker);
}

void GeometryRecorder::mappingPathCallback(const nav_msgs::Path::ConstPtr &path) {
    if (mode_ == 0 && is_recording_) {
        status_msg_.poses_stamped.clear();
        for (const auto &pose_stamped : path->poses) {
            status_msg_.poses_stamped.push_back(pose_stamped);
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "geometry_recorder");
    GeometryRecorder recorder;
    ros::spin();
    return 0;
}
