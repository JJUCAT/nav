#ifndef CONTROLLER_SERVER_HELPER_H
#define CONTROLLER_SERVER_HELPER_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <nav_msgs/Path.h>
#include <navit_costmap/costmap_2d_ros.h>
#include <navit_costmap/footprint.h>
#include <navit_costmap/footprint_collision_checker.h>
#include <navit_common/geometry_algorithms.h>
#include <navit_common/log.h>
#include <navit_common/path_handle.h>

namespace navit_controller {
class ControllerServerHelper {
    public:
        ControllerServerHelper() {};
        ~ControllerServerHelper() {};

        double getPathLength(const nav_msgs::Path& path);
        bool getCompletionPercentage(const tf2_ros::Buffer& tf,
                                       const geometry_msgs::PoseStamped& global_pose,
                                       const nav_msgs::Path& original_path,
                                       std::vector<geometry_msgs::PoseStamped>& global_plan,
                                       double& completion_percentage);

        bool taskPath(const geometry_msgs::PoseStamped robot_pose,
                      nav_msgs::Path& cleaned_paths,
                      nav_msgs::Path& uncleaned_paths);
        bool init(const std::shared_ptr<navit_costmap::Costmap2DROS> costmap);

        bool setGlobalPath(const nav_msgs::Path& path) {
            whole_path_.clear();
            for (const auto& pose_stamped : path.poses) {
                whole_path_.push_back(pose_stamped);
            }
            return true;
        }
        /* @brief extract sub path from global path
         * @param start_index: start index of global path
         * @param end_index: end index of global path
         * @param whole_path: global path
         * @param sub_path: sub path
         * @return true if extract successfully, otherwise false
        */
        bool extractFromGlobalPath(int start_index, int end_index, nav_msgs::Path& whole_path, nav_msgs::Path& sub_path);

        /* @brief find coordinate transform between target frame and source frame
         * @param target_frame: target frame
         * @param source_frame: source frame
         * @param transformStamped: coordinate transform
         * @return true if find successfully, otherwise false
        */
        bool doTransform(const nav_msgs::Path& path_in, const geometry_msgs::TransformStamped& transform_stamped, nav_msgs::Path& path_out, const std::string& frame_id);


        bool findNearestIndexOnPath(const geometry_msgs::PoseStamped& pose, const nav_msgs::Path& path, uint16_t& index) {
            if (path.poses.size() == 0) {
                NAVIT_ROS_ERROR_STREAM("Path is empty.");
                return false;
            }
            double min_distance = std::numeric_limits<double>::max();
            for (int i = 0; i < path.poses.size(); i++) {
                double distance;
                // double distance = navit_common::geometry::distanceBetweenPoses(pose, path.poses[i]);
                distance = sqrt(pow(pose.pose.position.x - path.poses[i].pose.position.x, 2) +
                                pow(pose.pose.position.y - path.poses[i].pose.position.y, 2));
                if (distance < min_distance) {
                    min_distance = distance;
                    index = i;
                }
            }
            return true;
        }

        void visualizePath(const nav_msgs::Path& path, ros::Publisher& publisher) {
            publisher.publish(path);
        }
        bool resamplePath(const nav_msgs::Path& input_path, double desired_spacing, nav_msgs::Path& output_path) {
            output_path = navit_common::resamplePath(input_path, desired_spacing);
            if (navit_common::checkPath(output_path)) {
                return true;
            } else {
                return false;
            }
        }
    private:
        std::shared_ptr<navit_costmap::Costmap2DROS> navit_costmap_ptr_;
        std::list<geometry_msgs::PoseStamped> whole_path_;
};

bool ControllerServerHelper::init(const std::shared_ptr<navit_costmap::Costmap2DROS> costmap) {
    navit_costmap_ptr_ = costmap;
    return true;
}

double ControllerServerHelper::getPathLength(const nav_msgs::Path& path) {
    if (path.poses.size() == 0 ) return 0.0;

    double diff_x, diff_y, distance_to_goal = 0.0;
    for(auto it = path.poses.begin(); it != path.poses.end()-1; it++) {
        diff_x = it->pose.position.x - (it + 1)->pose.position.x;
        diff_y = it->pose.position.y - (it + 1)->pose.position.y;
        distance_to_goal += std::sqrt(diff_x * diff_x + diff_y * diff_y);
    }
    return distance_to_goal;
}

bool ControllerServerHelper::getCompletionPercentage(const tf2_ros::Buffer& tf,
                                                     const geometry_msgs::PoseStamped& global_pose,
                                                     const nav_msgs::Path& original_path,
                                                     std::vector<geometry_msgs::PoseStamped>& global_plan,
                                                     double& completion_percentage)
{
    if (global_plan.empty()) return true;
    try
    {
        // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
        geometry_msgs::TransformStamped global_to_plan_transform = tf.lookupTransform(global_plan.front().header.frame_id, global_pose.header.frame_id, ros::Time(0));
        geometry_msgs::PoseStamped robot;
        tf2::doTransform(global_pose, robot, global_to_plan_transform);

        double dist_thresh_sq = 0.1;

        // iterate plan until a pose close the robot is found
        std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
        std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
        while (it != global_plan.end())
        {
        double dx = robot.pose.position.x - it->pose.position.x;
        double dy = robot.pose.position.y - it->pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < dist_thresh_sq)
        {
            erase_end = it;
            break;
        }
        ++it;
        }
        if (erase_end == global_plan.end())
        return false;

        if (erase_end != global_plan.begin())
        global_plan.erase(global_plan.begin(), erase_end);

        completion_percentage = ( 1.0 - (double) global_plan.size() / (double) original_path.poses.size() )* 100.0;

    //   auto distance_to_goal = distance_to_goal_ * (1.0 - completion_percentage / 100.0);
    }
    catch (const tf2::TransformException& ex)
    {
        ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
        return false;
    }
    return true;
}

bool ControllerServerHelper::taskPath(const geometry_msgs::PoseStamped robot_pose,
                                      nav_msgs::Path& cleaned_paths,
                                      nav_msgs::Path& uncleaned_paths) {
    double x, y, roll, pitch, yaw;
    x = robot_pose.pose.position.x;
    y = robot_pose.pose.position.y;

    tf::Matrix3x3(tf::Quaternion(robot_pose.pose.orientation.x,
                                 robot_pose.pose.orientation.y,
                                 robot_pose.pose.orientation.z,
                                 robot_pose.pose.orientation.w)).getRPY(roll, pitch, yaw);

    navit_costmap::Footprint footprint_spec = navit_costmap_ptr_->getRobotFootprint();
    std::vector<geometry_msgs::Point> oriented_footprint;

    navit_costmap::transformFootprint(x, y, yaw, footprint_spec, oriented_footprint);

    Polygon boost_footprint;
    for(const auto& point : oriented_footprint) {
        boost::geometry::append(boost_footprint, Point(point.x, point.y));
    }
    boost::geometry::append(boost_footprint, Point(oriented_footprint.front().x, oriented_footprint.front().y));

    auto it = whole_path_.begin();
    while (it != whole_path_.end()) {
        Point point;
        point.x(it->pose.position.x);
        point.y(it->pose.position.y);

        if (navit_common::geometry::isPoseInPolygon(point, boost_footprint)) {
            it = whole_path_.erase(it);
        } else {
            ++it;
        }
    }

    uncleaned_paths.header.frame_id = "map";
    for (const auto& pose_stamped : whole_path_) {
        uncleaned_paths.poses.push_back(pose_stamped);
    }
    return true;
}

bool ControllerServerHelper::extractFromGlobalPath(int start_index, int end_index, nav_msgs::Path& whole_path, nav_msgs::Path& sub_path) {
    if (start_index < 0 || end_index < 0 || start_index > end_index) return false;
    if (end_index > whole_path.poses.size()) {
        end_index = whole_path.poses.size();
    }
    // TODO(czk):vector 去除删除首元素会略有影响，list可以考虑，但是list又不能直接setplan，后续要优化下这个地方
    sub_path.poses.clear();
    sub_path.header.frame_id = whole_path.header.frame_id;

    for (int i = start_index; i < end_index; i++) {
        sub_path.poses.push_back(whole_path.poses[i]);
    }
    return true;
}

bool ControllerServerHelper::doTransform(const nav_msgs::Path& path_in, const geometry_msgs::TransformStamped& transform_stamped, nav_msgs::Path& path_out, const std::string& frame_id) {
    if (path_in.poses.size() == 0) {
        NAVIT_ROS_ERROR_STREAM("Path is empty.");
        return false;
    }

    path_out.header.frame_id = frame_id;

    for (const auto& pose_stamped : path_in.poses) {
        geometry_msgs::PoseStamped pose_stamped_out;
        pose_stamped_out.header.frame_id = frame_id;
        tf2::doTransform(pose_stamped, pose_stamped_out, transform_stamped);
        path_out.poses.push_back(pose_stamped_out);
    }
    return true;
}
} //namespace navit_controller
#endif


