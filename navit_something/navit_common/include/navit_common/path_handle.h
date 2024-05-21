#ifndef NAVIT_COMMON__PATH_HANDLE_H
#define NAVIT_COMMON__PATH_HANDLE_H

#include <ros/ros.h>
#include <tf/tf.h>

namespace navit_common {

bool checkPath(nav_msgs::Path& path)
{
  if (path.poses.size() < 2)
  {
    ROS_ERROR("Path size is too less, size is %d.", static_cast<int>(path.poses.size()));
    return false;
  }
  else
  {
    path.header.frame_id = "map";

    for (int i = 0; i < path.poses.size(); i++)
    {
      path.poses[i].header.frame_id = "map";
      float yaw = i < path.poses.size() - 1 ? atan2(path.poses[i + 1].pose.position.y - path.poses[i].pose.position.y,
                                                    path.poses[i + 1].pose.position.x - path.poses[i].pose.position.x) :
                                              atan2(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y,
                                                    path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x);
      geometry_msgs::Quaternion qua = tf::createQuaternionMsgFromYaw(yaw);
      path.poses[i].pose.orientation = qua;
    }
  }
  return true;
}

double distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2) {
return sqrt(pow(p2.pose.position.x - p1.pose.position.x, 2) +
            pow(p2.pose.position.y - p1.pose.position.y, 2) +
            pow(p2.pose.position.z - p1.pose.position.z, 2));
}

geometry_msgs::PoseStamped interpolate(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2, double ratio) {
    geometry_msgs::PoseStamped interpolated_pose;

    interpolated_pose.pose.position.x = (1 - ratio) * p1.pose.position.x + ratio * p2.pose.position.x;
    interpolated_pose.pose.position.y = (1 - ratio) * p1.pose.position.y + ratio * p2.pose.position.y;
    interpolated_pose.pose.position.z = p2.pose.position.z;

    interpolated_pose.pose.orientation = p2.pose.orientation;
    return interpolated_pose;
}

nav_msgs::Path resamplePath(const nav_msgs::Path& input_path, double desired_spacing, std::string frame_id = "map") {
    nav_msgs::Path output_path;
    output_path.header.frame_id = frame_id;

    if (input_path.poses.size() < 2 || desired_spacing < 0.0) {
      ROS_ERROR("Input size is less than 2");
      return output_path;
    }

    output_path.poses.push_back(input_path.poses[0]);
    double excess = 0.0;

    size_t i = 1;
    geometry_msgs::PoseStamped last_added = input_path.poses[0];

    while (i < input_path.poses.size()) {
        double d = distance(last_added, input_path.poses[i]);

        if (d < desired_spacing) {
            ++i;
        } else if (d + excess < desired_spacing) {
            excess += d;
            ++i;
        } else {
            double ratio = (desired_spacing - excess) / d;
            geometry_msgs::PoseStamped interpolated_pose = interpolate(last_added, input_path.poses[i], ratio);
            interpolated_pose.header.frame_id = frame_id;
            output_path.poses.push_back(interpolated_pose);
            last_added = interpolated_pose;

            if (ratio == 1.0) {
                ++i;
            }
            excess = 0.0;
        }
    }

    return output_path;
}
}
#endif