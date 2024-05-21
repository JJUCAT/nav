#ifndef CATKIN_INTEGRATED_NAVIGATION_POSE_STAMPED_PUBLISHER_H
#define CATKIN_INTEGRATED_NAVIGATION_POSE_STAMPED_PUBLISHER_H

#include <string>

#include <Eigen/Dense>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include "integrated_navigation/filtering/filter_defination.h"
#include "integrated_navigation/utils/utils.h"

namespace integrated_navigation {
    class PoseStampedPublisher {
    public:
        PoseStampedPublisher(ros::NodeHandle& nh,
                          std::string topic_name,
                          std::string frame_id,
                          int buff_size);
        PoseStampedPublisher() = default;

        void Publish(const State state);

    private:
        void PublishData(const State state);

        ros::Publisher publisher_;
        std::string frame_id_;
    };
}
#endif //CATKIN_INTEGRATED_NAVIGATION_POSE_STAMPED_PUBLISHER_H