#ifndef INTEGRATED_NAVIGATION_TF_BROADCASTER_HPP
#define INTEGRATED_NAVIGATION_TF_BROADCASTER_HPP

#include <string>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>

namespace integrated_navigation {
    class TFBroadCaster {
    public:
        TFBroadCaster(std::string frame_id, std::string child_frame_id);
        TFBroadCaster() = default;
        void SendTransform(Eigen::Matrix4d pose, double time);
    protected:
        tf::StampedTransform transform_;
        tf::TransformBroadcaster broadcaster_;
    };
}
#endif //INTEGRATED_NAVIGATION_TF_BROADCASTER_HPP