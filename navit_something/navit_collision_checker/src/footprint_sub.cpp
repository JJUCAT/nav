#include <navit_collision_checker/footprint_sub.h>

using namespace navit_costmap;

namespace navit_collision_checker {

    FootprintSub::FootprintSub(const ros::NodeHandle& nh,
                               const std::string& topic_name):
        nh_(nh),
        topic_name_(topic_name),
        footprint_(nullptr),
        footprint_received_(false)
    {
        sub_ = nh_.subscribe(topic_name_, 1, &FootprintSub::footprintCallback, this);
        ROS_INFO("footprint sub is initalized");
    }

    bool FootprintSub::getFootprint(std::vector<geometry_msgs::Point>& footprint)
    {
        // 原本是构造时候等待话题更新，这里改为使用时候等待话题更新
        ros::Rate r(10.0);
        while (!footprint_received_)
        {
            ROS_INFO_THROTTLE(1.0, "wait for topic [%s]", topic_name_.c_str());
            ros::spinOnce();
            r.sleep();
        }

        footprint = toPointVector( footprint_->polygon);
        return true;
    }

    void FootprintSub::footprintCallback(const geometry_msgs::PolygonStamped::ConstPtr& msg)
    {
        std::lock_guard<std::mutex> lock(footprint_lock_);
        footprint_ = msg;
        if(!footprint_received_)
            footprint_received_ = true;
    }

}
