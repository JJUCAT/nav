#include <navit_collision_checker/costmap_sub.h>

using namespace navit_costmap;

namespace navit_collision_checker {

    CostmapSub::CostmapSub(const ros::NodeHandle& nh,
                           const std::string& topic_name):
       nh_(nh),
       topic_name_(topic_name),
       costmap_msg_(nullptr),
       costmap_(nullptr),
       costmap_received_(false)
    {
        sub_ = nh_.subscribe(topic_name_, 1, &CostmapSub::costmapCallback, this);
        ROS_INFO("costmap sub initialized");
    }

    std::shared_ptr<Costmap2D>
    CostmapSub::getCostmap()
    {
        // 原本是构造时候等待话题更新，这里改为使用时候等待话题更新
        ros::Rate r(10.0);
        while (!costmap_received_)
        {
           ROS_INFO_THROTTLE(1.0, "wait for topic [%s]", topic_name_.c_str());
           ros::spinOnce();
           r.sleep(); 
        }

        toCostmap2D();
        return costmap_;
    }

    void CostmapSub::toCostmap2D()
    {
        if (costmap_ == nullptr) 
        {
            costmap_ = std::make_shared<Costmap2D>(
                costmap_msg_->info.width, 
                costmap_msg_->info.height,
                costmap_msg_->info.resolution, 
                costmap_msg_->info.origin.position.x,
                costmap_msg_->info.origin.position.y);
        } 
        else if (costmap_->getSizeInCellsX() != costmap_msg_->info.width || 
                 costmap_->getSizeInCellsY() != costmap_msg_->info.height ||
                 costmap_->getResolution() != costmap_msg_->info.resolution ||
                 costmap_->getOriginX() != costmap_msg_->info.origin.position.x ||
                 costmap_->getOriginY() != costmap_msg_->info.origin.position.y)
        {
            // Update the size of the costmap
            costmap_->resizeMap(
              costmap_msg_->info.width, 
              costmap_msg_->info.height,
              costmap_msg_->info.resolution,
              costmap_msg_->info.origin.position.x,
              costmap_msg_->info.origin.position.y);
        }

        unsigned char * master_array = costmap_->getCharMap();
        unsigned int index = 0;
        for (unsigned int i = 0; i < costmap_msg_->info.width; ++i) {
          for (unsigned int j = 0; j < costmap_msg_->info.height; ++j) {
            master_array[index] = costmap_msg_->data[index];
            ++index;
          }
        } 
    }

    void CostmapSub::costmapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        ROS_DEBUG("NEW costmap recevied");
        costmap_msg_ = msg;
        if(!costmap_received_)
        {
            costmap_received_ = true;
        }
    }

}
