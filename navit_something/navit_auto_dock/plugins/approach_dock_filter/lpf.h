#ifndef APPROACH_DOCK_FILTER_LPF_H
#define APPROACH_DOCK_FILTER_LPF_H

#include <navit_auto_dock/plugins/approach_dock_filter.h>
#include "linear_pose_filter_2d.h"

namespace navit_auto_dock{
namespace plugins{

    class Lpf : public ApproachDockFilter 
    {
        public:
            void initialize(const std::string& name,
                            const std::shared_ptr<tf2_ros::Buffer>& ) override;

            void reset() override;

            void update(geometry_msgs::PoseStamped& dock_pose,
                        const geometry_msgs::Twist&) override;

        private:
            std::shared_ptr<LinearPoseFilter2D> linear_pose_filter_;
            bool dock_found_;
            
    };

}
}
#endif
