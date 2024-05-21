#ifndef RECOVERY_H
#define RECOVERY_H

#include <tf2_ros/buffer.h>
#include <navit_collision_checker/collision_checker.h>
#include <navit_costmap/costmap_2d_ros.h>

namespace navit_core {

    /**
     *  主要参考nav2中的recovery，后续如果觉得订阅costmap过于吃资源的话，再考虑直接在recovery中维护一个costmap
     *  目前要求在controller costmap（local costmap）设置中，需要将
     *  always_send_full_costmap设置为true，以便于collision checker订阅
     */
    class Recovery
    {
        public:
            using Ptr = boost::shared_ptr<Recovery>;

            virtual ~Recovery(){}

            virtual void initialize(const std::string& name,
                                    const std::shared_ptr<tf2_ros::Buffer>& tf,
                                    const std::shared_ptr<navit_collision_checker::CollisionChecker>& collision_checker) = 0;

            virtual void SetExtraCostmap(const std::shared_ptr<navit_costmap::Costmap2DROS>& map) {
              std::cout << "!!! set empty extra costmap !!!" << std::endl;
            }
    };

}
#endif
