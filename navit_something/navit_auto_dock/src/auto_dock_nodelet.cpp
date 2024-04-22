#include <navit_auto_dock/approach_dock.h>
#include <navit_auto_dock/final_dock.h>
#include <tf2_ros/transform_listener.h>
#include <nodelet/nodelet.h>

namespace navit_auto_dock
{
    class AutoDockNodelet : public nodelet::Nodelet
    {
        public:
            AutoDockNodelet() {}
            ~AutoDockNodelet()
            {
                if (approach_dock_) approach_dock_.reset();
                if (final_dock_) final_dock_.reset();
            }

        private:
            void onInit() 
            {
                ros::NodeHandle& nh = getNodeHandle();

                auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
                tf2_ros::TransformListener tf_listener(*tf_buffer);

                approach_dock_.reset(new ApproachDock("/auto_dock/approach_dock", tf_buffer, nh));
                final_dock_.reset(new FinalDock("/auto_dock/final_dock", tf_buffer, nh));

                NODELET_INFO("AutoDockNodelet initialized");
            }

            boost::shared_ptr<ApproachDock> approach_dock_;
            boost::shared_ptr<FinalDock> final_dock_;

    };
}// namespace navit_auto_dock
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(navit_auto_dock::AutoDockNodelet, nodelet::Nodelet)
