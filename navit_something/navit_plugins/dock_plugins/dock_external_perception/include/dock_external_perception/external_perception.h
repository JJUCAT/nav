#ifndef EXTERNAL_PERCEPTION_H
#define EXTERNAL_PERCEPTION_H

#include <navit_auto_dock/plugins/approach_dock_perception.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/PoseStamped.h>

namespace dock_external_perception {
    class ExternalPerception : public navit_auto_dock::plugins::ApproachDockPerception
    {
        public:
            ExternalPerception(){}

            void initialize(const std::string name,
                            const std::shared_ptr<tf2_ros::Buffer>& tf) override;

            bool start(const geometry_msgs::PoseStamped& initial_dock_pose) override
            {
                ROS_WARN("external perception used! No initial guess is needed!");
                return true;
            }

            bool stop() override {return true;}

            bool getPose(geometry_msgs::PoseStamped& pose) override
            {
                pose = detected_dock_pose_;
                return true;
            }

        private:
            void dockDetectionCallback(const geometry_msgs::PoseStampedConstPtr& msg)
            {
               detected_dock_pose_.header = msg->header;
               detected_dock_pose_.pose = msg->pose; 
            }

            geometry_msgs::PoseStamped detected_dock_pose_;
            ros::Subscriber pose_sub_;

            struct config
            {
                std::string detection_topic_name="detected_dock_pose";
            } config_;
    };
};
#endif

