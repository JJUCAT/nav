#ifndef RECOVERY_BACKUP_H
#define RECOVERY_BACKUP_H

#include <navit_recovery/recovery_action.h>
#include <navit_msgs/BackUpAction.h>

using namespace navit_recovery;

namespace recovery_backup {
using ActionT = navit_msgs::BackUpAction;
using ActionGoalT = navit_msgs::BackUpGoal;
using ActionFeedbackT = navit_msgs::BackUpFeedback;
using ActionResultT = navit_msgs::BackUpResult;

    class RecoveryBackUp : public RecoveryAction<ActionT, ActionGoalT, ActionResultT>
    {
        public:
            RecoveryBackUp(){}
            ~RecoveryBackUp(){}

            Status onRun(const ActionGoalT::ConstPtr& action_goal) override;
            Status onCycleUpdate() override;

            void loadParams(const std::string& name) override;

        protected:
            ActionFeedbackT feedback_;
            ActionResultT result_;

            struct config_params
            {
                float Kp = 0.5;
                float abs_max_speed = 0.2;
                float control_frequency = 10.0;
                std::string cmd_vel_topic = "cmd_vel";
            } config_;

            ros::Publisher cmd_pub_;
            ros::Time end_;
            geometry_msgs::Twist cmd_vel_;

            float backup_distance_;
            float estimated_time_; 
    };
}
#endif
