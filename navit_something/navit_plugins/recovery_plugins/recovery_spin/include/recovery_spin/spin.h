#ifndef RECOVERY_spin_H
#define RECOVERY_spin_H

#include <navit_recovery/recovery_action.h>
#include <navit_msgs/SpinAction.h>

using namespace navit_recovery;

namespace recovery_spin {
using ActionT = navit_msgs::SpinAction;
using ActionGoalT = navit_msgs::SpinGoal;
using ActionFeedbackT = navit_msgs::SpinFeedback;
using ActionResultT = navit_msgs::SpinResult;

    class RecoverySpin : public RecoveryAction<ActionT, ActionGoalT, ActionResultT>
    {
        public:
            RecoverySpin(){}
            ~RecoverySpin(){}

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

            float spin_distance_;
            float estimated_time_; 
    };
}
#endif
