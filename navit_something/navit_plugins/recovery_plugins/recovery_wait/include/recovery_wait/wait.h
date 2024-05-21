#ifndef RECOVERY_WAIT_H
#define RECOVERY_WAIT_H

#include <navit_recovery/recovery_action.h>
#include <navit_msgs/WaitAction.h>

using namespace navit_recovery;

namespace recovery_wait {
using ActionType = navit_msgs::WaitAction;
using ActionGoal = navit_msgs::WaitGoal;
using ActionFeedback = navit_msgs::WaitFeedback;
using ActionResult = navit_msgs::WaitResult;

    class RecoveryWait : public RecoveryAction<ActionType, ActionGoal, ActionResult>
    {
        public:
            RecoveryWait();

            ~RecoveryWait();

            Status onRun(const ActionGoal::ConstPtr& action_goal) override;

            Status onCycleUpdate() override;  

            void loadParams(const std::string& name) override;

        protected:
            ActionFeedback feedback_;
            ActionResult result_;

            ros::Time end_;
    };

}
#endif
