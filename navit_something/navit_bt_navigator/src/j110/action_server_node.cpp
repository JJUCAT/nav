#include <navit_bt_navigator/j110/action_server_node.hpp>
#include "PointToPointBehaviorNode.h"

template <typename ActionType>
void BehaviorNodeActionServer::goalReceived(typename ActionType::Goal::ConstSharedPtr goal) {
    auto newBehavior = createBehaviorNodeForGoal<ActionType>(goal);
    behaviorTreeManager.switchBehavior(std::move(newBehavior));
}

template <typename ActionType>
std::unique_ptr<AbstractBehaviorTreeNode> BehaviorNodeActionServer::createBehaviorNodeForGoal(typename ActionType::Goal::ConstSharedPtr goal) {
    if constexpr (std::is_same<ActionType, YourActionType1>::value) {
        return std::make_unique<PointToPointBehaviorNode>();
    } else if constexpr (std::is_same<ActionType, YourActionType2>::value) {
        // Add more conditions for other action types
        return std::make_unique<AnotherBehaviorNode>();
    } else {
        return std::make_unique<DefaultBehaviorNode>();
    }
}