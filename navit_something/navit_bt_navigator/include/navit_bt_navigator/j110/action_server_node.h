#pragma once
#include <navit_bt_navigator/j110/base_behavior_tree_manager.hpp>
#include <navit_bt_navigator/j110/base_behavior_tree_node.hpp>

class BehaviorNodeActionServer {
public:
    template <typename ActionType>
    void goalReceived(typename ActionType::Goal::ConstSharedPtr goal);

private:
    BehaviorTreeManager behaviorTreeManager;

    template <typename ActionType>
    std::unique_ptr<AbstractBehaviorTreeNode> createBehaviorNodeForGoal(typename ActionType::Goal::ConstSharedPtr goal);
};
