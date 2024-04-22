#pragma once
#include <memory>

class BehaviorTreeManager {
public:
    BehaviorTreeManager();
    ~BehaviorTreeManager();

    void switchBehavior(std::unique_ptr<AbstractBehaviorTreeNode> newBehavior);
    void executeCurrentBehavior();

private:
    std::unique_ptr<AbstractBehaviorTreeNode> currentBehavior;
};