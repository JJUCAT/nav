#pragma once


class AbstractBehaviorTreeNode {
public:
    virtual ~AbstractBehaviorTreeNode() {}
    virtual void loadBehaviorTree() = 0;
    virtual void unloadBehaviorTree() = 0;
    virtual void execute() = 0;
};