#ifndef CEDGE_H
#define CEDGE_H

#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <string>
#include "cnode.h"
#include <ros/ros.h>

using namespace std;

class CEdge {
   public:
    enum DIRECTION_FIXED {
        DIRECTION_FIXED_NEGATIVE = -1,
        DIRECTION_FIXED_NONE     = 0,
        DIRECTION_FIXED_POSITIVE = 1,
        DIRECTION_FIXED_FOREWORD = 2,   // 机器人固定头超前行驶
    };

   public:
    CEdge();

    CEdge(const CNode &A, const CNode &B, CNode::IdType id, string edgeName = string("unset"),
          unsigned int _nAvaiable = 1, float _minConfidence = 0.3,
          CEdge::DIRECTION_FIXED _fixDirection = CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE, bool _isUltraOn = true,
          bool _isFallOn = true, bool _isLaserOn = true);

    CEdge(const CEdge &other);

    void PrintEdgeInfo();

    bool operator==(const CEdge &other) {
        if (id == other.id) {
            return true;
        }
        return false;
    }

    bool InEdge(CNode::IdType nNodeA, CNode::IdType nNodeB) {
        if ((nodeA_ == nNodeA && nodeB_ == nNodeB) || (nodeA_ == nNodeB && nodeB_ == nNodeA)) {
            return true;
        }
        return false;
    }

    void SetSpeed(double dSpeed) { max_speed_ = dSpeed; }

    void SetConfidence(double dConfidence) { minConfidence = dConfidence; }

    void SetFixDirection(DIRECTION_FIXED direction) { single_lane_ = direction; }

    bool operator!=(const CEdge &other) { return !(*this == other); }

    CEdge &operator=(const CEdge &other);

   public:
    CNode::IdType nodeA_;
    CNode::IdType nodeB_;
    unsigned int available_;
    float max_speed_;
    DIRECTION_FIXED single_lane_;
    bool avoidance_;
    bool anti_drop_;

    //以下参数暂不使用
    float length;
    CNode::IdType id;
    std::string edgeName;
    float leftSafeDist;
    float rightSafeDist;
    float minConfidence;
    bool isUltraOn;
};

#endif  // CEDGE_H
