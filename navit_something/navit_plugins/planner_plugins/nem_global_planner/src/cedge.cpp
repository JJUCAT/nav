#include "nem_global_planner/cedge.h"

CEdge::CEdge()
    : length(0.0),
      id(std::numeric_limits<CNode::IdType>::max()),
      edgeName(""),
      max_speed_(1.0),
      leftSafeDist(0.0),
      rightSafeDist(0.0),
      nodeA_(std::numeric_limits<CNode::IdType>::max()),
      nodeB_(std::numeric_limits<CNode::IdType>::max()),
      available_(1),
      minConfidence(0.3),
      single_lane_(CEdge::DIRECTION_FIXED::DIRECTION_FIXED_NONE),
      isUltraOn(true),
      anti_drop_(true),
      avoidance_(true) {}

CEdge::CEdge(const CNode &A, const CNode &B, CNode::IdType id, string edgeName /*=string("unset")*/,
             unsigned int _nAvaiable, float _minConfidence, CEdge::DIRECTION_FIXED _fixDirection, bool _isUltraOn,
             bool _isFallOn, bool _isLaserOn) {
    this->nodeA_        = A.id;
    this->nodeB_        = B.id;
    this->id            = id;
    this->edgeName      = edgeName;
    this->length        = sqrt((A.x - B.x) * (A.x - B.x) + (A.y - B.y) * (A.y - B.y));
    this->max_speed_    = 1.0;
    this->rightSafeDist = 0.0;
    this->leftSafeDist  = 0.0;
    available_          = _nAvaiable;
    minConfidence       = _minConfidence;
    single_lane_        = _fixDirection;
    isUltraOn           = _isUltraOn;
    anti_drop_          = _isFallOn;
    avoidance_          = _isLaserOn;
}

CEdge::CEdge(const CEdge &other)
    : length(other.length),
      id(other.id),
      edgeName(other.edgeName),
      max_speed_(other.max_speed_),
      leftSafeDist(other.leftSafeDist),
      rightSafeDist(other.rightSafeDist),
      nodeA_(other.nodeA_),
      nodeB_(other.nodeB_),
      available_(other.available_),
      minConfidence(other.minConfidence),
      single_lane_(other.single_lane_),
      isUltraOn(other.isUltraOn),
      anti_drop_(other.anti_drop_),
      avoidance_(other.avoidance_) {}

CEdge &CEdge::operator=(const CEdge &other) {
    if (this == &other) {
        return *this;
    }

    length        = other.length;
    id            = other.id;
    edgeName      = other.edgeName;
    max_speed_    = other.max_speed_;
    leftSafeDist  = other.leftSafeDist;
    rightSafeDist = other.rightSafeDist;
    nodeA_        = other.nodeA_;
    nodeB_        = other.nodeB_;
    available_    = other.available_;
    minConfidence = other.minConfidence;
    single_lane_  = other.single_lane_;
    isUltraOn     = other.isUltraOn;
    anti_drop_    = other.anti_drop_;
    avoidance_    = other.avoidance_;
    return *this;
}

void CEdge::PrintEdgeInfo() {
    ROS_INFO("\nedge id: %d,  %lld <----> %lld \n available: %d, max_speed: %f, direction: %d, avoidance: %d, anti_drop: %d",
        this->id, this->nodeA_, this->nodeB_, this->available_, this->max_speed_, this->single_lane_, this->avoidance_, this->anti_drop_);

}
