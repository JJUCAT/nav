#include "nem_global_planner/cnode.h"

CNode::CNode()
    : x(0.0),
      y(0.0),
      r(0.0),
      map_id(std::numeric_limits<unsigned long>::max()),
      id(std::numeric_limits<unsigned int>::max()),
      type(-1),
      nodeName("unset"),
      bTurnNode(false),
      visited(false) {}

CNode::CNode(const CNode &other)
    : x(other.x),
      y(other.y),
      r(other.r),
      map_id(other.map_id),
      id(other.id),
      type(other.type),
      nodeName(other.nodeName),
      bTurnNode(other.bTurnNode),
      visited(other.visited),
      adjacentNodes(other.adjacentNodes) {}

CNode::CNode(float _x, float _y, float _r, IdType _map_id, int _type, IdType _id, std::string _nodeName /*=unset*/)
    : x(_x),
      y(_y),
      r(_r),
      map_id(_map_id),
      id(_id),
      type(_type),
      nodeName(_nodeName),
      bTurnNode(false),
      visited(false) {}

void CNode::PrintNodeInfo() {
    ROS_INFO("node_id: %lld, map_id: %ld, type: %d, x: %f, y: %f, r: %f", this->id, this->map_id, this->type, this->x,
             this->y, this->r);
}

CNode &CNode::operator=(const CNode &other) {
    if (this == &other) {
        return *this;
    }
    x             = other.x;
    y             = other.y;
    r             = other.r;
    map_id        = other.map_id;
    id            = other.id;
    type          = other.type;
    nodeName      = other.nodeName;
    bTurnNode     = other.bTurnNode;
    visited       = other.visited;
    adjacentNodes = other.adjacentNodes;
    return *this;
}
