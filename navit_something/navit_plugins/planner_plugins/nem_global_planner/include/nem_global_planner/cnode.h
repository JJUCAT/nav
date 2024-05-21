#ifndef CNODE_H
#define CNODE_H

#include <iostream>
#include <limits>
#include <string>
#include <vector>
#include <ros/ros.h>

using namespace std;

struct NodeVisited {
    int id;
    bool visited = false;
};

class CNode {
   public:
    using IdType = long ;

    CNode();

    CNode(const CNode &other);

    CNode(float _x, float _y, float _r, IdType _map_id, int _type, IdType _id, std::string _nodeName = string("unset"));

    void SetR(float r) { this->r = r; }

    void SetType(int type) { this->type = type; }

    void SetVisited() { this->visited = true; }

    void SetAdjacentNodes(IdType id) {
        NodeVisited node;
        node.id = id;
        this->adjacentNodes.push_back(node);
    }

    void SetAdjacentVisited(unsigned int i) { this->adjacentNodes[i].visited = true; }

    void PrintNodeInfo();

    bool operator==(const CNode &other) { return id == other.id; }

    bool operator!=(const CNode &other) { return !(*this == other); }

    CNode &operator=(const CNode &other);

   public:
    float x, y, r;
    IdType map_id;
    IdType id;
    int type;
    std::string nodeName;
    bool bTurnNode;
    bool visited = false;
    vector<NodeVisited> adjacentNodes;
};

#endif  // CNODE_H
