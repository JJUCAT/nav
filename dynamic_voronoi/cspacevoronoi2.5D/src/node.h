#ifndef _NODE_H_
#define _NODE_H_

#include <unordered_map>
#include <unordered_set>

#include "MemoryManager.h"

class Node : public MemoryManagerObject {
  friend class MemoryManager<Node>;
public:
  typedef enum { unvisited, starting, voro, goaling } phaseState;
  typedef enum { none, open, closed } openState;
  IntPose pos;

  int f,g,h;

  Node *prev; // previous node in tree
  phaseState phase;
  openState state;


  void destroy() {}
  void recycle() {}

private:
  Node() {  }


};

class QueuedNode {
public:
  QueuedNode(Node* _n)
    :n(_n), prio(_n->f)
  {}

  Node* n;
  int prio;
};

class QueuedNodeCompare {
public:
  bool operator() (const QueuedNode &lhs, const QueuedNode &rhs) const{
    return (lhs.prio > rhs.prio);
  }
};

struct intposeComparison {
  inline bool operator() ( const IntPose& lhs, const IntPose& rhs) const {
    if (lhs.x < rhs.x) return true;
    if (lhs.x > rhs.x) return false;
    if (lhs.y < rhs.y) return true;
    if (lhs.y > rhs.y) return false;
    if (lhs.theta < rhs.theta) return true;
    return false;
  }
};

struct intpointComparison {
  inline bool operator() ( const INTPOINT& lhs, const INTPOINT& rhs) const {
    if (lhs.x < rhs.x) return true;
    if (lhs.x > rhs.x) return false;
    if (lhs.y < rhs.y) return true;
    return false;
  }
};

struct intposeAsPointComparison {
  inline bool operator() ( const IntPose& lhs, const IntPose& rhs) const {
    if (lhs.x < rhs.x) return true;
    if (lhs.x > rhs.x) return false;
    if (lhs.y < rhs.y) return true;
    return false;
  }
};

struct IntPointHash {
   std::size_t operator()(IntPoint pose) const {
     std::size_t h = pose.x + pose.y*1000;
     return h;
   }
};

struct intpointequality : std::binary_function<IntPoint, IntPoint, bool>
{
    bool operator()(IntPoint const& A, IntPoint const& B) const
    {
      if (A.x != B.x) return false;
      if (A.y != B.y) return false;
      return true;
    }
};

struct IntPoseHash {
   std::size_t operator()(IntPose pose) const {
     std::size_t h = pose.x + pose.y*1000 + pose.theta*1000000;
     return h;
   }
};

struct intposeequality : std::binary_function<IntPose, IntPose, bool> {
    bool operator()(IntPose const& A, IntPose const& B) const
    {
      if ((A.x != B.x) || (A.y != B.y) || (A.theta != B.theta)) return false;
      return true;
    }
};

#endif
