#ifndef _CSPACEVORONOI_H_
#define _CSPACEVORONOI_H_

#include "cspacevoronoilayer.h"
#include <list>
#include "point.h"
#include <map>
#include <set>

#include "node.h"
#include "simplemap.h"

typedef std::unordered_map< IntPose, Node*, IntPoseHash, intposeequality> NodeMapType;
typedef std::unordered_set< IntPose, IntPoseHash, intposeequality> PlannerMapType;
typedef std::unordered_set< IntPoint, IntPointHash, intpointequality> PlannerPointMapType;

#define HEURISTIC(p1, p2) (abs(p1.x-p2.x)+abs(p1.y-p2.y)+ANGULARDISTANCE(p1.theta-p2.theta))
#define NORMALIZEANGLE(a) (((a)<0) ? ((a)+maxAngle) : ((a)>=maxAngle ? ((a)-maxAngle) : (a)))
#define ANGULARDISTANCE(a) ((a)>maxAngle/2 ? (maxAngle-(a)) : (a))


class CSpaceVoronoi {
public:

  CSpaceVoronoi(std::vector<RobotColumn> &columns, SimpleMap<int> *map);
  ~CSpaceVoronoi();

  bool checkCollision(const Pose &p);
  bool collidesInAllOrientations(const double x, const double y);

  void initializeMap(SimpleMap<int> *_gridMap);
  void updateObstacles(std::vector<IntPose> *points, bool updateVoronoi);
  int updateClearance(int x, int y, int clearance, bool updateVoronoi=true);

  void update(bool updateRealDist=true);
  void prune();
  virtual std::list<IntPose>* computeShortestPath(IntPose start, IntPose goal);
  virtual void cleanup();

  void brushfireExpansion(IntPose pose, int minSqrDist, NodeMapType *nodeMap);
  void brushfireExpandLayer(int x, int y, int theta, bool makeGoalBubble, IntPose goal, int minSqrDist, NodeMapType *nodeMap);
  void expandNode(int nx, int ny, int nt, IntPose &goal, Node *s, NodeMapType &nodeMap, BucketPrioQueue<Node*> &openSet);

  int sizeX, sizeY;
  std::vector<CSpaceVoronoiLayer*> layers;
  double angularResolution;
  double sqrt2;
  int maxAngle;
  SimpleMap<int> gridMap;
  SimpleMap<int> newGridMap;

  std::list<IntPose> path;
  NodeMapType nodeMap;
  BucketPrioQueue<Node*> openSet;

  typedef std::set<IntPoint, intpointComparison> LastObstacleType;
  LastObstacleType *lastObstacles;
  std::vector<RobotColumn> columns;
};


#endif
