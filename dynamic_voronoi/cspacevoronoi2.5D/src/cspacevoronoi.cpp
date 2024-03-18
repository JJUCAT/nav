#include "cspacevoronoi.h"

#include <math.h>
#include <omp.h>

#define MINSQRDIST 1

template<> std::vector<Node*> MemoryManager<Node>::freeObjects = std::vector<Node*>();
template<> unsigned int MemoryManager<Node>::objectsInUse = 0;


CSpaceVoronoi::CSpaceVoronoi(std::vector<RobotColumn> &_columns, SimpleMap<int> *map) {
  MemoryManager<Node>::reserve(1000000);

  columns = _columns;

  double radius = 0;
  for (unsigned int i=0; i<columns.size(); i++) {
    double x = columns[i].x1;
    double y = columns[i].y1;
    double r = sqrt(x*x+y*y);
    if (r>radius) radius = r;
    x = columns[i].x2;
    y = columns[i].y2;
    r = sqrt(x*x+y*y);
    if (r>radius) radius = r;
  }
  sqrt2 = sqrt(2);  
  angularResolution = 1/radius;
  fprintf(stderr, "Angular resolution: %f deg,\n", angularResolution/2/M_PI*360);
  assert(angularResolution>0);

  printf("Generating layers\n");
  for (double theta = 0; theta<=2*M_PI; theta+=angularResolution) {
    layers.push_back(new CSpaceVoronoiLayer(columns,theta));
  }
  maxAngle = layers.size();

  //now we overlay each parts with the one for the succeeding discrete orientation to account for the flooring that takes place when checking a pose
  //(incoming theta will be mapped to a bin, robot can have all the orientations in the bin)
  for(unsigned int i=0; i<layers.size()-1; i++){
    layers[i]->overlayFootprint(layers[i+1]);
  }
  //the first layer has already been overlayed but we need a clean version to do the overlay with the last layer
  CSpaceVoronoiLayer temp(columns,0.0);
  layers[layers.size()-1]->overlayFootprint(&temp);  

  initializeMap(map);
  map->writeToPGM("test.pgm");
  lastObstacles = new LastObstacleType();

}


CSpaceVoronoi::~CSpaceVoronoi() {
  delete lastObstacles;
  for (unsigned int i=0; i<layers.size(); i++) {
    delete layers[i];
  }
}

void CSpaceVoronoi::initializeMap(SimpleMap<int> *_gridMap) {
  sizeX = _gridMap->getMapSizeX();
  sizeY = _gridMap->getMapSizeY();
  gridMap.copyFrom(_gridMap);
  newGridMap.copyFrom(_gridMap);


  printf("INITIALIZING LAYERS\n");

#pragma omp parallel for num_threads(6)
  for (unsigned int i=0; i<layers.size(); i++) {
    layers[i]->initializeMap(_gridMap);
  }
}

void CSpaceVoronoi::updateObstacles(std::vector<IntPose> *points, bool updateVoronoi) {
  LastObstacleType *newObstacles = new LastObstacleType();
  LastObstacleType::iterator it;

  LastObstacleType::const_iterator endit = lastObstacles->end();
  for (it = lastObstacles->begin(); it!=endit; ++it) {
    newGridMap.setCell(it->x, it->y, INT_MAX);
  }  

  unsigned int size = points->size();
  for (unsigned int i=0; i<size; i++) {
    IntPose pose = points->at(i);
    INTPOINT p(pose.x, pose.y);
    if (newGridMap.getCell(p)<=pose.theta) continue;
    it = lastObstacles->find(p);
    if (it != lastObstacles->end()) {
      lastObstacles->erase(it);
    }
    newGridMap.setCell(p.x, p.y, pose.theta);
    newObstacles->insert(p);
  }  

  endit = lastObstacles->end();
  for (it = lastObstacles->begin(); it!=endit; ++it) {
    updateClearance(it->x,it->y,INT_MAX);
  }  

  endit = newObstacles->end();
  for (it = newObstacles->begin(); it!=endit; ++it) {
    updateClearance(it->x, it->y, newGridMap.getCell(it->x, it->y));
  }  

  lastObstacles->clear();
  delete lastObstacles;
  lastObstacles = newObstacles;
}

int CSpaceVoronoi::updateClearance(int x, int y, int clearance, bool updateVoronoi) {
  int oldClearance = gridMap.getCell(x,y);
  gridMap.setCell(x,y,clearance);

  for (int i=0; i<(int)columns.size(); i++) {
    bool collidesNow = (clearance<=columns[i].upper);
    bool collidesBefore = (oldClearance<=columns[i].upper);
    if (collidesNow == collidesBefore) continue;
#pragma omp parallel for num_threads(6)
    for (unsigned int l=0; l<layers.size(); l++) 
      layers[l]->updateClearance(x,y,i,collidesNow,updateVoronoi);
  }
  return oldClearance;
}


void CSpaceVoronoi::update(bool updateRealDist) {
#pragma omp parallel for num_threads(4)
  for (unsigned int i=0; i<layers.size(); i++) {
    layers[i]->update(updateRealDist);
  }
}

void CSpaceVoronoi::prune() {

#pragma omp parallel for num_threads(4)
  for (unsigned int i=0; i<layers.size(); i++) layers[i]->prune();
}


std::list<IntPose>* CSpaceVoronoi::computeShortestPath(IntPose start, IntPose goal) {
  if (start==goal) {
    path.push_back(start);
    path.push_back(goal);
    fprintf(stderr, "Start and goal are identical!\n");
    return &path;
  }
  
  if (layers[start.theta]->getSqrDistance(start.x, start.y) < MINSQRDIST) {
    fprintf(stderr, "Start is occupied!\n");
    return &path;
  }
  if (layers[goal.theta]->getSqrDistance(goal.x, goal.y) < MINSQRDIST) {
    fprintf(stderr, "Goal is occupied!\n");
    return &path;
  }

  // modify the voronoi without changing real-valued distances
  int oldStartClearance = updateClearance(start.x, start.y, 0, false);
  int oldGoalClearance = updateClearance(goal.x, goal.y, 0, false);
  update(false);


  Node *startNode = MemoryManager<Node>::getNew();
  startNode->g = 0;
  startNode->h = HEURISTIC(start, goal);
  startNode->f = startNode->h;
  startNode->prev = NULL;
  startNode->pos = start;
  startNode->phase = Node::starting;
  startNode->state = Node::none;
  nodeMap[start] = startNode;

  Node* goalNode = MemoryManager<Node>::getNew();
  goalNode->g = INT_MAX;
  goalNode->h = 0;
  goalNode->f = goalNode->g + goalNode->h;
  goalNode->prev = NULL;
  goalNode->pos = goal;
  goalNode->phase = Node::goaling;
  goalNode->state = Node::none;
  nodeMap[goal] = goalNode;	

  for (unsigned int i=0; i<layers.size(); i++) {
    brushfireExpandLayer(goal.x, goal.y, i, true, goal, MINSQRDIST, &nodeMap);
    brushfireExpandLayer(start.x, start.y, i, false, goal, MINSQRDIST, &nodeMap);
  }


  startNode->state = Node::open;
  openSet.push(startNode->f, startNode);

  int steps = 0;
  while (!openSet.empty()) {
    Node* s = openSet.pop();
    if(s->state == Node::closed) continue;

    steps++;

    //We are expanding the goal, i.e., we have found a path
    if (s->pos == goal) { 
      while (s!=startNode) {
        path.push_front(s->pos);
        s = s->prev;
      }
      path.push_front(s->pos);
      updateClearance(start.x, start.y, oldStartClearance, false);
      updateClearance(goal.x, goal.y, oldGoalClearance, false);
      update(false);

      return &path;
    }

    //Expand current node s
    s->state = Node::closed;

    int x = s->pos.x;
    int y = s->pos.y;
    int theta = s->pos.theta;
        
    if (x>0)     expandNode(x-1, y  , theta, goal,s,nodeMap,openSet);
    if (x<sizeX) expandNode(x+1, y  , theta, goal,s,nodeMap,openSet);
    if (y>0)     expandNode(x  , y-1, theta, goal,s,nodeMap,openSet);
    if (y<sizeY) expandNode(x  , y+1, theta, goal,s,nodeMap,openSet);
    expandNode(x,y,NORMALIZEANGLE(theta-1),goal,s,nodeMap,openSet);
    expandNode(x,y,NORMALIZEANGLE(theta+1),goal,s,nodeMap,openSet);
  }

  return &path;
}

void CSpaceVoronoi::expandNode(int nx, int ny, int nt, IntPose &goal, Node *s, NodeMapType &nodeMap, BucketPrioQueue<Node*> &openSet) {
  static const double EPSILON = 0.00001;

          //skip the parent node
          IntPose newPos(nx,ny,nt);
          if (s->prev && newPos==s->prev->pos) return;
        

          int lt = nt;

          //once we are on the voronoi graph, skip cells that leave it
          bool isVoronoi = layers[lt]->isVoronoi(nx,ny);
          if (s->phase==Node::voro && !isVoronoi) return;

          //compute costs from parent to here
          int gNew = s->g + 1;
	  
          //find out if node already exists
          NodeMapType::iterator it = nodeMap.find(newPos);
          Node *n;
          if (it!=nodeMap.end()) n = it->second;
          else n = NULL;
          
          //these phases can be substituted by simple boolean markers
          if (s->phase==Node::goaling && ((n && n->phase!=Node::goaling) || !n)) return;
          if (s->phase==Node::starting && !n) return;

          if (n && gNew >= n->g - EPSILON) return;

          if (!n) {
            n = MemoryManager<Node>::getNew();	
            n->pos = newPos;
            n->phase = s->phase;
            n->h = HEURISTIC(newPos, goal);
            n->state = Node::open;
            nodeMap[newPos] = n;
          }

          n->prev = s;	  
          n->g = gNew;	  
          n->f = n->g + n->h;
          //check if we are switching phase from starting to voro
          if(n->phase==Node::starting && isVoronoi){
            n->phase = Node::voro;
          }	

          openSet.push(n->f, n);

}

void CSpaceVoronoi::brushfireExpandLayer(int x, int y, int theta, bool makeGoalBubble, IntPose goal, int minSqrDist, NodeMapType *nodeMap) {
  std::queue<IntPoint> q;
  q.push(IntPoint(x,y));
  
  CSpaceVoronoiLayer *layer = layers[theta];

  while(!q.empty()) {
    IntPoint p = q.front();
    q.pop();
    int x = p.x;
    int y = p.y;
    
    for (int dx=-1; dx<=1; dx++) {
      int nx = x+dx;
      if (nx<0 || nx>=sizeX) continue;
      for (int dy=-1; dy<=1; dy++) {
        int ny = y+dy;
        if (dx && dy) continue;
        if (ny<0 || ny>=sizeY) continue;
        IntPoint n = IntPoint(nx, ny);
          
        if (layer->getSqrDistance(nx,ny)<1) continue;
        IntPose nPose = IntPose(nx, ny, theta);
        if (nodeMap->count(nPose)>0) continue;
        //        if (nd) continue;


        Node *nd = MemoryManager<Node>::getNew();
        nd->g = INT_MAX;
        nd->h = HEURISTIC(nPose, goal);
        nd->f = INT_MAX;
        nd->prev = NULL;
        nd->pos = nPose;
        nd->state = Node::none;

        bool isVoronoi = layer->isVoronoi(nx,ny);

        if (makeGoalBubble) nd->phase = Node::goaling;
        else {
          if (isVoronoi) nd->phase = Node::voro;
          else nd->phase = Node::starting;
        }


        (*nodeMap)[nPose] = nd;
        if (!isVoronoi) q.push(n);
      }
    }      
  }
}

void CSpaceVoronoi::cleanup() {
	for(NodeMapType::iterator it=nodeMap.begin(); it !=nodeMap.end(); ++it){
		MemoryManager<Node>::destroy(it->second);
	}
  nodeMap.clear();
  openSet.clear();
  path.clear();
  char *dumbo = new char[1024*1024*100];
  delete[] dumbo;
}

bool CSpaceVoronoi::checkCollision(const Pose &p) {
  int lt = p.theta/angularResolution;
  return (layers[lt]->countMap.getCell(p.x, p.y)>0);
}

bool CSpaceVoronoi::collidesInAllOrientations(const double x, const double y){
  for(unsigned int i=0; i<layers.size(); i++){
    if(layers[i]->countMap.getCell(x, y) == 0)
      return false;
  }
  return true;
}
