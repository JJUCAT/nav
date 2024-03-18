#ifndef _CSPACEVORONOI_LAYER_H_
#define _CSPACEVORONOI_LAYER_H_


#include "dynamicvoronoi.h"
#include "simplemap.h"
#include <math.h>

class CSpaceVoronoiLayer : public DynamicVoronoi {
public:
  CSpaceVoronoiLayer(std::vector<RobotColumn> &columns, double t);
  ~CSpaceVoronoiLayer();
  void initializeMap(SimpleMap<int> *_gridMap);
  void updateClearance(int x, int y, int col, bool newlyOccupied, bool updateVoronoi=true);
  void saveCountMap(const char* filename);
  void drawRobot(int x, int y, SimpleMap<int> &map);

  SimpleMap<int> countMap;

  void overlayFootprint(const CSpaceVoronoiLayer* other);

private:

  std::vector< std::vector<ScanLine> > columnBoundaries; // maps a y-offset to x-offsets, one std::vector<ScanLine> per robotColumn
  std::vector<RobotColumn> columns;
};

#endif
