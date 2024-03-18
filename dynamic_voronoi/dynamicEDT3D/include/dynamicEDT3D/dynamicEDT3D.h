#ifndef _DYNAMICEDT3D_H_
#define _DYNAMICEDT3D_H_


#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include <queue>

#include "bucketedqueue.h"

//! A DynamicEDT3D object computes and updates a 3D distance map.
class DynamicEDT3D {
  
public:
  
  DynamicEDT3D(int _maxdist_squared);
  ~DynamicEDT3D();

  //! Initialization with an empty map
  void initializeEmpty(int _sizeX, int _sizeY, int sizeZ, bool initGridMap=true);
  //! Initialization with a given binary map (false==free, true==occupied)
  void initializeMap(int _sizeX, int _sizeY, int sizeZ, bool*** _gridMap);

  //! add an obstacle at the specified cell coordinate
  void occupyCell(int x, int y, int z);
  //! remove an obstacle at the specified cell coordinate
  void clearCell(int x, int y, int z);
  //! remove old dynamic obstacles and add the new ones
  void exchangeObstacles(std::vector<INTPOINT3D> newObstacles);

  //! update distance map to reflect the changes
  virtual void update(bool updateRealDist=true);

  //! returns the obstacle distance at the specified location
  float getDistance( int x, int y, int z );
  //! gets the closest occupied cell for that location
  INTPOINT3D getClosestObstacle( int x, int y, int z );

  //! returns the squared obstacle distance in cell units at the specified location
  int getSQCellDistance( int x, int y, int z );
  //! checks whether the specficied location is occupied
  bool isOccupied(int x, int y, int z);

  //! returns the x size of the workspace/map
  unsigned int getSizeX() {return sizeX;}
  //! returns the y size of the workspace/map
  unsigned int getSizeY() {return sizeY;}
  //! returns the z size of the workspace/map
  unsigned int getSizeZ() {return sizeZ;}

  typedef enum {invalidObstData = INT_MAX} ObstDataState;


protected: 
  struct dataCell {
    float dist;
    int obstX;
    int obstY;
    int obstZ;
    int sqdist;
    char queueing;
    bool needsRaise;
  };

  typedef enum {free=0, occupied=1} State;
  typedef enum {fwNotQueued=1, fwQueued=2, fwProcessed=3, bwQueued=4, bwProcessed=1} QueueingState;
  
  // methods
  inline void raiseCell(INTPOINT3D &p, dataCell &c, bool updateRealDist);
  inline void propagateCell(INTPOINT3D &p, dataCell &c, bool updateRealDist);
  inline void inspectCellRaise(int &nx, int &ny, int &nz, bool updateRealDist);
  inline void inspectCellPropagate(int &nx, int &ny, int &nz, dataCell &c, bool updateRealDist);

  void setObstacle(int x, int y, int z);
  void removeObstacle(int x, int y, int z);

private:
  void commitAndColorize(bool updateRealDist=true);

  inline bool isOccupied(int &x, int &y, int &z, dataCell &c);

  // queues
  BucketPrioQueue<INTPOINT3D> open;

  std::vector<INTPOINT3D> removeList;
  std::vector<INTPOINT3D> addList;
  std::vector<INTPOINT3D> lastObstacles;

  // maps
protected:
  int sizeX;
  int sizeY;
  int sizeZ;
  int sizeXm1;
  int sizeYm1;
  int sizeZm1;

  dataCell*** data;
  bool*** gridMap;

  // parameters
  int padding;
  double doubleThreshold;

  double sqrt2;
  double maxDist;
  int maxDist_squared;
};


#endif

