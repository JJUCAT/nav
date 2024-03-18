#ifndef DYNAMICEDTOCTOMAP_H_
#define DYNAMICEDTOCTOMAP_H_

#include "dynamicEDT3D.h"
#include <octomap/OcTree.h>

/// A DynamicEDTOctomap object connects a DynamicEDT3D object to an octomap.
class DynamicEDTOctomap: public DynamicEDT3D {
public:
    /** Create a DynamicEDTOctomap object that maintains a distance transform in the bounding box given by bbxMin, bbxMax and clamps distances at maxdist.
     *  treatUnknownAsOccupied configures the treatment of unknown cells in the distance computation.
     *
     *  The constructor copies occupancy data but does not yet compute the distance map. You need to call udpate to do this.
     */
	DynamicEDTOctomap(float maxdist, octomap::OcTree* _octree, octomap::point3d bbxMin, octomap::point3d bbxMax, bool treatUnknownAsOccupied);

	virtual ~DynamicEDTOctomap();

	///trigger updating of the distance map. This will query the octomap for the set of changes since the last update.
	///If you set updateRealDist to false, computations will be faster (square root will be omitted), but you can only retrieve squared distances
	virtual void update(bool updateRealDist=true);

	///retrieves distance and closestObstacle (closestObstacle is to be discarded if distance is maximum distance, the method does not write closestObstacle in this case)
	void getDistanceAndClosestObstacle(octomap::point3d& p, float &distance, octomap::point3d& closestObstacle);

    ///retrieves distance at point
    float getDistance(octomap::point3d& p);
    ///retrieves distance at key
    float getDistance(octomap::OcTreeKey& k);

    ///retrieves squared distance in cells at point
    int getSquaredDistanceInCells(octomap::point3d& p);

	///retrieve maximum distance value
	float getMaxDist(){
	  return maxDist*octree->getResolution();
	}

	///retrieve squared maximum distance value in grid cells
	int getSquaredMaxDistCells(){
	  return maxDist_squared;
	}

	///distance value returned when requesting distance for a cell outside the map
	static float distanceValue_Error;
	///distance value returned when requesting distance in cell units for a cell outside the map
	static int distanceInCellsValue_Error;

private:
	void initializeOcTree(octomap::point3d bbxMin, octomap::point3d bbxMax);
	inline void insertMaxDepthLeafAtInitialize(octomap::OcTreeKey key);
	inline void updateMaxDepthLeaf(octomap::OcTreeKey& key, bool occupied);

	inline void worldToMap(octomap::point3d &p, int &x, int &y, int &z);
	inline void mapToWorld(int &x, int &y, int &z, octomap::point3d &p);
	inline void mapToWorld(int &x, int &y, int &z, octomap::OcTreeKey &key);

	octomap::OcTree* octree;
	bool unknownOccupied;
	int treeDepth;
	double treeResolution;
	octomap::OcTreeKey boundingBoxMinKey;
	octomap::OcTreeKey boundingBoxMaxKey;
	int offsetX, offsetY, offsetZ;
};

#endif /* DYNAMICEDTOCTOMAP_H_ */
