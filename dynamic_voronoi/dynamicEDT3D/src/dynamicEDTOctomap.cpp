#include <dynamicEDT3D/dynamicEDTOctomap.h>

float DynamicEDTOctomap::distanceValue_Error = -1.0;
int DynamicEDTOctomap::distanceInCellsValue_Error = -1;

DynamicEDTOctomap::DynamicEDTOctomap(float maxdist, octomap::OcTree* _octree, octomap::point3d bbxMin, octomap::point3d bbxMax, bool treatUnknownAsOccupied)
: DynamicEDT3D(((int) (maxdist/_octree->getResolution()+1)*((int) (maxdist/_octree->getResolution()+1)))), octree(_octree), unknownOccupied(treatUnknownAsOccupied)
{
	treeDepth = octree->getTreeDepth();
	treeResolution = octree->getResolution();
	initializeOcTree(bbxMin, bbxMax);
	octree->enableChangeDetection(true);
}

DynamicEDTOctomap::~DynamicEDTOctomap() {

}


void DynamicEDTOctomap::update(bool updateRealDist){

	for(octomap::KeyBoolMap::const_iterator it = octree->changedKeysBegin(), end=octree->changedKeysEnd(); it!=end; ++it){
		//the keys in this list all go down to the lowest level!

		octomap::OcTreeKey key = it->first;

		//ignore changes outside of bounding box
		if(key[0] < boundingBoxMinKey[0] || key[1] < boundingBoxMinKey[1] || key[2] < boundingBoxMinKey[2])
			continue;
		if(key[0] > boundingBoxMaxKey[0] || key[1] > boundingBoxMaxKey[1] || key[2] > boundingBoxMaxKey[2])
			continue;

		octomap::OcTreeNode* node = octree->search(key);
		assert(node);
		//"node" is not necessarily at lowest level, BUT: the occupancy value of this node
		//has to be the same as of the node indexed by the key *it

		updateMaxDepthLeaf(key, octree->isNodeOccupied(node));
	}
	octree->resetChangeDetection();

	DynamicEDT3D::update(updateRealDist);
}

void DynamicEDTOctomap::initializeOcTree(octomap::point3d bbxMin, octomap::point3d bbxMax){

    boundingBoxMinKey = octree->coordToKey(bbxMin);
    boundingBoxMaxKey = octree->coordToKey(bbxMax);

	offsetX = -boundingBoxMinKey[0];
	offsetY = -boundingBoxMinKey[1];
	offsetZ = -boundingBoxMinKey[2];

	int _sizeX = boundingBoxMaxKey[0] - boundingBoxMinKey[0] + 1;
	int _sizeY = boundingBoxMaxKey[1] - boundingBoxMinKey[1] + 1;
	int _sizeZ = boundingBoxMaxKey[2] - boundingBoxMinKey[2] + 1;

	initializeEmpty(_sizeX, _sizeY, _sizeZ, false);


	if(unknownOccupied == false){
		for(octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(bbxMin,bbxMax), end=octree->end_leafs_bbx(); it!= end; ++it){
			if(octree->isNodeOccupied(*it)){
				int nodeDepth = it.getDepth();
				if( nodeDepth == treeDepth){
					insertMaxDepthLeafAtInitialize(it.getKey());
				} else {
					int cubeSize = 1 << (treeDepth - nodeDepth);
					octomap::OcTreeKey key=it.getIndexKey();
					for(int dx = 0; dx < cubeSize; dx++)
						for(int dy = 0; dy < cubeSize; dy++)
							for(int dz = 0; dz < cubeSize; dz++){
								unsigned short int tmpx = key[0]+dx;
								unsigned short int tmpy = key[1]+dy;
								unsigned short int tmpz = key[2]+dz;

								if(boundingBoxMinKey[0] > tmpx || boundingBoxMinKey[1] > tmpy || boundingBoxMinKey[2] > tmpz)
									continue;
								if(boundingBoxMaxKey[0] < tmpx || boundingBoxMaxKey[1] < tmpy || boundingBoxMaxKey[2] < tmpz)
									continue;

								insertMaxDepthLeafAtInitialize(octomap::OcTreeKey(tmpx, tmpy, tmpz));
							}
				}
			}
		}
	} else {
		octomap::OcTreeKey key;
		for(int dx=0; dx<sizeX; dx++){
			key[0] = boundingBoxMinKey[0] + dx;
			for(int dy=0; dy<sizeY; dy++){
				key[1] = boundingBoxMinKey[1] + dy;
				for(int dz=0; dz<sizeZ; dz++){
					key[2] = boundingBoxMinKey[2] + dz;

					octomap::OcTreeNode* node = octree->search(key);
					if(!node || octree->isNodeOccupied(node)){
						insertMaxDepthLeafAtInitialize(key);
					}
				}
			}
		}
	}
}

void DynamicEDTOctomap::insertMaxDepthLeafAtInitialize(octomap::OcTreeKey key){
	bool isSurrounded = true;


	for(int dx=-1; dx<=1; dx++)
		for(int dy=-1; dy<=1; dy++)
			for(int dz=-1; dz<=1; dz++){
				if(dx==0 && dy==0 && dz==0)
					continue;
				octomap::OcTreeNode* node = octree->search(octomap::OcTreeKey(key[0]+dx, key[1]+dy, key[2]+dz));
				if((!unknownOccupied && node==NULL) || ((node!=NULL) && (octree->isNodeOccupied(node)==false))){
					isSurrounded = false;
					break;
				}
			}

	if(isSurrounded){
		//obstacles that are surrounded by obstacles do not need to be put in the queues,
		//hence this initialization
		dataCell c;
		int x = key[0]+offsetX;
		int y = key[1]+offsetY;
		int z = key[2]+offsetZ;
		c.obstX = x;
		c.obstY = y;
		c.obstZ = z;
		c.sqdist = 0;
		c.dist = 0.0;
		c.queueing = fwProcessed;
		c.needsRaise = false;
		data[x][y][z] = c;
	} else {
		setObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
	}
}

void DynamicEDTOctomap::updateMaxDepthLeaf(octomap::OcTreeKey& key, bool occupied){
	if(occupied)
		setObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
	else
		removeObstacle(key[0]+offsetX, key[1]+offsetY, key[2]+offsetZ);
}

void DynamicEDTOctomap::worldToMap(octomap::point3d &p, int &x, int &y, int &z){
	octomap::OcTreeKey key = octree->coordToKey(p);
	x = key[0] + offsetX;
	y = key[1] + offsetY;
	z = key[2] + offsetZ;
}

void DynamicEDTOctomap::mapToWorld(int &x, int &y, int &z, octomap::point3d &p){
	octomap::OcTreeKey key(x-offsetX, y-offsetY, z-offsetZ);

	p = octree->keyToCoord(key);
}

void DynamicEDTOctomap::mapToWorld(int &x, int &y, int &z, octomap::OcTreeKey &key){
	key = octomap::OcTreeKey(x-offsetX, y-offsetY, z-offsetZ);
}

void DynamicEDTOctomap::getDistanceAndClosestObstacle(octomap::point3d& p, float &distance, octomap::point3d& closestObstacle){
    distance = maxDist;
	int x,y,z;
	worldToMap(p, x, y, z);
	if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
		dataCell c= data[x][y][z];

		distance = c.dist*treeResolution;
		if(c.obstX != invalidObstData){
			mapToWorld(c.obstX, c.obstY, c.obstZ, closestObstacle);
		} else {
		  //If we are at maxDist, it can very well be that there is no valid closest obstacle data for this cell, this is not an error.
		}
	} else {
	  distance = distanceValue_Error;
	  std::cerr<<"Point outside map! "<<p.x()<<","<<p.y()<<","<<p.z()<<std::endl;
	}
}

float DynamicEDTOctomap::getDistance(octomap::point3d& p){
  int x,y,z;
  worldToMap(p, x, y, z);
  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
      return data[x][y][z].dist*treeResolution;
  } else {
      std::cerr<<"Point outside map! "<<p.x()<<","<<p.y()<<","<<p.z()<<std::endl;
      return distanceValue_Error;
  }
}

float DynamicEDTOctomap::getDistance(octomap::OcTreeKey& k){
  int x = k[0] + offsetX;
  int y = k[1] + offsetY;
  int z = k[2] + offsetZ;

  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
      return data[x][y][z].dist*treeResolution;
  } else {
      std::cerr<<"Key outside map! "<<k[0]<<","<<k[1]<<","<<k[2]<<std::endl;
      return distanceValue_Error;
  }
}

int DynamicEDTOctomap::getSquaredDistanceInCells(octomap::point3d& p){
  int x,y,z;
  worldToMap(p, x, y, z);
  if(x>=0 && x<sizeX && y>=0 && y<sizeY && z>=0 && z<sizeZ){
    return data[x][y][z].sqdist;
  } else {
    std::cerr<<"Point outside map! "<<p.x()<<","<<p.y()<<","<<p.z()<<std::endl;
    return distanceInCellsValue_Error;
  }
}

