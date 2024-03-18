#include <dynamicEDT3D/dynamicEDTOctomap.h>

#include <iostream>



int main( int argc, char *argv[] ) {
  if(argc<=1){
    std::cout<<"usage: "<<argv[0]<<" <octoMap.bt>"<<std::endl;
    exit(0);
  }

  octomap::OcTree *tree = NULL;
  tree = new octomap::OcTree(0.05);

  //read in octotree
  tree->readBinary(argv[1]);

  std::cout<<"read in tree, "<<tree->getNumLeafNodes()<<" leaves "<<std::endl;

  double x,y,z;
  tree->getMetricMin(x,y,z);
  octomap::point3d min(x,y,z);
  //std::cout<<"Metric min: "<<x<<","<<y<<","<<z<<std::endl;
  tree->getMetricMax(x,y,z);
  octomap::point3d max(x,y,z);
  //std::cout<<"Metric max: "<<x<<","<<y<<","<<z<<std::endl;

  bool unknownAsOccupied = true;
  unknownAsOccupied = false;
  float maxDist = 1.0;
  //- the first argument ist the max distance at which distance computations are clamped
  //- the second argument is the octomap
  //- arguments 3 and 4 can be used to restrict the distance map to a subarea
  //- argument 5 defines whether unknown space is treated as occupied or free
  //The constructor copies data but does not yet compute the distance map
  DynamicEDTOctomap distmap(maxDist, tree, min, max, unknownAsOccupied);

  //This computes the distance map
  distmap.update(); 

  //This is how you can query the map
  octomap::point3d p(5.0,5.0,0.6);
  //As we don't know what the dimension of the loaded map are, we modify this point
  p.x() = min.x() + 0.3 * (max.x() - min.x());
  p.y() = min.y() + 0.6 * (max.y() - min.y());
  p.z() = min.z() + 0.5 * (max.z() - min.z());

  octomap::point3d closestObst;
  float distance;

  distmap.getDistanceAndClosestObstacle(p, distance, closestObst);

  std::cout<<"\n\ndistance at point "<<p.x()<<","<<p.y()<<","<<p.z()<<" is "<<distance<<std::endl;
  if(distance < distmap.getMaxDist())
    std::cout<<"closest obstacle to "<<p.x()<<","<<p.y()<<","<<p.z()<<" is at "<<closestObst.x()<<","<<closestObst.y()<<","<<closestObst.z()<<std::endl;

  //if you modify the octree via tree->insertScan() or tree->updateNode()
  //just call distmap.update() again to adapt the distance map to the changes made

  delete tree;

  return 0;
}
