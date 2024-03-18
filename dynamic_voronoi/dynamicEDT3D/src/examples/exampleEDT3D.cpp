#include <dynamicEDT3D/dynamicEDT3D.h>

#include <iostream>

int main( int , char** ) {

	//we build a sample map
  int sizeX, sizeY, sizeZ;
  sizeX=100;
  sizeY=100;
  sizeZ=100;

  bool*** map;
  map = new bool**[sizeX];
  for(int x=0; x<sizeX; x++){
  	map[x] = new bool*[sizeY];
  	for(int y=0; y<sizeY; y++){
  		map[x][y] = new bool[sizeZ];
  		for(int z=0; z<sizeZ; z++){
  				if(x<2 || x > sizeX-3 || y < 2 || y > sizeY-3 || z<2 || z > sizeZ-3)
  					map[x][y][z] = 1;
          else
          	map[x][y][z] = 0;
  		}
  	}
  }

  map[51][45][67] = 1;
  map[50][50][68] = 1;

  // create the EDT object and initialize it with the map
  int maxDistInCells = 20;
  DynamicEDT3D distmap(maxDistInCells*maxDistInCells);
  distmap.initializeMap(sizeX, sizeY, sizeZ, map);

  //compute the distance map
  distmap.update();

  // now perform some updates with random obstacles
  int numPoints = 20;
  for (int frame=1; frame<=10; frame++) {
  	std::cout<<"\n\nthis is frame #"<<frame<<std::endl;
    std::vector<IntPoint3D> newObstacles;
    for (int i=0; i<numPoints; i++) {
      double x = 2+rand()/(double)RAND_MAX*(sizeX-4);
      double y = 2+rand()/(double)RAND_MAX*(sizeY-4);
      double z = 2+rand()/(double)RAND_MAX*(sizeZ-4);
      newObstacles.push_back(IntPoint3D(x,y,z));
    }

    // register the new obstacles (old ones will be removed)
    distmap.exchangeObstacles(newObstacles);

    //update the distance map
    distmap.update();

    //retrieve distance at a point
    float dist = distmap.getDistance(30,67,33);
    int distSquared = distmap.getSQCellDistance(30,67,33);
    std::cout<<"distance at  30,67,33: "<< dist << " squared: "<< distSquared << std::endl;
    if(distSquared == maxDistInCells*maxDistInCells)
    	std::cout<<"we hit a cell with d = dmax, distance value is clamped."<<std::endl;

    //retrieve closest occupied cell at a point
    IntPoint3D closest = distmap.getClosestObstacle(30,67,33);
    if(closest.x == DynamicEDT3D::invalidObstData)
    	std::cout<<"we hit a cell with d = dmax, no information about closest occupied cell."<<std::endl;
    else
    std::cout<<"closest occupied cell to 30,67,33: "<< closest.x<<","<<closest.y<<","<<closest.z<<std::endl;

  }


  std::cout<<"\n\nthis is the last frame"<<std::endl;

  // now remove all random obstacles again.
  std::vector<IntPoint3D> empty;
  distmap.exchangeObstacles(empty);
  distmap.update();


  //retrieve distance at a point
  float dist = distmap.getDistance(30,67,33);
  int distSquared = distmap.getSQCellDistance(30,67,33);
  std::cout<<"distance at  30,67,33: "<< dist << " squared: "<< distSquared << std::endl;
  if(distSquared == maxDistInCells*maxDistInCells)
  	std::cout<<"we hit a cell with d = dmax, distance value is clamped."<<std::endl;

  //retrieve closest occupied cell at a point
  IntPoint3D closest = distmap.getClosestObstacle(30,67,33);
  if(closest.x == DynamicEDT3D::invalidObstData)
  	std::cout<<"we hit a cell with d = dmax, no information about closest occupied cell."<<std::endl;
  else
  	std::cout<<"closest occupied cell to 30,67,33: "<< closest.x<<","<<closest.y<<","<<closest.z<<std::endl;

  return 0;
}
