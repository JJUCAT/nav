#include "cspacevoronoi.h"

#include <fstream>
#include <time.h>


int main( int argc, char* argv[] ) {

	if(argc!=4) {
		std::cerr<<"usage: "<<argv[0]<<" <pgm map> <length> <width>\n";
		exit(-1);
	}
	std::ifstream is(argv[1]);
	if (!is) {
		std::cerr << "Could not open map file for reading.\n";
		exit(-1);
	}

	//create robot model "wheelchair"
	//note that heigt is given in number of layers
	int length = atoi(argv[2]);
	int width = atoi(argv[3]);
	std::vector<RobotColumn> columns;
	RobotColumn col;
	col.lower = 0;
	col.upper = 1;
	col.x1 = 0;
	col.x2 = (double)length/2.0;
	col.y1 = -(double)width/2.0;
	col.y2 = (double)width/2.0;
	columns.push_back(col);

	col.upper = 2;
	col.x1 = -(double)length/2.0;
	col.x2 = 0;
	columns.push_back(col);



	SimpleMap<int> map;
	map.loadFromPGM(is);
	is.close();

	int sizeX = map.getMapSizeX();
	int sizeY = map.getMapSizeY();

	// turn map into a height map
	for (int x=0; x<sizeX; x++) {
		for (int y=0; y<sizeY; y++) {
			if (map.getCell(x,y)>0) map.setCell(x,y,0); // totally occupied
			else map.setCell(x,y,INT_MAX);
		}
	}


	fprintf(stderr, "Map loaded (%dx%d).\n",  map.getMapSizeX(), map.getMapSizeY());


	// DYNAMIC CSPACE
	CSpaceVoronoi *cspace = new CSpaceVoronoi(columns, &map);
	cspace->update();
	cspace->prune();

	//compute a path
	IntPose start;
	start.x = 30;
	start.y = 30;
	start.theta = 0;
	IntPose goal;
	goal.x = 30;
	goal.y = 220;
	goal.theta = 0;
	Pose tmp;
	tmp.x = start.x;
	tmp.y = start.y;
	tmp.theta = start.theta;
	std::cout<<"robot collides at start? "<<start.x<<","<<start.y<<","<<start.theta<<": "<<(cspace->checkCollision(tmp)?"yes":"no")<<std::endl;

	tmp.x = goal.x;
	tmp.y = goal.y;
	tmp.theta = goal.theta;
	std::cout<<"robot collides at goal? "<<goal.x<<","<<goal.y<<","<<goal.theta<<": "<<(cspace->checkCollision(tmp)?"yes":"no")<<std::endl;

	std::list<IntPose>* path = cspace->computeShortestPath(start, goal);
	std::cout<<"path:"<<std::endl;
	if(path!=NULL){
	  std::cerr<<path->size()<<" cells"<<std::endl;
	  for(std::list<IntPose>::iterator it=path->begin(); it!=path->end(); ++it){
	    std::cout<<it->x<<","<<it->y<<","<<it->theta<<std::endl;
	  }
	} else {
	  std::cerr<<"NULL"<<std::endl;
	}


	// generate some obstacles
	std::vector<IntPose> obstacles;
	srand(0);
	for(int i=0; i<5; i++){
		int x = 2 + (sizeX-2)*((double)rand()/(double)RAND_MAX);
		int y = 2 + (sizeY-2)*((double)rand()/(double)RAND_MAX);
		obstacles.push_back(IntPose(x,y,0));
	}
	//update with new obstacles
	cspace->updateObstacles(&obstacles, false);


	// do a collision check
	int r = length;
	if (width>r) r = width;
	double x = r+2+(sizeX-r-2)*((double)rand()/(double)RAND_MAX);
	double y = r+2+(sizeY-r-2)*((double)rand()/(double)RAND_MAX);
	double t = 2*M_PI*((double)rand()/(double)RAND_MAX);
	Pose checkme(x,y,t);

	std::cout<<"robot collides at "<<checkme.x<<","<<checkme.y<<","<<checkme.theta<<": "<<(cspace->checkCollision(checkme)?"yes":"no")<<std::endl;

	if(sizeX > 60 && sizeY > 90){
		checkme.x = 52;
		checkme.y = 80;
		std::cout<<"robot collides at "<<checkme.x<<","<<checkme.y<<","<<checkme.theta<<": "<<(cspace->checkCollision(checkme)?"yes":"no")<<std::endl;
	}

	//some visualization
	cspace->layers[0]->saveCountMap("countmap_0.ppm");
	cspace->layers[3]->saveCountMap("countmap_3.ppm");
	cspace->layers[6]->saveCountMap("countmap_6.ppm");
	cspace->layers[9]->saveCountMap("countmap_9.ppm");

	delete cspace;
  MemoryManager<Node>::releaseFreeObjects();

	return 0;

}
