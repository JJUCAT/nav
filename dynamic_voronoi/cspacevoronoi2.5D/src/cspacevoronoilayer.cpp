#include "cspacevoronoilayer.h"
#include "point.h"
#include "float.h"

#define MAX(a,b) ((a)>(b) ? (a) : (b))
#define MIN(a,b) ((a)<(b) ? (a) : (b))

CSpaceVoronoiLayer::CSpaceVoronoiLayer(std::vector<RobotColumn> &_columns, double t) {
  
  columns = _columns;

  // assume rectangular column
  for (unsigned int col=0; col<columns.size(); col++) {
    RobotColumn c = columns[col];
    int numPoints = 4;
    double minY = DBL_MAX;
    numPoints++;
    std::vector<Point> points(numPoints);
    points[0] = Point(c.x1,c.y1);
    points[1] = Point(c.x2,c.y1);
    points[2] = Point(c.x2,c.y2);
    points[3] = Point(c.x1,c.y2);
    for (int i=0; i<numPoints-1; i++) {
      double nx = cos(t)*points[i].x - sin(t)*points[i].y;
      double ny = sin(t)*points[i].x + cos(t)*points[i].y;
      points[i].x = nx;
      points[i].y = ny;
      if (ny<minY) minY = ny;
    }
    points[4] = points[0];
  
    int dy = floor(minY);
    
    std::vector<ScanLine> boundaries;

    while (true) {
      bool hit = false;
      std::set<int> xPos;
      for (int i=0; i<numPoints-1; i++) {
        // line parallel or not?
        if (fabs(points[i+1].y - points[i].y)<0.0001 && ((int)floor(points[i+1].y))==dy) {
          hit = true;
          xPos.insert(floor(points[i+1].x));
          xPos.insert(floor(points[i].x));
        } else {
          double ddy = points[i+1].y - points[i].y;
          double ddx = points[i+1].x - points[i].x;
          double r = (dy-points[i].y)/ddy;
          if (r>=0 && r<=1) {
            hit = true;
            xPos.insert(floor(points[i].x + r*ddx));
          }

	  //also scan if the line at the top of the current row hits
	  r = (dy+0.999-points[i].y)/ddy;
	  if (r>=0 && r<=1) {
	    hit = true;
	    xPos.insert(floor(points[i].x + r*ddx));
	  }

	  //the polygon vertices might be sticking out (still not hitting the two lines checked above), so we check them separately
	  if(((int) floor(points[i].y)) == dy){
	    xPos.insert(floor(points[i].x));
	    hit = true;
	  }
        }
      }
      if (hit) {
        ScanLine l;
        l.dy = dy;
        l.x1 = *xPos.begin();
        l.x2 = *xPos.rbegin();
        l.upper = c.upper;
        l.lower = c.lower;
        boundaries.push_back(l);
      } else break;
      dy++;
    }

    //we need to account for the following:
    //when asking for positions later, x,y and orientation will be floored
    //therefore, the robot center can be anywhere in a cell, not just at the lower left corner

    //first we add 1 to every right end of a scanline (account for the fact that the robot cannot only be at the leftmost position in the cell, but anywhere to the right)
    for(unsigned int i=0; i<boundaries.size(); i++){
      boundaries[i].x2 += 1;
    }

    //now, we need to shift every scanline one up in y and merge with the line that is there (robot cannot only be at the bottom position in the cell, but anywhere above)

    //first, copy topmost line (this is at the end of the vector
    boundaries.push_back(boundaries.back());
    boundaries.back().dy +=1;
    
    //now, do the shift-merging
    for(unsigned int i=boundaries.size()-1; i>0; i--){
      boundaries[i].x1 = (std::min)(boundaries[i].x1, boundaries[i-1].x1);
      boundaries[i].x2 = (std::max)(boundaries[i].x2, boundaries[i-1].x2);
    }

    columnBoundaries.push_back(boundaries);
  }
  assert(columnBoundaries.size()==columns.size());
}

CSpaceVoronoiLayer::~CSpaceVoronoiLayer(){

}

void CSpaceVoronoiLayer::overlayFootprint(const CSpaceVoronoiLayer* other){
  assert(columnBoundaries.size() == other->columnBoundaries.size());

  for(unsigned int i=0; i<columnBoundaries.size(); i++){
    /*
    std::cerr<<"===NEW PART==="<<std::endl;
    std::cerr<<"my before "<<std::endl;
    for(unsigned int j=0; j<columnBoundaries[i].size(); j++){
      std::cerr<<"dy "<<columnBoundaries[i][j].dy<<" ["<<columnBoundaries[i][j].x1<<","<<columnBoundaries[i][j].x2<<"]"<<std::endl;
    }
    std::cerr<<"other before "<<std::endl;
    for(unsigned int j=0; j<other->columnBoundaries[i].size(); j++){
      std::cerr<<"dy "<<other->columnBoundaries[i][j].dy<<" ["<<other->columnBoundaries[i][j].x1<<","<<other->columnBoundaries[i][j].x2<<"]"<<std::endl;
    }
    */
    
    unsigned int idxOther=0;
    unsigned int idx=0;

    //in case the other scanlines start at lower dy, copy these lines to our scanlines
    while(idxOther < other->columnBoundaries[i].size() && other->columnBoundaries[i][idxOther].dy < columnBoundaries[i][idx].dy){
      idxOther++;
    }
    columnBoundaries[i].insert(columnBoundaries[i].begin(), other->columnBoundaries[i].begin(), other->columnBoundaries[i].begin()+idxOther);

    //in case we start at lower dy than the other, skip forward
    while(idx < columnBoundaries[i].size() && columnBoundaries[i][idx].dy < other->columnBoundaries[i][idxOther].dy){
      idx++;
    }

    while(idx<columnBoundaries[i].size() && idxOther<other->columnBoundaries[i].size()){
      assert(columnBoundaries[i][idx].dy == other->columnBoundaries[i][idxOther].dy);
      columnBoundaries[i][idx].x1 = (std::min)(columnBoundaries[i][idx].x1, other->columnBoundaries[i][idxOther].x1);
      columnBoundaries[i][idx].x2 = (std::max)(columnBoundaries[i][idx].x2, other->columnBoundaries[i][idxOther].x2);
      idx++;
      idxOther++;
    }
    
    if(idxOther < other->columnBoundaries[i].size()){
      columnBoundaries[i].insert(columnBoundaries[i].end(), other->columnBoundaries[i].begin()+idxOther, other->columnBoundaries[i].end());
    }

    /*
    std::cerr<<"my after "<<std::endl;
    for(unsigned int j=0; j<columnBoundaries[i].size(); j++){
      std::cerr<<"dy "<<columnBoundaries[i][j].dy<<" ["<<columnBoundaries[i][j].x1<<","<<columnBoundaries[i][j].x2<<"]"<<std::endl;
    }
    */
  }
}

void CSpaceVoronoiLayer::initializeMap(SimpleMap<int> *gridMap) {
  sizeX = gridMap->getMapSizeX();
  sizeY = gridMap->getMapSizeY();
  DynamicVoronoi::initializeEmpty(sizeX, sizeY, true);
  countMap.resize(sizeX, sizeY);
  for (int x=0; x<sizeX; x++) {
    for (int y=0; y<sizeY; y++) countMap.setCell(x,y,0);
  }

  for (int i=0; i<(int) columns.size(); i++) {
    for (int x=0; x<sizeX; x++) {
      for (int y=0; y<sizeY; y++) {
        int height = gridMap->getCell(x,y);
        if (height<columns[i].upper) {
          updateClearance(x,y,i,true);
        }
      }
    }
  }
}

void CSpaceVoronoiLayer::updateClearance(int x, int y, int col, bool newlyOccupied, bool updateVoronoi) {
    for (unsigned int k=0; k<columnBoundaries[col].size(); k++) {

      int ny = y-columnBoundaries[col][k].dy;
      if (ny<0 || ny>=sizeY) continue;
      int first = x-columnBoundaries[col][k].x2;
      int last =  x-columnBoundaries[col][k].x1;
      if (first>last) {
        int dummy = last;
        last = first;
        first = dummy;
      }
      first = MAX(first,0);
      last =  MIN(last,sizeX-1);
    
      if (newlyOccupied) {
        for (int nx=first; nx<=last; nx++) {
	  if((countMap.preIncrement(nx,ny) == 1) && updateVoronoi){
	    setObstacle(nx,ny);
	  }
        } 
      } else {
        for (int nx=first; nx<=last; nx++) {
	  if((countMap.preDecrement(nx,ny) == 0) && updateVoronoi){
	    removeObstacle(nx,ny);
	  }
        }
      }
    }
}

void CSpaceVoronoiLayer::saveCountMap(const char *filename) {
  countMap.writeToPGM(filename);
}


