#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint

#include <math.h>


// A light-weight integer point with fields x,y 
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

class Point {
  public:
    Point() {};
    Point(double _x, double _y) {
      x = _x;
      y = _y;
    }
    void rotate(double angle) {
      double nx = cos(angle)*x - sin(angle)*y;
      double ny = sin(angle)*x + cos(angle)*y;
      x = nx;
      y = ny;
    }
    double x,y;
  };


class Pose {
public:
    Pose() {};
  Pose(double _x, double _y, double _t) {
      x = _x;
      y = _y;
      theta = _t;
  }
  bool operator==(Pose &other) {
    return (x==other.x && y==other.y && theta==other.theta);
  }
  double x,y,theta;
};

class IntPose {
public:
  IntPose() : x(0), y(0), theta(0) {};
  IntPose(int _x, int _y, int _t) {
      x = _x;
      y = _y;
      theta = _t;
  }
  bool operator==(IntPose const& other) {
    return (x==other.x && y==other.y && theta==other.theta);
  }
  int x,y,theta;
};


#endif
