#ifndef _VOROPOINT_H_
#define _VOROPOINT_H_

#define INTPOINT IntPoint
#define INTPOINT3D IntPoint3D

/*! A light-weight integer point with fields x,y */
class IntPoint {
public:
  IntPoint() : x(0), y(0) {}
  IntPoint(int _x, int _y) : x(_x), y(_y) {}
  int x,y;
};

/*! A light-weight integer point with fields x,y,z */
class IntPoint3D {
public:
  IntPoint3D() : x(0), y(0), z(0) {}
  IntPoint3D(int _x, int _y, int _z) : x(_x), y(_y), z(_z) {}
  int x,y,z;
};


#endif
