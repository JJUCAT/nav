//
// Created by yjh on 23-2-24.
//
#include "geometry2.h"

#include <boost/geometry.hpp>
#include <cassert>

namespace geometry2 {
namespace {
constexpr float kEpsilon = 1.0e-3f;
constexpr float kLargeNumber = 1.0e30f;
}  // namespace

namespace bg = ::boost::geometry;

using std::abs;

struct Point {
    double x, y;
};

float area(const Box& x) { return bg::area(x); }

float area(const Polygon& x) { return bg::area(x); }

float area(const proto::Polygon& x) { return bg::area(asPolygon(x)); }

bool cover(const Polygon& shape, Vec2f point) { return bg::covered_by(point, shape); }

Polygon convexHull(const Points& points) {
  Polygon hull;
  bg::convex_hull(points, hull);
  return hull;
}

Polygon expand(const Polygon& polygon, float x, float y) {
  Polygon out;
  Points points;
  points.reserve((polygon.size() - 1) * 5);
  for (auto p = polygon.begin() + 1; p != polygon.end(); ++p) {
    points.push_back(*p);
    points.push_back(*p + Vec2f{x, 0.0f});
    points.push_back(*p + Vec2f{-x, 0.0f});
    points.push_back(*p + Vec2f{0.0f, y});
    points.push_back(*p + Vec2f{0.0f, -y});
  }
  bg::convex_hull(points, out);
  return out;
}

float signedDistance(Vec2f p, Vec2f s0, Vec2f s1, float* along_segment) {
  assert(along_segment != nullptr);
  Vec2f s = s1 - s0;
  Vec2f t = p - s0;
  float s_len = s.length();
  if (s_len < kEpsilon) {
    *along_segment = 0.0f;
    return t.length();
  }
  float distance = -s.cross(t) / s_len;
  float projected_length = s.dot(t) / s_len;
  if (projected_length < 0) {
    *along_segment = 0.0f;
    return std::copysign(t.length(), distance);
  } else if (projected_length > s_len) {
    *along_segment = s_len;
    return std::copysign((p - s1).length(), distance);
  } else {
    *along_segment = projected_length;
    return distance;
  }
}

float signedDistance(Vec2f p, const Polyline& path, float* along_path, float* along_segment, int* index) {
  if (path.empty()) return kNanFloat;

  assert(along_path != nullptr);
  if (path.size() == 1) {
    *along_path = 0.0f;
    return p.distance(path[0]);
  }
  float min_distance = kLargeNumber;
  float min_signed_distance = kNanFloat, min_along_path = kNanFloat;
  float length = 0.0;
  int min_index = 0;
  for (size_t i = 1; i < path.size(); ++i) {
    float projected_length;
    float signed_distance = signedDistance(p, path[i - 1], path[i], &projected_length);
    float distance = abs(signed_distance);
    if (distance < min_distance || (distance == min_distance && (p - path[i - 1]).dot(path[i] - path[i - 1]) == 0.0f)) {
      min_distance = distance;
      min_signed_distance = signed_distance;
      min_along_path = length + projected_length;
      *along_segment = projected_length;
      min_index = i;
    }
    length += path[i - 1].distance(path[i]);
  }
  *along_path = min_along_path;
  if (index != nullptr) *index = min_index;
  return min_signed_distance;
}

float squaredDistance(Vec2f p, const Polygon& polygon) {
  return bg::comparable_distance(p, polygon);
}

float squaredDistance(Vec2f p, const Segment& s) { return bg::comparable_distance(p, s); }

float squaredDistance(const Polygon& polygon, const Polyline& polyline) {
  return bg::comparable_distance(polyline, polygon);
}

bool intersection(const Polyline& x, const Polyline& y, std::vector<Vec2f>* out) {
  if (out == nullptr) return bg::intersects(x, y);
  out->clear();
  bg::intersection(x, y, *out);
  return !out->empty();
}

bool intersection(const Polyline& x, const Polygon& y, std::vector<Polyline>* out) {
  if (out == nullptr) return bg::intersects(x, y);
  out->clear();
  bg::intersection(x, y, *out);
  return !out->empty();
}

bool intersection(const Polygon& x, const Polygon& y, std::vector<Polygon>* out) {
  if (out == nullptr) return bg::intersects(x, y);
  out->clear();
  bg::intersection(x, y, *out);
  return !out->empty();
}

Box envelope(const Polygon& x) { return bg::return_envelope<Box, Polygon>(x); }

Points extremePoints(const Polygon& x) {
  Points extremes;
  extremes.resize(4);
  float left = kLargeNumber;
  float right = -kLargeNumber;
  float down = kLargeNumber;
  float up = -kLargeNumber;
  for (const Vec2f& p : x) {
    if (p.x < left) {
      left = p.x;
      extremes[0] = p;
    }
    if (p.x > right) {
      right = p.x;
      extremes[1] = p;
    }
    if (p.y < down) {
      down = p.y;
      extremes[2] = p;
    }
    if (p.y > up) {
      up = p.y;
      extremes[3] = p;
    }
  }
  return extremes;
}

Polygon unionConvexHull(const Polygon& x, const Polygon& y) {
  Polygon out;
  if (x.empty()) {
    bg::convex_hull(y, out);
  } else if (y.empty()) {
    bg::convex_hull(x, out);
  } else {
    Points p;
    p.reserve(x.size() + y.size() - 2);
    p.insert(p.end(), x.begin(), x.end() - 1);
    p.insert(p.end(), y.begin(), y.end() - 1);
    bg::convex_hull(p, out);
  }
  return out;
}

std::vector<Polygon> unionOrdinary(const Polygon& x, const Polygon& y) {
  std::vector<Polygon> out;
  bg::union_(x, y, out);
  return out;
}

bool isValid(const Box& x) { return bg::is_valid(x); }

bool isValid(const Polygon& x) { return bg::is_valid(x); }

bool isValid(const Polyline& x) { return bg::is_valid(x); }

bool isValid(const proto::Polygon& x) { return bg::is_valid(asPolygon(x)); }

bool isValid(const proto::Polyline& x) { return bg::is_valid(asPolyline(x)); }

float length(const Polyline& x) { return bg::length(x); }

float length(const proto::Polyline& x) { return lengthFirstN(x, x.points_size()); }

float lengthFirstN(const proto::Polyline& x, int n) {
  float l = 0.0f;
  for (int i = std::min(n, x.points_size() - 1); i > 0; --i) {
    l += Vec2f{x.points(i - 1)}.distance(x.points(i));
  }
  return l;
}
namespace linetools {
// compute distance
double distance(Vec2f p1, Vec2f p2) {
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
}
// 计算两条线段的交点
bool getIntersection(Vec2f p1, Vec2f p2, Vec2f p3, Vec2f p4, Vec2f& intersection) {
    double a1 = p2.y - p1.y;
    double b1 = p1.x - p2.x;
    double c1 = p2.x * p1.y - p1.x * p2.y;
    double a2 = p4.y - p3.y;
    double b2 = p3.x - p4.x;
    double c2 = p4.x * p3.y - p3.x * p4.y;
    double d = a1 * b2 - a2 * b1;
    if (d == 0) {
        return false;
    }
    intersection.x = (b1 * c2 - b2 * c1) / d;
    intersection.y = (a2 * c1 - a1 * c2) / d;
    if (intersection.x < min(p1.x, p2.x) || intersection.x > max(p1.x, p2.x)) {
        return false;
    }
    if (intersection.x < min(p3.x, p4.x) || intersection.x > max(p3.x, p4.x)) {
        return false;
    }
    return true;
}

bool pointInPolygon(Vec2f point, std::vector<Vec2f> polygon) {
    int count = 0;
    for (int i = 0; i < polygon.size(); i++) {
        Vec2f p1 = polygon[i];
        Vec2f p2 = polygon[(i+1)%polygon.size()];
        if ((p1.y <= point.y && p2.y > point.y) || (p1.y > point.y && p2.y <= point.y)) {
            Vec2f intersection;
            if (getIntersection(p1, p2, point, Vec2f{point.x+10000, point.y}, intersection)) {
                if (intersection.x > point.x) {
                    count++;
                }
            }
        }
    }
    return (count % 2 == 1);
}
std::vector<Vec2f> mergePolygons(std::vector<std::vector<Vec2f>> polygons) {
    std::vector<Vec2f> mergedPolygon;
    for (int i = 0; i < polygons.size(); i++) {
        for (int j = 0; j < polygons[i].size(); j++) {
            Vec2f p1 = polygons[i][j];
            Vec2f p2 = polygons[i][(j+1)%polygons[i].size()];
            Vec2f intersection;
            bool isIntersection = false;
            for (int k = i+1; k < polygons.size(); k++) {
                for (int l = 0; l < polygons[k].size(); l++) {
                    Vec2f p3 = polygons[k][l];
                    Vec2f p4 = polygons[k][(l+1)%polygons[k].size()];
                    if (getIntersection(p1, p2, p3, p4, intersection)) {
                        mergedPolygon.push_back(intersection);
                        isIntersection = true;
                        break;
                    }
                }
                if (isIntersection) {
                    break;
                }
            }
            if (!isIntersection) {
                mergedPolygon.push_back(p1);
            }
        }
    }
    return mergedPolygon;
}
std::vector<Vec2f> generatePolygon(std::vector<std::vector<Vec2f>> segments, double distance) {
    std::vector<std::vector<Vec2f>> polygons;
    for (int i = 0; i < segments.size(); i++) {
        Vec2f p1 = segments[i][0];
        Vec2f p2 = segments[i][1];
        double k = (p2.y - p1.y) / (p2.x - p1.x);
        double b = p1.y - k * p1.x;
        double d1 = distance * sqrt(1 + k * k);
        double xl = p1.x - d1;
        double xr = p1.x + d1;
        double yl = k * xl + b;
        double yr = k * xr + b;
        double xu = xl + distance * sqrt(1 + 1 / (k * k));
        double xd = xr - distance * sqrt(1 + 1 / (k * k));
        double yu = k * xu + b + distance;
        double yd = k * xd + b - distance;
        std::vector<Vec2f> polygon;
        polygon.push_back(Vec2f{xl, yl});
        polygon.push_back(Vec2f{xu, yu});
        polygon.push_back(Vec2f{xr, yr});
        polygon.push_back(Vec2f{xd, yd});
        polygons.push_back(polygon);
    }
    std::vector<Vec2f> mergedPolygon = mergePolygons(polygons);
    std::vector<Vec2f> polygon;
    for (int i = 0; i < mergedPolygon.size(); i++) {
        if (pointInPolygon(mergedPolygon[i], polygons[0])) {
            polygon.push_back(mergedPolygon[i]);
        }
    }
    return polygon;
}

}
}  // namespace geometry2
