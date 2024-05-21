#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/multi_point.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/ring.hpp>
#include <boost/geometry/geometries/segment.hpp>

#include "math.h"
#include "geometry.pb.h"

namespace geometry2 {

template <typename T>
struct Vec2 {
  T x, y;

  Vec2() : x(0), y(0) {}

  Vec2(T x, T y) : x(x), y(y) {}

  Vec2(const proto::Vector3d& p) : x(p.x()), y(p.y()) {}

  Vec2(const proto::Vector3f& p) : x(p.x()), y(p.y()) {}

  constexpr const T* asArray() const { return &x; }

  static constexpr Vec2 unit(T angle) { return {std::cos(angle), std::sin(angle)}; }

  constexpr Vec2 operator+(const Vec2& other) const { return {x + other.x, y + other.y}; }

  constexpr Vec2 operator-(const Vec2& other) const { return {x - other.x, y - other.y}; }

  constexpr Vec2 operator*(T scale) const { return {x * scale, y * scale}; }

  constexpr Vec2 operator/(T scale) const { return {x / scale, y / scale}; }

  constexpr bool operator==(const Vec2& other) const { return x == other.x && y == other.y; }

  constexpr T heading() const { return std::atan2(y, x); }

  constexpr T length() const { return hypotFast<T>(x, y); }

  constexpr T lengthL1() const { return abs(x) + abs(y); }

  constexpr T cross(const Vec2& other) const { return x * other.y - y * other.x; }

  constexpr T dot(const Vec2& other) const { return x * other.x + y * other.y; }

  constexpr T distance(const Vec2& other) const { return hypotFast<T>(x - other.x, y - other.y); }

  constexpr T distanceL1(const Vec2& other) const { return abs(x - other.x) + abs(y - other.y); }

  constexpr Vec2 normalize() const { return *this / length(); }

  // Rotates this vector by the angle specified by unit_vector.
  constexpr Vec2 rotate(Vec2 unit_vector) const {
    return {x * unit_vector.x - y * unit_vector.y, x * unit_vector.y + y * unit_vector.x};
  }

  constexpr Vec2 rotate(T angle) const { return rotate(unit(angle)); }
};

template <typename T>
struct Vec3{
    T x, y, z;

    Vec3() : x(0), y(0), z(0) {}

    Vec3(T x, T y, T z) : x(x), y(y), z(z){}

    Vec3(const proto::Vector3d& p) : x(p.x()), y(p.y()), z(p.z()) {}

    Vec3(const proto::Vector3f& p) : x(p.x()), y(p.y()), z(p.z()){}
};

using Vec2f = Vec2<float>;
using Vec2d = Vec2<double>;
static_assert(sizeof(Vec2f) == 8, "Incorrect size of Vec2f");
static_assert(sizeof(Vec2d) == 16, "Incorrect size of Vec2d");

using Vec3f = Vec3<float>;
using Vec3d = Vec3<double>;

static_assert(sizeof(Vec3f) == 12, "Incorrect size of Vec3f");
static_assert(sizeof(Vec3d) == 24, "Incorrect size of Vec3d");
} // namespace geometry2

BOOST_GEOMETRY_REGISTER_POINT_2D(::geometry2::Vec2f, float, cs::cartesian, x, y)

namespace geometry2 {

using Segment = ::boost::geometry::model::segment<Vec2f>;

using Points = ::boost::geometry::model::multi_point<Vec2f>;

// Axis aligned bounding box (AABB).
// The constructor takes the minimum corner point and the maximum corner point.
using Box = ::boost::geometry::model::box<Vec2f>;

// A piece-wise linear curve without self-crossing.
using Polyline = ::boost::geometry::model::linestring<Vec2f>;

// A simple polygon consisting of non-intersecting line segments. Points must be clockwise.
// Polygon is expected to have a close end, i.e., the last point should always repeat the first point.
using Polygon = ::boost::geometry::model::ring<Vec2f>;

// Copy-based adapters.
inline Polyline asPolyline(const proto::Polyline& polyline) {
  return {polyline.points().begin(), polyline.points().end()};
}

inline Polygon asPolygon(const proto::Polygon& polygon) {
  return {polygon.points().begin(), polygon.points().end()};
}

Box envelope(const Polygon& x);

// Returns left-, right-, down-, and up-most points
Points extremePoints(const Polygon& x);

float area(const Box& x);
float area(const Polygon& x);
float area(const proto::Polygon& x);

// Returns whether point is inside shape or on its boarder.
bool cover(const Polygon& shape, Vec2f point);

Polygon convexHull(const Points& points);

// Expand polygon with ±x and ±y. Returns a convex hull.
Polygon expand(const Polygon& polygon, float x, float y);

bool intersection(const Polyline& x, const Polyline& y, std::vector<Vec2f>* out = nullptr);
bool intersection(const Polyline& x, const Polygon& y, std::vector<Polyline>* out = nullptr);
bool intersection(const Polygon& x, const Polygon& y, std::vector<Polygon>* out = nullptr);

bool isValid(const Box& x);
bool isValid(const Polygon& x);
bool isValid(const Polyline& x);
bool isValid(const proto::Polygon& x);
bool isValid(const proto::Polyline& x);

float length(const Polyline& x);
float length(const proto::Polyline& x);
// Length of first n segments.
float lengthFirstN(const proto::Polyline& x, int n);

// Returns distance from p to the segment (s0, s1). "distance" is positive when p is on the left
// side of the segment. "along_segment" stores the distance from s0 to the closest point.
float signedDistance(Vec2f p, Vec2f s0, Vec2f s1, float* along_segment);

// Returns distance from p to path. "distance" is positive when p is on the left side of the path.
// "along_path" stores the distance traveled from path[0] to the closest point.
float signedDistance(Vec2f p, const Polyline& path, float* along_path, float* along_segment, int* index = nullptr);

float squaredDistance(Vec2f p, const Polygon& polygon);
float squaredDistance(Vec2f p, const Segment& s);
float squaredDistance(const Polygon& polygon, const Polyline& polyline);

// Convex hull of the union of x and y.
Polygon unionConvexHull(const Polygon& x, const Polygon& y);

// Ordinary union of x and y.
std::vector<Polygon> unionOrdinary(const Polygon& x, const Polygon& y);

} // namespace geometry2
