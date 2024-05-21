#pragma once

#include <algorithm>
#include <cmath>
// https://developers.redhat.com/blog/2016/02/29/why-cstdlib-is-more-complicated-than-you-might-think/
#include <cstdlib>
#include <type_traits>
#include <limits>

using ::std::abs;
using ::std::max;
using ::std::min;
using ::std::sqrt;

constexpr double kNanDouble = std::numeric_limits<double>::quiet_NaN();

constexpr float kNanFloat = std::numeric_limits<float>::quiet_NaN();

constexpr double kDegToRad = M_PI / 180.0;

constexpr double kRadToDeg = 180.0 / M_PI;

template <typename T>
T degToRad(T deg) {
  return T{kDegToRad} * deg;
}

template <typename T>
T radToDeg(T rad) {
  return T{kRadToDeg} * rad;
}

// Wraps an angle to [0, 2 * PI).
template <typename T>
T wrapAngle(T angle) {
  T r = fmod(angle, T{M_PI * 2});
  return r < T{0} ? r + T{M_PI * 2} : r;
}

// Normalizes an angle to [-PI, PI).
template <typename T>
T normalizeAngle(T angle) {
  T r = fmod(angle + T{M_PI}, T{M_PI * 2});
  return r < T{0} ? r + T{M_PI} : r - T{M_PI};
}

template <class T>
T clamp(const T& v, const T& lo, const T& hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

template <typename T>
T sqr(const T& x) {
  return x * x;
}

template <typename T>
T cubic(const T& x) {
  return x * x * x;
}

// Benchmark results for 65536 calls of:
//  rsqrtFast() sum: 510.508     Time:  294 us
//  1/sqrt() sum: 510.537        Time: 1373 us
//  sqrtFast() sum: 1.11846e+07  Time:  134 us
//  sqrt() sum: 1.11847e+07      Time:  289 us
// It's weird that sqrtFast() is even faster than rsqrtFast().

// Computes the approximation of 1 / sqrt(x).
// inline float rsqrtFast(float x) { return _mm_cvtss_f32(_mm_rsqrt_ss(_mm_set1_ps(x))); }

// Computes the approximation of sqrt(x).
// inline float sqrtFast(float x) { return rsqrtFast(x) * x; }

template <typename T>
int sgn(const T& x) {
  return int{T{0} < x} - int{x < T{0}};
}

// Benchmark results for 65536 calls of:
//  std::hypot sum: 3.0365e+09   Time: 784 us
//  hypotFast sum: 3.0365e+09    Time: 107 us
//  hypot3 sum: 3.71919e+09      Time: 271 us
//  hypot3Fast sum: 3.71919e+09  Time: 138 us

// Unsafe but faster version of std::hypot().
template <typename T>
T hypotFast(T x, T y) {
  return sqrt(sqr(x) + sqr(y));
}

// Computes the square root of the sum of the squares of x, y, and z, without
// overflow or underflow.
template <typename T>
T hypot3(T x, T y, T z) {
  x = abs(x);
  y = abs(y);
  z = abs(z);
  T a = max(max(x, y), z);
  return a * sqrt(sqr(x / a) + sqr(y / a) + sqr(z / a));
}

// Unsafe but faster version of hypot3().
template <typename T>
T hypot3Fast(T x, T y, T z) {
  return sqrt(sqr(x) + sqr(y) + sqr(z));
}

template <typename T>
constexpr bool isPowerOfTwo(T x) {
  static_assert(std::is_integral<T>::value, "T must be an integral type");
  return x != 0 && (x & (x - 1)) == 0;
}

template <class T, unsigned N>
unsigned arraySize(const T (&)[N]) {
  return N;
}
