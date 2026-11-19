#pragma once

#include <array>
#include <cmath>
#include <algorithm>

using std::sqrt;
using std::sin;
using std::cos;
using std::tan;
using std::asin;
using std::acos;
using std::atan2;

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

// Row-major 3x3 matrix: m[row][col]
using Mat3 = std::array<std::array<double, 3>, 3>;

constexpr Vec3 UNIT_K = {0.0, 0.0, 1.0};
constexpr double PI = 3.14159265358979323846;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double DEG_TO_RAD = PI / 180.0;

double safeAcos(double x);
Vec3 cross(const Vec3& a, const Vec3& b);
double dot(const Vec3& a, const Vec3& b);
double norm(const Vec3& a);
Vec3 scale(const Vec3& a, double s);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 matVec(const Mat3& m, const Vec3& v);
Mat3 RYRX(const Vec3 &N);
