#pragma once

#include <cmath>
#include <vector>
#include <iostream>
namespace control {
namespace math_tool {
#define kMathEpsilon 1e-6
double NormalizeAngle(const double angle);
double NormalizeAngle_deeg(const double angle);
double DistanceSquare(double x1, double y1, double x2, double y2);
double InnerProd(double x1, double y1, double x2, double y2);
double CrossProd(double x1, double y1, double x2, double y2);
template <typename T>
inline T Square(const T value) {
  return value * value;
}

double lerp(const double &x0, const double t0, const double &x1, const double t1,
       const double t); 
double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t);

int sign(const double& x);

template <typename T>
inline T InterpolateUsingLinearApproximation(const T &p0,
    const T &p1, const double s) {
  double s0 = p0.s;
  double s1 = p1.s;
  // CHECK_LE(s0, s1);/

  T path_point;
  double weight = (s - s0) / (s1 - s0);
  double x = (1 - weight) * p0.x + weight * p1.x;
  double y = (1 - weight) * p0.y + weight * p1.y;
  double v = (1 - weight) * p0.v + weight * p1.v;
  double theta = math_tool::slerp(p0.theta, p0.s, p1.theta, p1.s, s);
  path_point.x = x;
  path_point.y = y;
  path_point.theta = theta;
  path_point.v = v;
  path_point.s = s;
  return path_point;
}


template <typename T>
bool GetPathPointWithPathS(const std::vector<T>& path_data,
  const double path_s, T* const path_point) {
  if (path_data.empty()) {
    // std::cout<<__FUNCTION__<<" , "<<__LINE__<<" Path data is empty!! "<<std::endl;
    return false;
  }

  auto comp = [](const T &tp, const double path_s) {
    return tp.s < path_s + 1e-6;
  };
  auto it_lower = std::lower_bound(path_data.begin(), path_data.end(), path_s,
                          comp);

  if (it_lower == path_data.begin()) {
    *path_point = path_data.front();

    return true;
  }
  if (it_lower == path_data.end()) {
    *path_point = path_data.back();
    return true;
  }
  *path_point = InterpolateUsingLinearApproximation(*(it_lower - 1),
    *it_lower, path_s);
  return true;
}

}  // namespace math_tool
}  // namespace control
