#pragma once

#include <cmath>
#include "math_utils.h"

namespace control {
namespace math_tool {

double NormalizeAngle(const double angle) {
  double a = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (a < 0.0) {
    a += (2.0 * M_PI);
  }
  return a - M_PI;
}

double NormalizeAngle_deeg(const double angle) {
  double a = std::fmod(angle + 180.0, 2.0 * 180.0);
  if (a < 0.0) {
    a += (2.0 * 180.0);
  }
  return a - 180.0;
}

double DistanceSquare(double x1, double y1, double x2, double y2) {
  double x0 = x1 - x2;
  double y0 = y1 - y2;
  return (Square(x0) + Square(y0));
}


double InnerProd(double x1, double y1, double x2, double y2) {
  return (x1*x2 + y1*y2);
}

double CrossProd(double x1, double y1, double x2, double y2) {
  return (x1*y2 - x2*y1);
}


double lerp(const double &x0, const double t0, const double &x1, const double t1,
       const double t) {
  if (std::abs(t1 - t0) <= 1.0e-6) {
    // std::cout << "input time difference is too small" << std::endl;
    return x0;
  }
  const double r = (t - t0) / (t1 - t0);
  const double x = x0 + r * (x1 - x0);
  return x;
}



double slerp(const double a0, const double t0, const double a1, const double t1,
             const double t) {
  if (std::abs(t1 - t0) <= kMathEpsilon) {
    // std::cout << "input time difference is too small" << std::endl;
    return NormalizeAngle_deeg(a0);
  }
  const double a0_n = NormalizeAngle_deeg(a0);
  const double a1_n = NormalizeAngle_deeg(a1);
  double d = a1_n - a0_n;
  if (d > M_PI) {
    d = d - 2 * M_PI;
  } else if (d < -M_PI) {
    d = d + 2 * M_PI;
  }

  const double r = (t - t0) / (t1 - t0);
  const double a = a0_n + d * r;
  return NormalizeAngle_deeg(a);
}

int sign(const double& x)
{
  if(x>1e-6)
    return 1;
  else if(x<-1e-6)//一定要是==
    return -1;
  else
    return 0;
}


}  // namespace math_tool
}  // namespace control