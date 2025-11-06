/**
 * @file BezierSpline.hpp
 * @author Andrew Hilton (2131N)
 * @brief Bezier Spline Paths
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

#include <array>
#include <vector>

#include "VOSS/pathing/AbstractPath.hpp"
#include "VOSS/pathing/AbstractSpline.hpp"
#include "VOSS/utils/Point.hpp"

namespace voss
{
template <size_t samples>
class CubicBezierSpline : public AbstractSpline, public AbstractPath
{
 private:
  const Point P0;
  const Point P1;
  const Point P2;
  const Point P3;

  std::array<std::pair<double, Point>, samples> path;

 public:
  CubicBezierSpline(const Point P0, const Point P1, const Point P2, const Point P3)
      : P0(P0), P1(P1), P2(P2), P3(P3), AbstractSpline()
  {
    this->fillPath();
  }

  Point get_point(double t) override
  {
    // Cubic Bezier: B(t) = (1-t)^3 P0 + 3(1-t)^2 t P1 + 3(1-t) t^2 P2 + t^3 P3
    double u = 1.0 - t;
    double b0 = u * u * u;
    double b1 = 3.0 * u * u * t;
    double b2 = 3.0 * u * t * t;
    double b3 = t * t * t;

    Point out;
    out.x = b0 * P0.x + b1 * P1.x + b2 * P2.x + b3 * P3.x;
    out.y = b0 * P0.y + b1 * P1.y + b2 * P2.y + b3 * P3.y;
    return out;
  }

  Point get_global_velocity(double t) override
  {
    // First derivative: B'(t) = 3(1-t)^2 (P1-P0) + 6(1-t)t (P2-P1) + 3 t^2 (P3-P2)
    double u = 1.0 - t;
    double v0 = 3.0 * u * u;
    double v1 = 6.0 * u * t;
    double v2 = 3.0 * t * t;

    Point out;
    out.x = v0 * (P1.x - P0.x) + v1 * (P2.x - P1.x) + v2 * (P3.x - P2.x);
    out.y = v0 * (P1.y - P0.y) + v1 * (P2.y - P1.y) + v2 * (P3.y - P2.y);
    return out;
  }

  Point get_global_acceleration(double t) override
  {
    // Second derivative: B''(t) = 6(1-t)(P2 - 2P1 + P0) + 6 t (P3 - 2P2 + P1)
    double u = 1.0 - t;
    double a0 = 6.0 * u;
    double a1 = 6.0 * t;

    Point term0{P2.x - 2.0 * P1.x + P0.x, P2.y - 2.0 * P1.y + P0.y};
    Point term1{P3.x - 2.0 * P2.x + P1.x, P3.y - 2.0 * P2.y + P1.y};

    Point out;
    out.x = a0 * term0.x + a1 * term1.x;
    out.y = a0 * term0.y + a1 * term1.y;
    return out;
  }

 private:
  void fillPath()
  {
    std::vector<Point> points;

    for (size_t i = 0; i < samples; ++i)
    {
      double t = (samples == 1) ? 0.0 : double(i) / double(samples - 1);
      path[i] = {t, get_point(t)};
      points.push_back(path[i].second);
    }

    this->set_points(points);
  }
};
}  // namespace voss