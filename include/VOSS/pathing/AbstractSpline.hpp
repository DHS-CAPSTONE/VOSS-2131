/**
 * @file AbstractSpline.hpp
 * @author Andrew Hilton (2131N)
 * @brief Abstract Spline Pathing Class
 * @version 0.1
 * @date 2025-10-29
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <utility>

#include "VOSS/utils/Point.hpp"

namespace voss
{
class AbstractSpline
{
 protected:
  double max_path_w;
  double max_path_v;
  mutable bool path_limits_computed = false;

 public:
  AbstractSpline() : max_path_w(0.0), max_path_v(0.0), path_limits_computed(false) {}

  virtual Point get_point(double t) = 0;
  virtual Point get_global_velocity(double t) = 0;
  virtual Point get_global_acceleration(double t) = 0;

  virtual double get_curvature(double t)
  {
    Point velocity = get_global_velocity(t);
    Point acceleration = get_global_acceleration(t);

    double num = velocity.x * acceleration.y - velocity.y * acceleration.x;
    double denom = std::pow(velocity.x * velocity.x + velocity.y * velocity.y, 1.5);
    if (denom == 0.0) return 0.0;  // avoid divide-by-zero
    return num / denom;
  }

  virtual double get_heading(double t)
  {
    Point velocity = get_global_velocity(t);
    return atan2(velocity.y, velocity.x);
  }

  virtual double get_angular_velocity(double t)
  {
    Point velocity = get_global_velocity(t);

    return get_curvature(t) * std::hypot(velocity.x, velocity.y);
  }

  virtual double get_angular_acceleration(double t)
  {
    Point acceleration = get_global_acceleration(t);

    return get_curvature(t) * std::hypot(acceleration.x, acceleration.y);
  }

  virtual std::pair<double, double> get_greatest_angular_velocity(const int steps = 100)
  {
    double max_ang_vel = 0.0;
    double max_ang_t = 0.0;
    for (int i = 0; i <= steps; ++i)
    {
      double t = static_cast<double>(i) / steps;
      double ang_vel = fabs(get_angular_velocity(t));
      if (ang_vel > max_ang_vel)
      {
        max_ang_vel = ang_vel;
        max_ang_t = t;
      }
    }
    return {max_ang_t, max_ang_vel};
  }

  virtual std::pair<double, double> get_greatest_linear_velocity(const int steps = 100)
  {
    double max_lin_vel = 0.0;
    double max_lin_t = 0.0;
    for (int i = 0; i <= steps; ++i)
    {
      double t = static_cast<double>(i) / steps;
      Point vel = get_global_velocity(t);
      double lin_vel = fabs(std::hypot(vel.x, vel.y));
      if (lin_vel > max_lin_vel)
      {
        max_lin_vel = lin_vel;
        max_lin_t = t;
      }
    }
    return {max_lin_t, max_lin_vel};
  }

  std::pair<double, double> get_path_limits()
  {
    if (!path_limits_computed)
    {
      auto [t, max_w] = get_greatest_angular_velocity();
      this->max_path_w = max_w;
      
      auto [t2, max_v] = get_greatest_linear_velocity();
      this->max_path_v = max_v;
      path_limits_computed = true;
    }
    return {this->max_path_v, this->max_path_w};
  }
};
}  // namespace voss