#pragma once

#include <cmath>
#include <vector>

#include "VOSS/utils/Point.hpp"

namespace voss
{
class AbstractPath
{
 private:
  std::vector<Point> path_points;
  double length = 0.0;

 protected:
  AbstractPath() = default;

  explicit AbstractPath(const std::vector<Point>& points) { set_points(points); }

  void set_points(const std::vector<Point>& points)
  {
    path_points.clear();
    for (auto& point : points) { path_points.push_back(point); }
    for (size_t i = 1; i < path_points.size(); i++)
    {
      length += std::hypot(
          path_points[i].x - path_points[i - 1].x, path_points[i].y - path_points[i - 1].y);
    }
  }

 public:
  virtual std::pair<double, Point> get_nearest(const Point& p) const
  {
    double dist = std::numeric_limits<double>::infinity();
    Point closest_point;
    double t = -1.0;

    for (size_t i = 0; i < path_points.size(); i++)
    {
      double cur_dist = std::hypot(p.x - path_points[i].x, p.y - path_points[i].y);
      if (cur_dist < dist)
      {
        dist = cur_dist;
        closest_point = path_points[i];
        t = i / static_cast<double>(path_points.size() - 1);
      }
    }

    return {t, closest_point};
  }

  virtual double get_length(double t) const
  {
    if (t == 1.0) { return length; }

    double length = 0.0;
    size_t end = static_cast<size_t>(path_points.size() * t);

    for (size_t i = 1; i < end; i++)
    {
      length += std::hypot(
          path_points[i].x - path_points[i - 1].x, path_points[i].y - path_points[i - 1].y);
    }
    return length;
  }
};
}  // namespace voss