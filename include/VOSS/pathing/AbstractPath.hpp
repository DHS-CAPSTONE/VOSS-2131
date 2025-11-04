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

 protected:
  AbstractPath() = default;

  explicit AbstractPath(const std::vector<Point>& points) { set_points(points); }

  void set_points(const std::vector<Point>& points)
  {
    path_points.clear();
    for (auto& point : points) { path_points.push_back(point); }
  }

  std::pair<double, Point> closest_point(const Point& p) const
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
};
}  // namespace voss