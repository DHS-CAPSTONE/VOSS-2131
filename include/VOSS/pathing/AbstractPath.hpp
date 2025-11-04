#pragma once

#include <cmath>
#include <vector>

#include "VOSS/utils/Point.hpp"
#include "knn_search/kdtree.hpp"

namespace voss
{
class AbstractPath
{
 private:
  Kdtree::KdNodeVector nodes;
  mutable Kdtree::KdTree* tree = nullptr;

 protected:
  AbstractPath() = default;

  virtual ~AbstractPath()
  {
    if (tree) delete tree;
  }

  explicit AbstractPath(const std::vector<Point>& points) { set_points(points); }

  void set_points(const std::vector<Point>& points)
  {
    nodes.clear();
    for (size_t i = 0; i < points.size(); ++i)
    {
      Kdtree::CoordPoint coord_point = {points[i].x, points[i].y};
      nodes.push_back(Kdtree::KdNode(coord_point, nullptr, i));
    }

    if (tree)
    {
      delete tree;
      tree = nullptr;
    }
    tree = new Kdtree::KdTree(&nodes);
  }

  std::pair<double, Point> closest_point(const Point& p) const
  {
    const Kdtree::CoordPoint query = {p.x, p.y};
    Kdtree::KdNodeVector result;

    if (!tree)
    {
      // no points set; return zero t and a default point
      return {0.0, Point{0.0, 0.0}};
    }

    tree->k_nearest_neighbors(query, 1, &result);

    double index = static_cast<double>(result[0].index);
    Point closest_point = {result[0].point[0], result[0].point[1]};

    double t = (nodes.size() > 1) ? index / static_cast<double>(nodes.size() - 1) : 0.0;
    return {t, closest_point};
  }
};
}  // namespace voss