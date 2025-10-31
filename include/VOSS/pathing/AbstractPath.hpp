#pragma once
#include <array>
#include <cmath>
#include <vector>

#include "Nanoflann/nanoflann.hpp"
#include "VOSS/utils/Point.hpp"
namespace voss
{
class AbstractPath
{
 public:
  using DoublePoint = std::array<double, 2>;

 private:
  struct PathPoints
  {
    std::vector<DoublePoint> pts;

    inline size_t kdtree_get_point_count() const { return pts.size(); }
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const { return pts[idx][dim]; }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const
    {
      return false;
    }
  };

  using KDTree = nanoflann::
      KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, PathPoints>, PathPoints, 2>;

  PathPoints cloud_;
  std::unique_ptr<KDTree> index_;

 protected:
  AbstractPath() = default;

  explicit AbstractPath(const std::vector<DoublePoint>& points) { set_points(points); }

  void set_points(const std::vector<DoublePoint>& points)
  {
    cloud_.pts = points;
    index_ = std::make_unique<KDTree>(2, cloud_, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index_->buildIndex();
  }

  /// Find nearest vertex on the path (returns index)
  size_t closest_point_index(const Point& p, double* out_dist_sqr = nullptr) const
  {
    if (!index_) return 0;
    const double query_pt[2] = {p.x, p.y};
    typename KDTree::IndexType ret_index = 0;
    double out_dist_sqr_local = 0.0;

    index_->knnSearch(query_pt, 1, &ret_index, &out_dist_sqr_local);
    if (out_dist_sqr) *out_dist_sqr = out_dist_sqr_local;
    return static_cast<size_t>(ret_index);
  }

  /// Return nearest point (distance + point)
  std::pair<double, Point> closest_point(const Point& p) const
  {
    double dist_sqr;
    size_t idx = closest_point_index(p, &dist_sqr);
    return {std::sqrt(dist_sqr), Point{cloud_.pts[idx][0], cloud_.pts[idx][1]}};
  }
};
}  // namespace voss