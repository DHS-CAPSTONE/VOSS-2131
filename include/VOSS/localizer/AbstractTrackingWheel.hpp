#pragma once

#include <atomic>

namespace voss::localizer
{

class AbstractTrackingWheel
{
 protected:
  AbstractTrackingWheel();
  std::atomic<double> tpi;
  virtual double get_raw_position() = 0;

 public:
  double get_dist_traveled();
  void set_tpi(double new_tpi);
  virtual void reset() = 0;
};

}  // namespace voss::localizer