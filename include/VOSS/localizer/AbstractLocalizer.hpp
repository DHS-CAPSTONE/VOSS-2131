#pragma once

#include "VOSS/utils/Point.hpp"
#include "VOSS/utils/Pose.hpp"
#include "pros/rtos.hpp"

namespace voss::localizer
{

class AbstractLocalizer
{
 protected:
  pros::Mutex mtx;
  AtomicPose pose;
  AtomicPose prev_pose;
  Pose local_velocity = {0.0, 0.0, 0.0};

  uint32_t last_timestamp = 0;

 public:
  AbstractLocalizer();

  virtual void update() = 0;
  void begin_localization();

  virtual void set_pose(Pose pose);
  virtual void set_pose(double x, double y, double theta);

  Pose get_pose();
  Pose get_velocity();
  double get_x();
  double get_y();
  double get_orientation_rad();
  double get_orientation_deg();
  Point get_position();
  void wait_until_near(Point target, double tolerance);
  void wait_until_distance(double distance);
  virtual void calibrate() = 0;
};

}  // namespace voss::localizer