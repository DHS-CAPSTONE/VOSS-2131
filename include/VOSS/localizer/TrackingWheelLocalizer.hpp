#pragma once

#include <atomic>
#include <memory>

#include "VOSS/localizer/AbstractLocalizer.hpp"
#include "VOSS/localizer/AbstractTrackingWheel.hpp"
#include "pros/imu.hpp"

namespace voss::localizer
{

class TrackingWheelLocalizer : public AbstractLocalizer
{
 protected:
  std::atomic<double> prev_left_pos, prev_right_pos, prev_middle_pos;
  AtomicPose prev_pose;
  AtomicPose real_pose;
  Pose local_offset = {0, 0, 0};

  std::atomic<double> left_right_dist, middle_dist;
  std::unique_ptr<AbstractTrackingWheel> left_tracking_wheel, right_tracking_wheel,
      middle_tracking_wheel;
  std::unique_ptr<pros::IMU> imu;

 public:
  TrackingWheelLocalizer(
      std::unique_ptr<AbstractTrackingWheel> left,
      std::unique_ptr<AbstractTrackingWheel> right,
      std::unique_ptr<AbstractTrackingWheel> middle,
      std::unique_ptr<pros::IMU> imu,
      double left_right_dist,
      double middle_dist);
  void update() override;
  void calibrate() override;
  void set_pose(Pose pose) override;
  void set_pose(double x, double y, double theta) override;
  void set_local_offset(Pose local_offset);

  friend class TrackingWheelLocalizerBuilder;
};

}  // namespace voss::localizer