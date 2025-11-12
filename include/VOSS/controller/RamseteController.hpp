#pragma once

#include <algorithm>
#include <memory>

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/pathing/BezierSpline.hpp"
#include "VOSS/pathing/MotionProfile.hpp"

namespace voss::controller
{

class RamseteController : public AbstractController
{
 protected:
  CubicBezierSpline<300> Spline;
  MotionProfile linearProfile;
  Pose lastTarget;
  double lead = 0.5;
  double endVelocity = 0.0;
  double maxAcceleration;
  double track_width = 0.0;
  double max_chassis_velocity = 68.0;

 public:
  RamseteController(std::shared_ptr<localizer::AbstractLocalizer> l);

  chassis::DiffChassisCommand get_command(
      bool reverse, bool thru, std::shared_ptr<AbstractExitCondition> ec) override
  {
    Pose current_pose = l->get_pose();
    Point current_point(current_pose.x, current_pose.y);
    Pose current_vel = l->get_velocity();

    if (this->target.x != this->lastTarget.x || this->target.y != this->lastTarget.y ||
        this->target.theta != this->lastTarget.theta)
    {
      double c = hypot(current_pose.x - target.x, current_pose.y - target.y) * this->lead;

      Point P1(
          cos(current_pose.theta.value()) * c + current_pose.x,
          sin(current_pose.theta.value()) * c + current_pose.y);
      Point P2(
          target.x - cos(current_pose.theta.value()) * c,
          target.y - sin(current_pose.theta.value()) * c);
      Point P3(target.x, target.y);

      this->Spline = CubicBezierSpline<300>(current_point, P1, P2, P3);

      double length = this->Spline.get_length(1.0);
      auto [max_v, max_w] = this->Spline.get_path_limits();

      double v_sign = (cos(current_pose.theta.value()) * current_vel.x +
                       sin(current_pose.theta.value()) * current_vel.y) >= 0
                          ? 1.0
                          : -1.0;
      double initial_v = v_sign * std::hypot(current_vel.x, current_vel.y);
      
      this->linearProfile = MotionProfile(initial_v, endVelocity, maxAcceleration, max_v, length);

      this->lastTarget = target;
    }

    auto [nearest_t, nearest_point] = this->Spline.get_nearest(current_point);
    double distance_traveled = this->Spline.get_length(std::clamp(nearest_t, 0.0, 1.0));
    double current_time = this->linearProfile.get_time(distance_traveled);

    double target_linear = this->linearProfile.get_velocity(current_time);
    double target_angular = target_linear * this->Spline.get_curvature(nearest_t);

    double left = target_linear - target_angular * track_width / 2.0;
    double right = target_linear + target_angular * track_width / 2.0;

    double ratio = std::max(fabs(left), fabs(right)) / this->max_chassis_velocity;
    if (ratio > 1.0)
    {
      left /= ratio;
      right /= ratio;
    }

    if (nearest_t >= 1) { return chassis::DiffChassisCommand{chassis::Stop{}}; }

    return chassis::DiffChassisCommand{chassis::diff_commands::WheelVelocities{left, right}};
  }

  chassis::DiffChassisCommand get_angular_command(
      bool reverse,
      bool thru,
      voss::AngularDirection direction,
      std::shared_ptr<AbstractExitCondition> ec) override;

  void reset() override;

  friend class RamseteControllerBuilder;
};

}  // namespace voss::controller