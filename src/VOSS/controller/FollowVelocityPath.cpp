#include "VOSS/controller/FollowVelocityPath.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#include "VOSS/chassis/ChassisCommand.hpp"
#include "VOSS/pathing/BezierSpline.hpp"
#include "VOSS/utils/Point.hpp"

namespace voss::controller
{

FollowVelocityPath::FollowVelocityPath(
    std::shared_ptr<localizer::AbstractLocalizer> l, CubicBezierSpline<1000> spline)
    : AbstractController(std::move(l)), spline(std::move(spline))
{
}

chassis::DiffChassisCommand FollowVelocityPath::get_command(
    bool reverse, bool thru, std::shared_ptr<AbstractExitCondition> ec)
{
  auto [t, nearest] = spline.get_nearest(this->l->get_position());

  Point velocity = spline.get_global_velocity(t);
  double linear_velocity = std::hypot(velocity.x, velocity.y);
  double angular_velocity = spline.get_angular_velocity(t);

  double left = linear_velocity - angular_velocity * track_width / 2.0;
  double right = linear_velocity + angular_velocity * track_width / 2.0;

  std::cout << t << ", " << linear_velocity << ", " << angular_velocity << std::endl;
  double ratio = std::max(fabs(left), fabs(right)) / 40.0;
  if (ratio > 1.0)
  {
    left /= ratio;
    right /= ratio;
  }

  if (t >= 1) { return chassis::DiffChassisCommand{chassis::Stop{}}; }

  return chassis::DiffChassisCommand{chassis::diff_commands::WheelVelocities{left, right}};
}

chassis::DiffChassisCommand FollowVelocityPath::get_angular_command(
    bool reverse,
    bool thru,
    voss::AngularDirection direction,
    std::shared_ptr<AbstractExitCondition> ec)
{
  return chassis::DiffChassisCommand{chassis::Stop{}};
}

}  // namespace voss::controller