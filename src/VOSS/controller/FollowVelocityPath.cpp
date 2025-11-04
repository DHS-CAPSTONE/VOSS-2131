#include "VOSS/controller/FollowVelocityPath.hpp"

#include <cmath>
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

  return chassis::DiffChassisCommand{chassis::diff_commands::WheelVelocities{
      linear_velocity - angular_velocity * track_width / 2.0,
      linear_velocity + angular_velocity * track_width / 2.0}};
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