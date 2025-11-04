#pragma once

#include "VOSS/controller/AbstractController.hpp"
#include "VOSS/pathing/BezierSpline.hpp"

namespace voss::controller
{

class FollowVelocityPath : public AbstractController
{
 protected:
  CubicBezierSpline<1000> spline;
  double track_width;

 public:
  FollowVelocityPath(
      std::shared_ptr<localizer::AbstractLocalizer> l, CubicBezierSpline<1000> spline);

  chassis::DiffChassisCommand get_command(
      bool reverse, bool thru, std::shared_ptr<AbstractExitCondition> ec) override;

  chassis::DiffChassisCommand get_angular_command(
      bool reverse,
      bool thru,
      voss::AngularDirection direction,
      std::shared_ptr<AbstractExitCondition> ec) override;

  void reset() override {};
};

}  // namespace voss::controller