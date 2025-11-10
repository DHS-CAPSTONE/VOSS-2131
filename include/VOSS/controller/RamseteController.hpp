#pragma once

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
  MotionProfile AngularMotionProfile;

 public:
  RamseteController(std::shared_ptr<localizer::AbstractLocalizer> l);

  chassis::DiffChassisCommand get_command(
      bool reverse, bool thru, std::shared_ptr<AbstractExitCondition> ec) override;
  chassis::DiffChassisCommand get_angular_command(
      bool reverse,
      bool thru,
      voss::AngularDirection direction,
      std::shared_ptr<AbstractExitCondition> ec) override;

  void reset() override;

  friend class RamseteControllerBuilder;
};

}  // namespace voss::controller