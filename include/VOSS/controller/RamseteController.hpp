#pragma once

#include <memory>

#include "VOSS/controller/AbstractController.hpp"

namespace voss::controller
{

class RamseteController : public AbstractController
{
 protected:
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

  std::shared_ptr<RamseteController> modify_linear_constants(double kP, double kI, double kD);
  std::shared_ptr<RamseteController> modify_angular_constants(double kP, double kI, double kD);
  std::shared_ptr<RamseteController> modify_min_error(double min_error);

  friend class RamseteControllerBuilder;
  friend class BoomerangControllerBuilder;
};

}  // namespace voss::controller