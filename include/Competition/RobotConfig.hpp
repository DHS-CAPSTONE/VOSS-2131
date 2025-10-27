#pragma once

#include <memory>

#include "VOSS/chassis/DiffChassis.hpp"
#include "VOSS/controller/ArcPIDController.hpp"
#include "VOSS/controller/BoomerangController.hpp"
#include "VOSS/controller/PIDController.hpp"
#include "VOSS/controller/SwingController.hpp"
#include "VOSS/exit_conditions/ExitConditions.hpp"
#include "VOSS/localizer/IMELocalizer.hpp"
#include "VOSS/selector/Selector.hpp"
#include "pros/misc.hpp"

extern voss::chassis::DiffChassis chassis;
extern std::shared_ptr<voss::localizer::IMELocalizer> odom;
extern std::shared_ptr<voss::controller::PIDController> pid;
extern std::shared_ptr<voss::controller::BoomerangController> boomerang;
extern std::shared_ptr<voss::controller::SwingController> swing;
extern std::shared_ptr<voss::controller::ArcPIDController> arc;
extern std::shared_ptr<voss::controller::ExitConditions> ec;

extern voss::Screen screen;
extern pros::Controller primary;