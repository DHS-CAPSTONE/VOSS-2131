#include "Competition/RobotConfig.hpp"

#include "VOSS/chassis/DiffChassis.hpp"
#include "VOSS/controller/ArcPIDControllerBuilder.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/exit_conditions/ExitConditions.hpp"
#include "VOSS/localizer/IMELocalizerBuilder.hpp"

std::shared_ptr<voss::localizer::IMELocalizer> odom =
    voss::localizer::IMELocalizerBuilder::new_builder()
        .with_left_motors({10, -9, -8})
        .with_right_motors({-1, 2, 3})
        .with_imu(21)
        .with_left_right_tpi(400.0 / 9.0)
        .with_middle_tpi(400.0 / 9.0)
        .with_track_width(10.75)  //
        .build();

std::shared_ptr<voss::controller::PIDController> pid =
    voss::controller::PIDControllerBuilder::new_builder(odom)
        .with_linear_constants(8, 0, 70)
        .with_angular_constants(250, 0.001, 2500)
        .with_min_error(5)
        .with_min_vel_for_thru(40)
        .build();

std::shared_ptr<voss::controller::BoomerangController> boomerang =
    voss::controller::BoomerangControllerBuilder::new_builder(odom)
        .with_linear_constants(8, 0, 70)
        .with_angular_constants(250, 0.001, 2500)
        .with_lead_pct(0.6)
        .with_min_vel_for_thru(70)
        .with_min_error(10)
        .build();

std::shared_ptr<voss::controller::SwingController> swing =
    voss::controller::SwingControllerBuilder::new_builder(odom)
        .with_angular_constants(250, 0.05, 2435)
        .build();

std::shared_ptr<voss::controller::ArcPIDController> arc =
    voss::controller::ArcPIDControllerBuilder(odom)
        .with_track_width(16)
        .with_linear_constants(20, 0.02, 169)
        .with_angular_constants(250, 0.05, 2435)
        .with_min_error(5)
        .with_slew(8)
        .build();

std::shared_ptr<voss::controller::ExitConditions> ec =
    voss::controller::ExitConditions::new_conditions()
        .add_settle(400, 0.5, 400)
        .add_tolerance(1.0, 2.0, 200)
        .add_timeout(22500)
        .add_thru_smoothness(4)
        .build();

voss::chassis::DiffChassis chassis(
    {10, -9, -8},  //
    {-1, 2, 3},
    pid,
    ec,
    8,
    0.0,
    0.0,
    pros::E_MOTOR_BRAKE_COAST);

voss::Screen screen;
pros::Controller primary(pros::E_CONTROLLER_MASTER);
