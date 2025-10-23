#include "main.h"

#include "VOSS/chassis/DiffChassis.hpp"
#include "VOSS/controller/ArcPIDControllerBuilder.hpp"
#include "VOSS/controller/BoomerangControllerBuilder.hpp"
#include "VOSS/controller/PIDControllerBuilder.hpp"
#include "VOSS/controller/SwingControllerBuilder.hpp"
#include "VOSS/exit_conditions/ExitConditions.hpp"
#include "VOSS/localizer/IMELocalizerBuilder.hpp"
#include "VOSS/utils/debug.hpp"
#include "VOSS/utils/flags.hpp"

auto odom = voss::localizer::IMELocalizerBuilder::new_builder()
                .with_left_motors({1, 2, 3})
                .with_right_motors({4, 5, 6})
                .with_imu(19)
                .with_left_right_tpi(200.0 * 400.0 / 600.0)
                .with_middle_tpi(200.0 * 400.0 / 600.0)
                .with_track_width(10.75)  //
                .build();

auto pid = voss::controller::PIDControllerBuilder::new_builder(odom)
               .with_linear_constants(8, 0, 70)
               .with_angular_constants(250, 0.001, 2500)
               .with_min_error(5)
               .with_min_vel_for_thru(40)
               .build();

auto boomerang = voss::controller::BoomerangControllerBuilder::new_builder(odom)
                     .with_linear_constants(8, 0, 70)
                     .with_angular_constants(250, 0.001, 2500)
                     .with_lead_pct(0.6)
                     .with_min_vel_for_thru(70)
                     .with_min_error(10)
                     .build();

auto swing = voss::controller::SwingControllerBuilder::new_builder(odom)
                 .with_angular_constants(250, 0.05, 2435)
                 .build();

auto arc = voss::controller::ArcPIDControllerBuilder(odom)
               .with_track_width(16)
               .with_linear_constants(20, 0.02, 169)
               .with_angular_constants(250, 0.05, 2435)
               .with_min_error(5)
               .with_slew(8)
               .build();

pros::Controller master(pros::E_CONTROLLER_MASTER);
auto ec = voss::controller::ExitConditions::new_conditions()
              .add_settle(400, 0.5, 400)
              .add_tolerance(1.0, 2.0, 200)
              .add_timeout(22500)
              .add_thru_smoothness(4)
              .build()
              ->exit_if([]() -> bool { return master.get_digital(pros::E_CONTROLLER_DIGITAL_UP); });

auto chassis = voss::chassis::DiffChassis(
    {1, 2, 3},  //
    {4, 5, 6},
    pid,
    ec,
    8,
    pros::E_MOTOR_BRAKE_COAST);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize()
{
  pros::lcd::initialize();
  odom->begin_localization();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol()
{
  while (true)
  {
    voss::Pose p = odom->get_pose();

    chassis.tank(master.get_analog(ANALOG_LEFT_Y), master.get_analog(ANALOG_RIGHT_Y));

    if (master.get_digital_new_press(DIGITAL_Y))
    {
      voss::enable_debug();
      chassis.move({-36, -36, 90}, boomerang, 70, voss::Flags::RELATIVE);
      voss::disable_debug();
    }

    pros::lcd::clear_line(1);
    pros::lcd::clear_line(2);
    pros::lcd::clear_line(3);
    pros::lcd::print(1, "%lf", p.x);
    pros::lcd::print(2, "%lf", p.y);
    pros::lcd::print(3, "%lf", odom->get_orientation_deg());
    pros::lcd::print(4, "%s", (odom == nullptr) ? "true" : "false");
    pros::delay(10);
  }
}