#include "main.h"

#include "Competition/MatchAutos.hpp"
#include "Competition/RobotConfig.hpp"
#include "VOSS/chassis/DiffChassis.hpp"
#include "VOSS/utils/angle.hpp"
#include "VOSS/utils/debug.hpp"
#include "VOSS/utils/flags.hpp"

void initialize()
{
  odom->begin_localization();
  odom->set_pose({0, 0, 0});

  screen.addAutos({
      {"Debug", "Debug Auto, DO NOT RUN AT COMP", debug},
      {"Left Side", "Left Side Half Autonomous Win Point", leftSideAWP},
      {"Right Side", "Right Side Half Autonomous Win Point", rightSideAWP},
      {"Skills", "Skills Autonomous", skills},
  });

  screen.initialize(2, true);

  screen.addTelemetries({
      {"Battery", []() { return std::to_string(pros::battery::get_capacity()); }},
      {"Position",
       []() {
         auto position = odom->get_pose();
         return "  " + std::to_string(position.x) + ", " + std::to_string(position.y) + ", " +
                std::to_string(voss::to_degrees(
                    position.theta.value_or(std::numeric_limits<double>::infinity())));
       }},
      {"Velocity",
       []() {
         auto velocity = odom->get_velocity();
         return "  " + std::to_string(velocity.x) + ", " + std::to_string(velocity.y) + ", " +
                std::to_string(voss::to_degrees(
                    velocity.theta.value_or(std::numeric_limits<double>::infinity())));
       }},
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() { chassis.move({0, 0, 0}, follow_velocity_path, 10000, voss::Flags::RELATIVE); }

void opcontrol()
{
  while (true)
  {
    chassis.tank(primary.get_analog(ANALOG_LEFT_Y), primary.get_analog(ANALOG_RIGHT_Y));

    if (primary.get_digital_new_press(DIGITAL_Y))
    {
      voss::enable_debug();
      chassis.move({36, 36, 90}, arc, 70, voss::Flags::RELATIVE);
      voss::disable_debug();
    }

    // double x = 10.0;
    // chassis.execute(voss::chassis::diff_commands::WheelVelocities{x, x}, 1000.0);

    pros::delay(10);
  }
}