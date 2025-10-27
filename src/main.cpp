#include "main.h"
#include "Competition/RobotConfig.hpp"
#include "Competition/MatchAutos.hpp"

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
       [](){
         auto position = odom->get_pose();
         return "\n  X: " + std::to_string(position.x) + "\n  Y: " + std::to_string(position.y) +
                "\n  Theta: " + std::to_string(position.theta.value_or(0.0));
       }},
  });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

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

    pros::delay(10);
  }
}