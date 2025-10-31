#pragma once

#include <variant>

namespace voss::chassis
{

struct Stop
{
};

namespace diff_commands
{
struct Voltages
{
  double left;
  double right;
};
struct WheelVelocities
{
  double left;
  double right;
};

struct Chained
{
  double left;
  double right;
};
struct Swing
{
  double left;
  double right;
};
}  // namespace diff_commands

namespace holonomic_commands
{
struct Voltages
{
  double v_x;
  double v_y;
  double v_theta;
};
}  // namespace holonomic_commands

using DiffChassisCommand = std::variant<
    Stop,
    diff_commands::Voltages,
    diff_commands::WheelVelocities,
    diff_commands::Chained,
    diff_commands::Swing>;
using HolonomicChassisCommand = std::variant<Stop, holonomic_commands::Voltages>;

template <class... Ts>
struct overload : Ts...
{
  using Ts::operator()...;
};
template <class... Ts>
overload(Ts...) -> overload<Ts...>;

}  // namespace voss::chassis