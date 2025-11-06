/**
 * @file MotionProfile.hpp
 * @author Andrew Hilton (2131N)
 * @brief Motion Profile Class
 * @version 0.1
 * @date 2025-11-06
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cmath>
class MotionProfile
{
 private:
  const double v_initial;
  const double v_final;

  const double a_max;
  const double v_max;

  const double d_total;

  const double d_acceleration;
  const double d_deceleration;
  const double d_coast;

  const double t_acceleration;
  const double t_deceleration;
  const double t_coast;
  const double t_total;

 public:
  MotionProfile(
      const double v_initial,
      const double v_final,

      const double a_max,
      const double v_max,

      const double distance)
      : v_initial(v_initial),
        v_final(v_final),
        a_max(a_max),
        v_max(v_max),
        d_total(distance),
        d_acceleration((v_max / a_max) * (v_max / a_max) * 0.5 * a_max),
        d_deceleration((v_max - v_final) / a_max * (v_max - v_final) / a_max * 0.5 * a_max),
        d_coast(distance - d_acceleration - d_deceleration),
        t_acceleration(v_max / a_max),
        t_deceleration((v_max - v_final) / a_max),
        t_coast(d_coast / v_max),
        t_total(t_acceleration + t_coast + t_deceleration)
  {
  }

  double get_velocity(double time)
  {
    if (time < t_acceleration) { return v_initial + a_max * time; }
    else if (time < t_acceleration + t_coast) { return v_max; }
    else if (time < t_total) { return -a_max * (time - t_total) + v_final; }
    else { return infinity(); }
  }

  double distance_to_time(double distance)
  {

    if (distance < d_acceleration) { return sqrt(2.0 * distance / a_max); }
    else if (distance < d_coast + d_acceleration)
    {
      return (distance - d_acceleration) / v_max + t_acceleration;
    }
    else if (distance < d_total)
    {
      return sqrt(2 * (distance - d_acceleration - d_coast) / a_max) + t_acceleration +
             t_deceleration;
    }
    return infinity();
  }
};