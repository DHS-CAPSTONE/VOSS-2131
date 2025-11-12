/**
 * @file MotionProfile.hpp
 * @author Andrew Hilton (2131N)
 * @brief Trapezoidal Motion Profile
 * @version 0.2
 * @date 2025-11-09
 */

#pragma once

#include <cmath>

class MotionProfile
{
 private:
  double v_initial;
  double v_final;

  double a_max;
  double v_max;

  double d_total;

  // Computed parameters
  double v_peak;  // actual top velocity (may be < v_max)
  double t_accel;
  double t_decel;
  double t_coast;
  double t_total;

  double d_accel;
  double d_decel;
  double d_coast;

 public:
  MotionProfile(double v_initial, double v_final, double a_max, double v_max, double distance)
      : v_initial(v_initial), v_final(v_final), a_max(a_max), v_max(v_max), d_total(distance)
  {
    // 1️⃣ Compute distances required for accel and decel phases at v_max
    double d_accel_full = (std::pow(v_max, 2) - std::pow(v_initial, 2)) / (2.0 * a_max);
    double d_decel_full = (std::pow(v_max, 2) - std::pow(v_final, 2)) / (2.0 * a_max);

    // 2️⃣ Check if we have enough distance to reach v_max
    if (d_accel_full + d_decel_full <= d_total)
    {
      // Trapezoidal profile (full-speed reach)
      v_peak = v_max;
      d_accel = d_accel_full;
      d_decel = d_decel_full;
      d_coast = d_total - d_accel - d_decel;

      t_accel = (v_peak - v_initial) / a_max;
      t_decel = (v_peak - v_final) / a_max;
      t_coast = d_coast / v_peak;
      t_total = t_accel + t_coast + t_decel;
    }
    else
    {
      // Triangular profile (never reach v_max)
      // Solve for v_peak using energy balance:
      // total_distance = d_accel + d_decel
      // where d_accel = (v_peak^2 - v_i^2) / (2a), d_decel = (v_peak^2 - v_f^2) / (2a)
      v_peak =
          std::sqrt((2 * a_max * d_total + std::pow(v_initial, 2) + std::pow(v_final, 2)) / 2.0);

      d_accel = (std::pow(v_peak, 2) - std::pow(v_initial, 2)) / (2.0 * a_max);
      d_decel = (std::pow(v_peak, 2) - std::pow(v_final, 2)) / (2.0 * a_max);
      d_coast = 0.0;

      t_accel = (v_peak - v_initial) / a_max;
      t_decel = (v_peak - v_final) / a_max;
      t_coast = 0.0;
      t_total = t_accel + t_decel;
    }
  }

  double get_acceleration(double t) const
  {
    if (t < 0) return 0;
    if (t < t_accel) return a_max;
    if (t < t_accel + t_coast) return 0.0;
    if (t < t_total) return -a_max;
    return 0.0;
  }

  double get_velocity(double t) const
  {
    if (t < 0) return v_initial;
    if (t < t_accel) { return v_initial + a_max * t; }
    else if (t < t_accel + t_coast) { return v_peak; }
    else if (t < t_total)
    {
      double t_into_decel = t - (t_accel + t_coast);
      return v_peak - a_max * t_into_decel;
    }
    return v_final;
  }

  double get_distance(double t) const
  {
    if (t < 0) return 0.0;
    if (t < t_accel) { return v_initial * t + 0.5 * a_max * t * t; }
    else if (t < t_accel + t_coast)
    {
      double t_coast_phase = t - t_accel;
      return d_accel + v_peak * t_coast_phase;
    }
    else if (t < t_total)
    {
      double t_decel_phase = t - (t_accel + t_coast);
      return d_accel + d_coast + v_peak * t_decel_phase -
             0.5 * a_max * t_decel_phase * t_decel_phase;
    }
    return d_total;
  }

  double get_total_time() const { return t_total; }
  double get_peak_velocity() const { return v_peak; }
  double get_time(double d) const
  {
    if (d <= 0) return 0.0;

    // --- Acceleration phase ---
    if (d < d_accel)
    {
      // d = v_i * t + 0.5 * a * t^2
      // Solve quadratic: 0.5 * a * t^2 + v_i * t - d = 0
      double discriminant = v_initial * v_initial + 2 * a_max * d;
      return (-v_initial + std::sqrt(discriminant)) / a_max;
    }

    // --- Coasting phase ---
    else if (d < d_accel + d_coast)
    {
      double d_coast_phase = d - d_accel;
      return t_accel + d_coast_phase / v_peak;
    }

    // --- Deceleration phase ---
    else if (d < d_total)
    {
      double d_decel_phase = d - (d_accel + d_coast);

      // During decel:
      // d' = v_peak * t - 0.5 * a * t^2
      // 0.5 * a * t^2 - v_peak * t + d' = 0
      double discriminant = v_peak * v_peak - 2 * a_max * d_decel_phase;
      double t_decel_phase = (v_peak - std::sqrt(discriminant)) / a_max;
      return t_accel + t_coast + t_decel_phase;
    }

    return t_total;
  }
};