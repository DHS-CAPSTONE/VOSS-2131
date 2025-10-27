/**
 * @file ChangeDetector.hpp
 * @author Andrew Hilton (2131N)
 * @brief Class that detects change in a value
 * @version 0.1
 * @date 2025-07-17
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once

namespace voss {

template <typename T>
class ChangeDetector
{
 private:
  T value;        // Current Value
  bool changed_;  // Whether the Value has changed or not

 public:
  /**
   * @brief Construct a new Change Detector
   *
   */
  ChangeDetector() : value(T()), changed_(false) {}
  /**
   * @brief Construct a new Change Detector
   *
   * @param initial_value Initial value of observed process
   */
  ChangeDetector(T initial_value) : value(initial_value), changed_(false) {}

  /**
   * @brief Check if the value has changed
   *
   * @param new_value new value to be checked
   * @return T Type of the new value
   */
  T checkValue(T new_value)
  {
    changed_ = (value != new_value);
    value = new_value;

    return changed_;
  }
  /**
   * @brief Get the last checked value
   *
   * @return T last value
   */
  T getValue() const { return value; }

  /**
   * @brief Get wether or not the last value was different (ie. changed)
   *
   * @return true It changed
   * @return false It stayed constant
   */
  bool getChanged() const { return changed_; }
};
}