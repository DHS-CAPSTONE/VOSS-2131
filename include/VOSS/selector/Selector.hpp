/**
 * @file screen.hpp
 * @author Andrew Hilton (2131N)
 * @brief Screen UI classes and Implementation
 * @version 0.1
 * @date 2025-07-17
 *
 * @copyright Copyright (c) 2025
 *
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>

#include "VOSS/utils/ChangeDetector.hpp"
#include "pros/colors.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.h"

namespace voss {
struct AutoInfo
{
  std::string name;                         // Name of Autonomous
  std::string description;                  // Description of Autonomous
  std::function<void(bool)> autonCallback;  // Callback to Autonomous Code
};

class RectButton
{
 private:
  int16_t x_, y_, width_, height_;     // Rectangle Characteristics
  uint8_t boarder_width_;              // Pen Boarder Width
  pros::Color boarder_color_;          // Color of Boarder Width
  pros::Color fill_color_;             // Background Fill Color
  std::string text_;                   // Text of Button
  int16_t x_padding_, y_padding_;      // Padding from button edge to text
  pros::text_format_e_t text_format_;  // Text Format (ie. large, medium, small)
  std::function<void()> callback_;     // Callback for when Button is pressed
  bool ignore_press_ = true;           // ALlows for update despite not being pressed
  bool pressed_ = false;               // Is Button Pressed?

 public:
  /**
   * @brief Construct a new Rect Button
   *
   * @param x X Position on Screen (0-480)
   * @param y Y Position on Screen (0-240)
   * @param width Width of Button in Pixels
   * @param height Height of Button in Pixels
   * @param boarder_color Boarder Color
   * @param fill_color Fill Color
   * @param boarder_width Boarder Width in Pixels
   * @param text Text to be displayed on a button
   * @param x_padding Padding from boarder edge
   * @param y_padding Padding from boarder edge
   * @param textFormat Text formate (ie. large, medium, small)
   * @param callback Callback for when button is pressed
   */
  RectButton(
      int16_t x,
      int16_t y,
      int16_t width,
      int16_t height,
      pros::Color boarder_color,
      pros::Color fill_color,
      uint8_t boarder_width = 1,
      const std::string& text = "",
      int16_t xPadding = 1,
      int16_t yPadding = 1,
      pros::text_format_e_t text_format = pros::text_format_e_t::E_TEXT_MEDIUM,
      std::function<void()> callback = []() {});

  /**
   * @brief Set the Text of a Button
   *
   * @param new_text New text to be displayed
   */
  void setText(const std::string& new_text);

  /**
   * @brief Set the Background Fill Color object
   *
   * @param new_fill_color new color to fill the background with
   */
  void setFillColor(pros::Color new_fill_color);

  /**
   * @brief get whether or not the button is pressed
   *
   * @return true (Is Pressing)
   * @return false (Is not Pressing)
   */
  bool getPressed();

  /**
   * @brief Update a button (draw, get presses, etc)
   *
   */
  void update();
};

class Screen
{
 private:
  std::vector<AutoInfo> autos_;            // List of auto info
  size_t current_auto_index_ = size_t(0);  // Current index of list

  std::map<std::string, std::function<std::string()>>
      telemetry_data_;  // Map of Telemetries (label, function that returns text to be displayed
                        // every loop)

  RectButton name_button_;         // Button that displays the name of the selected Autonomous
  RectButton description_button_;  // Button that shows a description of the selected Autonomous
  RectButton color_button_;        // Button that toggles the color of the alliance

  bool is_red_team_ = true;  // Is the robot on red team?

  pros::Task screen_task_;    // Thread to run the screen task
  bool initialized_ = false;  // Has the screen been initialized?

  ChangeDetector<bool> screen_press_detector_;  // Change detector to get button presses

 public:
  /**
   * @brief Construct a new Screen object
   *
   */
  Screen();

  /**
   * @brief Initialize the Screen
   *
   * @param current_auto_index Set the Currently Displayed/Selected Autonomous
   */
  void initialize(size_t current_auto_index = 0, bool is_red_team = true);

  /**
   * @brief Add a list of Autos (AutoInfo Structs) to the screen
   *
   * @param autos List of AutoInfo Structs ie. {name, description, callback}
   */
  void addAutos(const std::vector<AutoInfo>& autos);
  /**
   * @brief Add list of telemetries to be shown on the screen
   *
   * @param telemetryData Map of (index, function with string return)
   */
  void addTelemetries(const std::map<std::string, std::function<std::string()>>& telemetry_data);

  /**
   * @brief Get the Current Auto Callback object
   *
   * @return std::function<void(bool)> Autonomous Route
   */
  std::function<void(bool)> getCurrentAutoCallback() const;

  /**
   * @brief Get the Current Auto object
   *
   * @return const AutoInfo& Currently Shown AutoInfo
   */
  const AutoInfo& getCurrentAuto() const;

  /**
   * @brief Set the team color
   *
   * @param is_red_team
   */
  void setRedTeam(bool is_red_team);

  /**
   * @brief Get the team color
   *
   * @return true red team
   * @return false blue team
   */
  bool getRedTeam();

 protected:
  /**
   * @brief Background thread impl
   *
   */
  void run();
};
}  // namespace voss