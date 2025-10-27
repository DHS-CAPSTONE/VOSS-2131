#include <utility>

#include "VOSS/selector/Selector.hpp"
#include "VOSS/utils/SplitString.hpp"
#include "pros/screen.hpp"

namespace voss {

RectButton::RectButton(
    int16_t x,
    int16_t y,
    int16_t width,
    int16_t height,
    pros::Color boarderColor,
    pros::Color fillColor,
    uint8_t boarder_width,
    const std::string& text,
    int16_t x_padding,
    int16_t y_padding,
    pros::text_format_e_t text_format,
    std::function<void()> callback)
    : x_(x),
      y_(y),
      width_(width),
      height_(height),
      boarder_width_(boarder_width),
      boarder_color_(boarderColor),
      fill_color_(fillColor),
      text_format_(text_format),
      text_(text),
      x_padding_(x_padding),
      y_padding_(y_padding),
      callback_(callback)
{
}

void RectButton::setText(const std::string& newText)
{
  // Update the text
  text_ = newText;

  // If the text isn't empty
  if (!text_.empty())
  {
    // Set the text color and text background
    pros::screen::set_eraser(fill_color_);
    pros::screen::set_pen(boarder_color_);

    // Split string newlines
    auto tokens = splitStr(text_, '\n');

    // Calculate character line height
    int16_t line_height;
    switch (text_format_)
    {
      case pros::text_format_e_t::E_TEXT_SMALL:
        line_height = 10;
        break;
      case pros::text_format_e_t::E_TEXT_MEDIUM:
        line_height = 20;
        break;
      case pros::text_format_e_t::E_TEXT_LARGE:
        line_height = 30;
        break;
      default:
        line_height = 20;
    }

    // For each token print a string on the brain
    for (size_t i = 0; i < tokens.size(); i++)
    {
      pros::screen::print(
          text_format_,
          x_ + x_padding_,
          y_ + y_padding_ + line_height * i,
          "%s",
          tokens[i].c_str());
    }
  }
}

void RectButton::setFillColor(pros::Color newFillColor)
{
  // Set the new fill color
  fill_color_ = newFillColor;

  // Update the button
  ignore_press_ = true;
  update();
}

bool RectButton::getPressed()
{
  // Return if the button has been pressed
  return pressed_;
}

void RectButton::update()
{
  // If the screen has been pressed or ignore press has been set
  if (pros::screen::touch_status().touch_status == pros::last_touch_e_t::E_TOUCH_PRESSED ||
      ignore_press_)
  {
    // Turn ignore press off (we updating)
    ignore_press_ = false;

    // Set the fill and boarder color
    pros::screen::set_eraser(fill_color_);
    pros::screen::set_pen(boarder_color_);

    // Fill the background
    pros::screen::erase_rect(x_, y_, x_ + width_, y_ + height_);

    // Make smaller and smaller Rectangles to make the boarder width
    for (uint8_t i = 0; i < boarder_width_; ++i)
    {
      pros::screen::draw_rect(x_ + i, y_ + i, x_ + width_ - i, y_ + height_ - i);
    }

    // if the text isn't empty then draw the text
    if (!text_.empty()) { this->setText(text_); }

    // Set pressed to whether or not the touch status (x, y) is within the rectangle
    pressed_ =
        pros::screen::touch_status().x >= x_ && pros::screen::touch_status().x <= x_ + width_ &&
        pros::screen::touch_status().y >= y_ && pros::screen::touch_status().y <= y_ + height_;

    // if pressed then call the callback
    if (pressed_) { callback_(); }
  }
}

// Screen definitions
Screen::Screen()
    : name_button_(
          0,
          0,
          420,
          60,
          pros::Color::white,
          pros::Color::dark_red,
          2,
          "",
          10,
          10,
          pros::E_TEXT_LARGE),
      description_button_(
          0,
          60,
          480,
          170,
          pros::Color::white,
          pros::Color::dark_red,
          2,
          "",
          10,
          10,
          pros::E_TEXT_MEDIUM),
      color_button_(
          420,
          0,
          60,
          60,
          pros::Color::white,
          pros::Color::red,
          2,
          "",
          10,
          10,
          pros::E_TEXT_MEDIUM,
          [this]() { this->setRedTeam(!is_red_team_); }),
      screen_task_([this] { this->run(); }, "Screen Task")
{
}

void Screen::initialize(size_t current_auto_index, bool is_red_team)
{
  initialized_ = true;  // Initalized Called
  // Set Current Auto (Wrap to not use uninitialized memory)
  current_auto_index_ = autos_.empty() ? 0 : current_auto_index % autos_.size();
  this->setRedTeam(is_red_team);
}

void Screen::addAutos(const std::vector<AutoInfo>& autos)
{
  // Insert Autos to the auton list
  autos_.insert(autos_.end(), autos.begin(), autos.end());
}

void Screen::addTelemetries(
    const std::map<std::string, std::function<std::string()>>& telemetry_data)
{
  // Add telemetry data to the telemetry data list
  telemetry_data_.insert(telemetry_data.begin(), telemetry_data.end());
}

std::function<void(bool)> Screen::getCurrentAutoCallback() const
{
  // Return the current index's callback
  return autos_[current_auto_index_].autonCallback;
}

const AutoInfo& Screen::getCurrentAuto() const { return autos_[current_auto_index_]; }
void Screen::setRedTeam(bool isRedTeam) { is_red_team_ = isRedTeam; }
bool Screen::getRedTeam() { return is_red_team_; }

void Screen::run()
{
  while (true)
  {
    // If not initalized
    if (!initialized_)
    {
      // Delay a little
      pros::delay(10);
      // Skip everything below
      continue;
    }

    // If Auto list empty
    if (autos_.empty())
    {
      // Set the Name to "No Autos Available"
      name_button_.setText("No Autos Available");

      // Set the description
      std::string description = "Please add autos to the list.\n";

      // Add telemetry data to description
      for (const auto& [key, value] : telemetry_data_)
      {
        description += key + ": " + value() + "\n";
      }

      // Update button name with new text
      description_button_.setText(description);
    }
    else
    {
      // Display Current Auto Name
      name_button_.setText(autos_[current_auto_index_].name);

      // Set the Description
      std::string description = autos_[current_auto_index_].description + "\n";

      // Add the telemetry data to the description
      for (const auto& [key, value] : telemetry_data_)
      {
        description += key + ": " + value() + "\n";
      }

      // Update the button name with new text
      description_button_.setText(description);
    }

    // Update all the buttons
    name_button_.update();
    description_button_.update();
    color_button_.update();

    // Check if the screen has been pressed (but isn't pressing)
    if (screen_press_detector_.checkValue(
            pros::screen::touch_status().touch_status == pros::last_touch_e_t::E_TOUCH_PRESSED) &&
        screen_press_detector_.getValue())
    {
      // if color button not pressed, then cycle auto
      if (!color_button_.getPressed())
      {
        current_auto_index_ = (current_auto_index_ + 1) % autos_.size();
      }

      // Color according to what color the team is
      if (is_red_team_)
      {
        color_button_.setFillColor(pros::Color::red);
        description_button_.setFillColor(pros::Color::dark_red);
        name_button_.setFillColor(pros::Color::dark_red);
      }
      else
      {
        color_button_.setFillColor(pros::Color::blue);
        description_button_.setFillColor(pros::Color::dark_blue);
        name_button_.setFillColor(pros::Color::dark_blue);
      }
    }

    // Don't hog the CPU
    pros::delay(50);
  }
}
} // namespace voss