#include "racerbot_wiimote_driver/joy_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <set>

namespace racerbot_wiimote
{

void JoyLayout::validate() const
{
  for (int index : {steering_axis, throttle_axis, accelerate_button, brake_button,
      deadman_button})
  {
    if (index < 0) {
      throw std::invalid_argument("Joy axis and button indices must be non-negative");
    }
  }
  if (steering_axis == throttle_axis) {
    throw std::invalid_argument("steering_axis and throttle_axis must differ");
  }
  const std::set<int> buttons{accelerate_button, brake_button, deadman_button};
  if (buttons.size() != 3) {
    throw std::invalid_argument("accelerate, brake and deadman buttons must all differ");
  }
}

JoyFrame build_joy(const WiimoteInputs & inputs, float steering, const JoyLayout & layout)
{
  JoyFrame frame;
  frame.axes.assign(static_cast<size_t>(std::max(layout.steering_axis, layout.throttle_axis)) + 1,
    0.0F);
  frame.buttons.assign(
    static_cast<size_t>(std::max({layout.accelerate_button, layout.brake_button,
      layout.deadman_button})) + 1, 0);
  if (!inputs.two) {
    return frame;  // deadman released: everything zero
  }
  frame.buttons[layout.deadman_button] = 1;
  frame.buttons[layout.accelerate_button] = inputs.a ? 1 : 0;
  frame.buttons[layout.brake_button] = inputs.b ? 1 : 0;
  // A and B together cancel: an ambiguous request is no request.
  frame.axes[layout.throttle_axis] =
    (inputs.a && !inputs.b) ? 1.0F : (inputs.b && !inputs.a) ? -1.0F : 0.0F;
  frame.axes[layout.steering_axis] = std::clamp(steering, -1.0F, 1.0F);
  return frame;
}

void TiltSettings::validate() const
{
  if (!(y_min < y_center && y_center < y_max)) {
    throw std::invalid_argument(
            "Expected accelerometer_y_min < accelerometer_y_center < accelerometer_y_max");
  }
  if (!(filter_alpha > 0.0 && filter_alpha <= 1.0)) {
    throw std::invalid_argument("accelerometer_filter_alpha must be in (0, 1]");
  }
  if (!(release_threshold >= 0.0 && release_threshold < activation_threshold &&
    activation_threshold <= 1.0))
  {
    throw std::invalid_argument("Expected 0 <= release threshold < activation threshold <= 1");
  }
}

TiltSteering::TiltSteering(const TiltSettings & settings)
: settings_(settings)
{
  settings_.validate();
}

void TiltSteering::reset()
{
  initialized_ = false;
  active_ = false;
  filtered_ = 0.0;
}

float TiltSteering::update(int raw_y)
{
  const double delta = static_cast<double>(raw_y) - settings_.y_center;
  // The two sides may be calibrated separately.
  const double span = delta >= 0.0 ? settings_.y_max - settings_.y_center :
    settings_.y_center - settings_.y_min;
  double normalized = std::clamp(delta / span, -1.0, 1.0);
  if (settings_.invert) {
    normalized = -normalized;
  }
  if (!initialized_) {
    filtered_ = normalized;
    initialized_ = true;
  } else {
    filtered_ += settings_.filter_alpha * (normalized - filtered_);
  }
  const double magnitude = std::abs(filtered_);
  // Schmitt trigger: a hand tremor around centre does not flick the wheels.
  if (active_ && magnitude <= settings_.release_threshold) {
    active_ = false;
  } else if (!active_ && magnitude >= settings_.activation_threshold) {
    active_ = true;
  }
  if (!active_) {
    return 0.0F;
  }
  // Remove the dead zone and rescale so full tilt still reaches 1.0.
  const double output = std::clamp(
    (magnitude - settings_.release_threshold) / (1.0 - settings_.release_threshold), 0.0, 1.0);
  return static_cast<float>(std::copysign(output, filtered_));
}

}  // namespace racerbot_wiimote
