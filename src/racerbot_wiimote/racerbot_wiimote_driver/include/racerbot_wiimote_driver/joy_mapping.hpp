#ifndef RACERBOT_WIIMOTE_DRIVER__JOY_MAPPING_HPP_
#define RACERBOT_WIIMOTE_DRIVER__JOY_MAPPING_HPP_

// Wiimote input -> sensor_msgs/Joy layout, with no ROS and no xwiimote, so
// every rule here is unit-tested (test/test_joy_mapping.cpp).

#include <cstdint>
#include <stdexcept>
#include <vector>

namespace racerbot_wiimote
{

// Where each control lands in the published Joy message.
struct JoyLayout
{
  int steering_axis = 0;      // tilt, -1 (right) .. +1 (left)
  int throttle_axis = 1;      // +1 while A is held, -1 while B is held
  int accelerate_button = 1;  // A
  int brake_button = 2;       // B
  int deadman_button = 4;     // "2"

  // Throws std::invalid_argument for a negative or shared index.
  void validate() const;
};

// Raw Wiimote state for one tick.
struct WiimoteInputs
{
  bool a = false;
  bool b = false;
  bool two = false;
  int accel_y = 0;
};

struct JoyFrame
{
  std::vector<float> axes;
  std::vector<int32_t> buttons;
};

// Builds the Joy message for one tick.
//
// Safety rule: unless the deadman ("2") is held, every button and axis is
// zero. joy_teleop's no-buttons "default" profile then publishes an active
// zero command, so letting go of "2" stops the car instead of leaving the
// last command in place until timeouts expire.
JoyFrame build_joy(const WiimoteInputs & inputs, float steering, const JoyLayout & layout);

// Tilt steering: raw accelerometer Y -> -1..1 with a calibrated centre,
// optional inversion, a low-pass filter and a hysteresis dead zone.
struct TiltSettings
{
  double y_min = -100.0;
  double y_center = 0.0;
  double y_max = 100.0;
  bool invert = false;
  double filter_alpha = 0.2;          // 1.0 = no filtering
  double activation_threshold = 0.12;  // normalised units
  double release_threshold = 0.08;

  // Throws std::invalid_argument if the settings cannot work.
  void validate() const;
};

class TiltSteering
{
public:
  explicit TiltSteering(const TiltSettings & settings);

  // One filter step; returns the steering axis value in [-1, 1].
  float update(int raw_y);

  // Forget the filter state (after a reconnect the old value is stale).
  void reset();

private:
  TiltSettings settings_;
  bool initialized_ = false;
  bool active_ = false;
  double filtered_ = 0.0;
};

}  // namespace racerbot_wiimote

#endif  // RACERBOT_WIIMOTE_DRIVER__JOY_MAPPING_HPP_
