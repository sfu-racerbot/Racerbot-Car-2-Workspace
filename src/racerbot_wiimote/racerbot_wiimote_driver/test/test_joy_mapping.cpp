// Wiimote -> Joy rules. Run with `colcon test --packages-select
// racerbot_wiimote_driver` (needs ament_cmake_gtest).
#include <gtest/gtest.h>

#include <stdexcept>

#include "racerbot_wiimote_driver/joy_mapping.hpp"

using racerbot_wiimote::build_joy;
using racerbot_wiimote::JoyLayout;
using racerbot_wiimote::TiltSettings;
using racerbot_wiimote::TiltSteering;
using racerbot_wiimote::WiimoteInputs;

namespace
{
WiimoteInputs held(bool a, bool b, bool two)
{
  WiimoteInputs inputs;
  inputs.a = a;
  inputs.b = b;
  inputs.two = two;
  return inputs;
}
}  // namespace

// --- deadman: the half that matters ---------------------------------------

TEST(BuildJoy, DeadmanReleasedZeroesEverythingEvenWithAHeld)
{
  const auto frame = build_joy(held(true, false, false), 0.8F, JoyLayout{});
  for (float axis : frame.axes) {EXPECT_EQ(axis, 0.0F);}
  for (int button : frame.buttons) {EXPECT_EQ(button, 0);}
}

TEST(BuildJoy, NoInputAtAllIsAllZero)
{
  const auto frame = build_joy(WiimoteInputs{}, 0.0F, JoyLayout{});
  for (float axis : frame.axes) {EXPECT_EQ(axis, 0.0F);}
  for (int button : frame.buttons) {EXPECT_EQ(button, 0);}
}

TEST(BuildJoy, DeadmanHeldWithANonZeroThrottleAndSteering)
{
  const auto frame = build_joy(held(true, false, true), 0.5F, JoyLayout{});
  EXPECT_EQ(frame.buttons.at(4), 1);   // deadman ("2") at the F710's LB slot
  EXPECT_EQ(frame.buttons.at(1), 1);   // A
  EXPECT_EQ(frame.buttons.at(2), 0);
  EXPECT_EQ(frame.axes.at(1), 1.0F);   // throttle forward
  EXPECT_EQ(frame.axes.at(0), 0.5F);   // steering passes through
}

TEST(BuildJoy, BReversesAndBothCancel)
{
  EXPECT_EQ(build_joy(held(false, true, true), 0.0F, JoyLayout{}).axes.at(1), -1.0F);
  EXPECT_EQ(build_joy(held(true, true, true), 0.0F, JoyLayout{}).axes.at(1), 0.0F);
  EXPECT_EQ(build_joy(held(false, false, true), 0.0F, JoyLayout{}).axes.at(1), 0.0F);
}

TEST(BuildJoy, SteeringIsClampedToUnitRange)
{
  EXPECT_EQ(build_joy(held(false, false, true), 3.0F, JoyLayout{}).axes.at(0), 1.0F);
  EXPECT_EQ(build_joy(held(false, false, true), -3.0F, JoyLayout{}).axes.at(0), -1.0F);
}

TEST(BuildJoy, ArraysAreSizedForTheLargestIndex)
{
  // The original driver resized axes to 2 and then indexed the configured
  // axis, which threw for any accelerometer_axis_index above 1.
  JoyLayout layout;
  layout.steering_axis = 5;
  layout.throttle_axis = 2;
  layout.deadman_button = 9;
  const auto frame = build_joy(held(true, false, true), -0.25F, layout);
  ASSERT_EQ(frame.axes.size(), 6u);
  ASSERT_EQ(frame.buttons.size(), 10u);
  EXPECT_EQ(frame.axes[5], -0.25F);
  EXPECT_EQ(frame.axes[2], 1.0F);
  EXPECT_EQ(frame.buttons[9], 1);
}

TEST(JoyLayout, RejectsNegativeOrSharedIndices)
{
  JoyLayout negative;
  negative.deadman_button = -1;
  EXPECT_THROW(negative.validate(), std::invalid_argument);
  JoyLayout shared_axes;
  shared_axes.throttle_axis = shared_axes.steering_axis;
  EXPECT_THROW(shared_axes.validate(), std::invalid_argument);
  JoyLayout shared_buttons;
  shared_buttons.accelerate_button = shared_buttons.deadman_button;
  EXPECT_THROW(shared_buttons.validate(), std::invalid_argument);
  EXPECT_NO_THROW(JoyLayout{}.validate());
}

// --- tilt steering ----------------------------------------------------------

TiltSettings unfiltered()
{
  TiltSettings settings;  // y in [-100, 100], centre 0
  settings.filter_alpha = 1.0;
  return settings;
}

TEST(TiltSteering, CentreIsZeroAndFullTiltIsOne)
{
  TiltSteering tilt(unfiltered());
  EXPECT_EQ(tilt.update(0), 0.0F);
  EXPECT_FLOAT_EQ(tilt.update(100), 1.0F);
  EXPECT_FLOAT_EQ(tilt.update(-100), -1.0F);
  EXPECT_FLOAT_EQ(tilt.update(400), 1.0F);   // past the end clamps
}

TEST(TiltSteering, DeadZoneIsRemovedAndRescaled)
{
  TiltSteering tilt(unfiltered());
  // Closed form: (|n| - release) / (1 - release), release 0.08.
  EXPECT_FLOAT_EQ(tilt.update(50), static_cast<float>((0.5 - 0.08) / 0.92));
}

TEST(TiltSteering, HysteresisNeedsActivationThenHoldsUntilRelease)
{
  TiltSteering tilt(unfiltered());
  EXPECT_EQ(tilt.update(11), 0.0F);         // 0.11 < activation 0.12
  EXPECT_GT(tilt.update(12), 0.0F);         // reaches activation
  EXPECT_GT(tilt.update(9), 0.0F);          // 0.09 > release 0.08: still active
  EXPECT_EQ(tilt.update(8), 0.0F);          // at release: off
  EXPECT_EQ(tilt.update(11), 0.0F);         // must climb to activation again
}

TEST(TiltSteering, AsymmetricCalibrationAndInversion)
{
  TiltSettings settings = unfiltered();
  settings.y_min = -50.0;
  settings.y_center = 10.0;
  settings.y_max = 210.0;
  TiltSteering tilt(settings);
  EXPECT_FLOAT_EQ(tilt.update(210), 1.0F);
  EXPECT_FLOAT_EQ(tilt.update(-50), -1.0F);
  settings.invert = true;
  TiltSteering inverted(settings);
  EXPECT_FLOAT_EQ(inverted.update(210), -1.0F);
}

TEST(TiltSteering, FilterSmoothsAndResetForgets)
{
  TiltSettings settings;
  settings.filter_alpha = 0.5;
  TiltSteering tilt(settings);
  tilt.update(0);
  // One step half way from 0 to 1.0: filtered 0.5 -> (0.5 - 0.08) / 0.92.
  EXPECT_FLOAT_EQ(tilt.update(100), static_cast<float>((0.5 - 0.08) / 0.92));
  tilt.reset();
  EXPECT_FLOAT_EQ(tilt.update(100), 1.0F);  // first sample after reset is taken as is
}

TEST(TiltSettings, RejectsSettingsThatCannotWork)
{
  TiltSettings bad_order;
  bad_order.y_center = 200.0;
  EXPECT_THROW(TiltSteering{bad_order}, std::invalid_argument);
  TiltSettings bad_alpha;
  bad_alpha.filter_alpha = 0.0;
  EXPECT_THROW(TiltSteering{bad_alpha}, std::invalid_argument);
  TiltSettings bad_thresholds;
  bad_thresholds.release_threshold = 0.2;
  EXPECT_THROW(TiltSteering{bad_thresholds}, std::invalid_argument);
}
