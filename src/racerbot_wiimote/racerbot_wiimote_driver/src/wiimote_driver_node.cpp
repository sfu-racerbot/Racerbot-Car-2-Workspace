#include "wiimote_driver_node.hpp"

#include <xwiimote.h>

#include <chrono>
#include <stdexcept>
#include <string>

namespace
{
constexpr char kButtonsTopic[] = "/wiimote/buttons";
constexpr char kLedTopic[] = "/wiimote/led";
constexpr char kBatteryTopic[] = "/wiimote/battery";
constexpr char kAccelerometerTopic[] = "/wiimote/accel";
constexpr char kRumbleTopic[] = "/wiimote/rumble";
constexpr unsigned int kConnectedLed = 0;
constexpr unsigned int kDeadmanLed = 1;
}  // namespace

WiimoteDriverNode::WiimoteDriverNode()
: Node("wiimote_driver_node")
{
  // /wiimote/joy, not /joy: the autonomy nodes read the F710's LB on /joy,
  // and a Wiimote must never stand in for that deadman (workspace policy).
  joy_topic_ = declare_parameter<std::string>("joy_topic", "/wiimote/joy");
  raw_mode_ = declare_parameter<bool>("raw_mode", false);
  const auto poll_period_ms = declare_parameter<int64_t>("poll_period_ms", 10);
  reconnect_period_s_ = declare_parameter<double>("reconnect_period_s", 1.0);
  battery_period_s_ = declare_parameter<double>("battery_period_s", 5.0);
  input_timeout_s_ = declare_parameter<double>("input_timeout_s", 0.5);

  layout_.steering_axis = static_cast<int>(declare_parameter<int64_t>("steering_axis_index", 0));
  layout_.throttle_axis = static_cast<int>(declare_parameter<int64_t>("throttle_axis_index", 1));
  layout_.accelerate_button = static_cast<int>(
    declare_parameter<int64_t>("accelerate_button_index", 1));
  layout_.brake_button = static_cast<int>(declare_parameter<int64_t>("brake_button_index", 2));
  layout_.deadman_button = static_cast<int>(
    declare_parameter<int64_t>("deadman_button_index", 4));
  layout_.validate();

  racerbot_wiimote::TiltSettings tilt;
  tilt.y_min = declare_parameter<double>("accelerometer_y_min", -100.0);
  tilt.y_center = declare_parameter<double>("accelerometer_y_center", 0.0);
  tilt.y_max = declare_parameter<double>("accelerometer_y_max", 100.0);
  tilt.invert = declare_parameter<bool>("invert_accelerometer_y", false);
  tilt.filter_alpha = declare_parameter<double>("accelerometer_filter_alpha", 0.2);
  tilt.activation_threshold =
    declare_parameter<double>("accelerometer_activation_threshold", 0.12);
  tilt.release_threshold = declare_parameter<double>("accelerometer_release_threshold", 0.08);
  tilt_.emplace(tilt);  // validates

  if (poll_period_ms <= 0 || reconnect_period_s_ <= 0.0 || battery_period_s_ <= 0.0 ||
    input_timeout_s_ <= 0.0)
  {
    throw std::invalid_argument(
            "poll, reconnect, battery and input timeout periods must be positive");
  }

  if (raw_mode_) {
    RCLCPP_INFO(get_logger(), "Raw mode: no Joy messages, no deadman LED.");
  } else {
    joy_publisher_ = create_publisher<sensor_msgs::msg::Joy>(joy_topic_, 10);
  }
  buttons_publisher_ =
    create_publisher<racerbot_wiimote_msgs::msg::WiimoteButtons>(kButtonsTopic, 10);
  battery_publisher_ =
    create_publisher<racerbot_wiimote_msgs::msg::WiimoteBattery>(kBatteryTopic, 1);
  accelerometer_publisher_ =
    create_publisher<racerbot_wiimote_msgs::msg::WiimoteAccelerometer>(kAccelerometerTopic, 10);
  led_subscriber_ = create_subscription<racerbot_wiimote_msgs::msg::WiimoteLED>(
    kLedTopic, 1, [this](racerbot_wiimote_msgs::msg::WiimoteLED::SharedPtr msg) {
      led_callback(msg);
    });
  rumble_subscriber_ = create_subscription<racerbot_wiimote_msgs::msg::WiimoteRumble>(
    kRumbleTopic, 1, [this](racerbot_wiimote_msgs::msg::WiimoteRumble::SharedPtr msg) {
      rumble_callback(msg);
    });

  last_connect_attempt_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  last_battery_ = last_connect_attempt_;
  timer_ = create_wall_timer(
    std::chrono::milliseconds(poll_period_ms), [this]() {tick();});
  RCLCPP_INFO(get_logger(), "Wiimote driver started; Joy on %s.",
    raw_mode_ ? "(raw mode, none)" : joy_topic_.c_str());
}

void WiimoteDriverNode::try_connect()
{
  const auto now = this->now();
  if ((now - last_connect_attempt_).seconds() < reconnect_period_s_) {
    return;
  }
  last_connect_attempt_ = now;
  try {
    device_ = WiimoteDevice::open_first();
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 10000, "%s", e.what());
    return;
  }
  if (!device_) {
    if (!waiting_logged_) {
      RCLCPP_WARN(get_logger(),
        "No Wiimote connected. Press 1+2 on it (or run `ros2 run racerbot_wiimote_driver "
        "wiimote_connect`); this node connects as soon as it appears.");
      waiting_logged_ = true;
    }
    return;
  }
  waiting_logged_ = false;
  tilt_->reset();
  RCLCPP_INFO(get_logger(), "Wiimote connected (%s).", device_->path().c_str());
  set_led_safely(kConnectedLed, true);
  last_battery_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
}

void WiimoteDriverNode::disconnect(const std::string & why)
{
  RCLCPP_WARN(get_logger(), "Wiimote lost (%s). No Joy is published until it reconnects.",
    why.c_str());
  device_.reset();
}

void WiimoteDriverNode::tick()
{
  if (!device_) {
    // No Wiimote: say "nothing held" so the teleop layer commands a stop.
    publish_zero_joy();
    try_connect();
    return;
  }
  if (!device_->update()) {
    disconnect("Bluetooth link closed");
    publish_zero_joy();
    return;
  }
  if (device_->seconds_since_input() > input_timeout_s_) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No input from the Wiimote for %.1f s (out of range, or lying perfectly still); "
      "publishing nothing held.", device_->seconds_since_input());
    publish_zero_joy();
    set_led_safely(kDeadmanLed, false);
    return;
  }
  // Each output is independent: an LED or battery error must never stop the
  // Joy messages (the original driver lost /joy entirely on an LED
  // permission error, because one exception aborted the whole tick).
  publish_inputs();
  if (!raw_mode_) {
    set_led_safely(kDeadmanLed, device_->is_button_down(XWII_KEY_TWO));
  }
  if ((this->now() - last_battery_).seconds() >= battery_period_s_) {
    last_battery_ = this->now();
    publish_battery();
  }
}

void WiimoteDriverNode::publish_inputs()
{
  const auto stamp = this->now();
  const auto accel = device_->accelerometer();

  racerbot_wiimote_msgs::msg::WiimoteButtons buttons;
  buttons.header.stamp = stamp;
  buttons.a = device_->is_button_down(XWII_KEY_A);
  buttons.b = device_->is_button_down(XWII_KEY_B);
  buttons.one = device_->is_button_down(XWII_KEY_ONE);
  buttons.two = device_->is_button_down(XWII_KEY_TWO);
  buttons.home = device_->is_button_down(XWII_KEY_HOME);
  buttons.plus = device_->is_button_down(XWII_KEY_PLUS);
  buttons.minus = device_->is_button_down(XWII_KEY_MINUS);
  buttons.up = device_->is_button_down(XWII_KEY_UP);
  buttons.down = device_->is_button_down(XWII_KEY_DOWN);
  buttons.left = device_->is_button_down(XWII_KEY_LEFT);
  buttons.right = device_->is_button_down(XWII_KEY_RIGHT);
  buttons_publisher_->publish(buttons);

  racerbot_wiimote_msgs::msg::WiimoteAccelerometer accel_msg;
  accel_msg.header.stamp = stamp;
  accel_msg.x = static_cast<int16_t>(accel.x);
  accel_msg.y = static_cast<int16_t>(accel.y);
  accel_msg.z = static_cast<int16_t>(accel.z);
  accelerometer_publisher_->publish(accel_msg);

  // The filter runs every tick so it is settled when the deadman goes down.
  const float steering = tilt_->update(accel.y);
  if (raw_mode_) {
    return;
  }
  racerbot_wiimote::WiimoteInputs inputs;
  inputs.a = buttons.a;
  inputs.b = buttons.b;
  inputs.two = buttons.two;
  const auto frame = racerbot_wiimote::build_joy(inputs, steering, layout_);
  sensor_msgs::msg::Joy joy;
  joy.header.stamp = stamp;
  joy.header.frame_id = "wiimote";
  joy.axes = frame.axes;
  joy.buttons = frame.buttons;
  joy_publisher_->publish(joy);
}

void WiimoteDriverNode::publish_zero_joy()
{
  if (raw_mode_) {
    return;
  }
  const auto frame = racerbot_wiimote::build_joy({}, 0.0F, layout_);
  sensor_msgs::msg::Joy joy;
  joy.header.stamp = this->now();
  joy.header.frame_id = "wiimote";
  joy.axes = frame.axes;
  joy.buttons = frame.buttons;
  joy_publisher_->publish(joy);
}

void WiimoteDriverNode::publish_battery()
{
  try {
    racerbot_wiimote_msgs::msg::WiimoteBattery msg;
    msg.header.stamp = this->now();
    msg.battery_percentage = device_->battery();
    battery_publisher_->publish(msg);
    if (msg.battery_percentage < 20) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000,
        "Wiimote battery at %u%%: replace the AA batteries soon.", msg.battery_percentage);
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 60000, "%s", e.what());
  }
}

void WiimoteDriverNode::set_led_safely(unsigned int led, bool on)
{
  if (!device_) {
    return;
  }
  try {
    device_->set_led(led, on);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "%s", e.what());
  }
}

void WiimoteDriverNode::led_callback(const racerbot_wiimote_msgs::msg::WiimoteLED::SharedPtr msg)
{
  set_led_safely(0, msg->led_1);
  set_led_safely(1, msg->led_2);
  set_led_safely(2, msg->led_3);
  set_led_safely(3, msg->led_4);
}

void WiimoteDriverNode::rumble_callback(
  const racerbot_wiimote_msgs::msg::WiimoteRumble::SharedPtr msg)
{
  if (!device_) {
    return;
  }
  try {
    device_->set_rumble(msg->enabled);
  } catch (const std::exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 30000, "%s", e.what());
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<WiimoteDriverNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("wiimote_driver_node"), "Fatal error: %s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
