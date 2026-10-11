#ifndef WIIMOTE_DRIVER_NODE_HPP_
#define WIIMOTE_DRIVER_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "racerbot_wiimote_driver/joy_mapping.hpp"
#include "racerbot_wiimote_msgs/msg/wiimote_accelerometer.hpp"
#include "racerbot_wiimote_msgs/msg/wiimote_battery.hpp"
#include "racerbot_wiimote_msgs/msg/wiimote_buttons.hpp"
#include "racerbot_wiimote_msgs/msg/wiimote_led.hpp"
#include "racerbot_wiimote_msgs/msg/wiimote_rumble.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "wiimote_device.hpp"

// Publishes a Wiimote as sensor_msgs/Joy plus raw button, battery and
// accelerometer topics.
//
// It never commands the car: joy_teleop (wiimote_teleop_launch.py) turns the
// Joy messages into /teleop. Whenever the Wiimote is missing, disconnected
// or silent for input_timeout_s, it publishes "nothing held", so the teleop
// layer commands a stop instead of repeating a stale command. It reconnects
// by itself when the Wiimote comes back.
class WiimoteDriverNode : public rclcpp::Node
{
public:
  WiimoteDriverNode();

private:
  void tick();
  void try_connect();
  void disconnect(const std::string & why);
  void publish_inputs();
  void publish_zero_joy();
  void publish_battery();
  void set_led_safely(unsigned int led, bool on);

  void led_callback(const racerbot_wiimote_msgs::msg::WiimoteLED::SharedPtr msg);
  void rumble_callback(const racerbot_wiimote_msgs::msg::WiimoteRumble::SharedPtr msg);

  // Parameters
  std::string joy_topic_;
  bool raw_mode_ = false;
  double reconnect_period_s_ = 1.0;
  double battery_period_s_ = 5.0;
  double input_timeout_s_ = 0.5;
  racerbot_wiimote::JoyLayout layout_;
  std::optional<racerbot_wiimote::TiltSteering> tilt_;

  std::unique_ptr<WiimoteDevice> device_;
  rclcpp::Time last_connect_attempt_;
  rclcpp::Time last_battery_;
  bool waiting_logged_ = false;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr joy_publisher_;
  rclcpp::Publisher<racerbot_wiimote_msgs::msg::WiimoteButtons>::SharedPtr buttons_publisher_;
  rclcpp::Publisher<racerbot_wiimote_msgs::msg::WiimoteBattery>::SharedPtr battery_publisher_;
  rclcpp::Publisher<racerbot_wiimote_msgs::msg::WiimoteAccelerometer>::SharedPtr
    accelerometer_publisher_;
  rclcpp::Subscription<racerbot_wiimote_msgs::msg::WiimoteLED>::SharedPtr led_subscriber_;
  rclcpp::Subscription<racerbot_wiimote_msgs::msg::WiimoteRumble>::SharedPtr rumble_subscriber_;
};

#endif  // WIIMOTE_DRIVER_NODE_HPP_
