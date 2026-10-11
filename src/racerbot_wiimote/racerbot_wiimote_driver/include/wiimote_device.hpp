#ifndef WIIMOTE_DEVICE_HPP_
#define WIIMOTE_DEVICE_HPP_

#include <xwiimote.h>

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>

// Current state of the Wiimote's accelerometer.
struct WiimoteAccelerometerData
{
  int x = 0;
  int y = 0;
  int z = 0;
};

// RAII wrapper around one connected Wiimote (one xwiimote interface object
// with the core buttons and the accelerometer open on a single fd).
//
// Owned through std::unique_ptr by the node, so it is never copied or
// moved: the node drops it when the Wiimote disconnects and opens a new one
// when it comes back.
class WiimoteDevice
{
public:
  // Opens the first connected Wiimote, or returns nullptr if none is
  // connected. Throws std::runtime_error if one is found but cannot be
  // opened (usually permissions on /dev/input).
  static std::unique_ptr<WiimoteDevice> open_first();

  ~WiimoteDevice();
  WiimoteDevice(const WiimoteDevice &) = delete;
  WiimoteDevice & operator=(const WiimoteDevice &) = delete;

  // Reads every pending event. Returns false once the Wiimote is gone
  // (Bluetooth link lost, switched off, out of range): the caller must stop
  // using this object. Never blocks.
  bool update();

  // Whether the button is held, as of the last update(). key_code is one of
  // the XWII_KEY_* constants. After the device is gone every key reads up.
  bool is_button_down(unsigned int key_code) const;

  WiimoteAccelerometerData accelerometer() const {return accelerometer_;}

  // LEDs 0-3, left to right. Writes only on change. Throws on failure.
  void set_led(unsigned int led_number, bool on);

  // Battery percentage 0-100. Throws on failure.
  uint8_t battery() const;

  void set_rumble(bool enabled);

  const std::string & path() const {return path_;}

  // Seconds since the last event of any kind. In a hand the accelerometer
  // reports continuously, so a long silence means the radio link has
  // stalled even if Linux has not noticed the disconnect yet.
  double seconds_since_input() const
  {
    return std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_event_).count();
  }

private:
  WiimoteDevice(struct xwii_iface * iface, std::string path);
  void apply_event(const xwii_event & event);

  struct xwii_iface * iface_ = nullptr;
  int fd_ = -1;
  std::string path_;
  bool gone_ = false;
  std::unordered_map<unsigned int, bool> buttons_;
  std::optional<bool> leds_[4];
  WiimoteAccelerometerData accelerometer_;
  std::chrono::steady_clock::time_point last_event_ = std::chrono::steady_clock::now();
};

#endif  // WIIMOTE_DEVICE_HPP_
