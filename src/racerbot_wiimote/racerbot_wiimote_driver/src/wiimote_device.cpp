#include "wiimote_device.hpp"

#include <poll.h>

#include <cerrno>
#include <cstdlib>
#include <stdexcept>

namespace
{
struct MonitorDeleter
{
  void operator()(struct xwii_monitor * monitor) const noexcept
  {
    if (monitor) {
      xwii_monitor_unref(monitor);
    }
  }
};
using MonitorPtr = std::unique_ptr<struct xwii_monitor, MonitorDeleter>;

std::optional<std::string> find_device_path()
{
  // poll=false, direct=false: enumerate the Wiimotes connected right now.
  MonitorPtr monitor(xwii_monitor_new(false, false));
  if (!monitor) {
    return std::nullopt;
  }
  char * entry = xwii_monitor_poll(monitor.get());
  if (!entry) {
    return std::nullopt;
  }
  std::string path(entry);
  std::free(entry);
  return path;
}
}  // namespace

std::unique_ptr<WiimoteDevice> WiimoteDevice::open_first()
{
  const auto path = find_device_path();
  if (!path) {
    return nullptr;
  }
  struct xwii_iface * iface = nullptr;
  int err = xwii_iface_new(&iface, path->c_str());
  if (err) {
    throw std::runtime_error(
            "Cannot create a Wiimote interface for " + *path + " (error " +
            std::to_string(err) + ")");
  }
  // Core buttons, writable for LEDs and rumble, plus the accelerometer, all
  // on one fd. Hotplug watching makes a disconnect arrive as an event.
  err = xwii_iface_open(iface, XWII_IFACE_CORE | XWII_IFACE_ACCEL | XWII_IFACE_WRITABLE);
  if (err) {
    xwii_iface_unref(iface);
    throw std::runtime_error(
            "Cannot open the Wiimote at " + *path + " (error " + std::to_string(err) +
            "). Error -13 means no permission on /dev/input: see the Wiimote "
            "guide's udev step.");
  }
  xwii_iface_watch(iface, true);
  return std::unique_ptr<WiimoteDevice>(new WiimoteDevice(iface, *path));
}

WiimoteDevice::WiimoteDevice(struct xwii_iface * iface, std::string path)
: iface_(iface), fd_(xwii_iface_get_fd(iface)), path_(std::move(path))
{
}

WiimoteDevice::~WiimoteDevice()
{
  if (iface_) {
    xwii_iface_close(iface_, XWII_IFACE_ALL);
    xwii_iface_unref(iface_);
  }
}

bool WiimoteDevice::update()
{
  if (gone_) {
    return false;
  }
  struct pollfd pfd = {fd_, POLLIN, 0};
  const int ready = poll(&pfd, 1, 0);
  if (ready < 0) {
    if (errno == EINTR) {
      return true;
    }
    gone_ = true;
  } else if (ready > 0 && (pfd.revents & (POLLERR | POLLHUP | POLLNVAL))) {
    gone_ = true;
  } else if (ready > 0) {
    while (!gone_) {
      xwii_event event{};
      const int err = xwii_iface_dispatch(iface_, &event, sizeof(event));
      if (err == -EAGAIN) {
        break;
      }
      if (err != 0) {
        gone_ = true;  // the kernel closed the device
        break;
      }
      apply_event(event);
    }
  }
  if (gone_) {
    buttons_.clear();  // a vanished remote holds nothing down
  }
  return !gone_;
}

void WiimoteDevice::apply_event(const xwii_event & event)
{
  last_event_ = std::chrono::steady_clock::now();
  switch (event.type) {
    case XWII_EVENT_KEY:
      buttons_[event.v.key.code] = (event.v.key.state != 0);
      break;
    case XWII_EVENT_ACCEL:
      accelerometer_.x = event.v.abs[0].x;
      accelerometer_.y = event.v.abs[0].y;
      accelerometer_.z = event.v.abs[0].z;
      break;
    case XWII_EVENT_WATCH:
      // Sent when an interface closes, including on disconnect.
      if (!(xwii_iface_opened(iface_) & XWII_IFACE_CORE)) {
        gone_ = true;
      }
      break;
    default:
      break;
  }
}

bool WiimoteDevice::is_button_down(unsigned int key_code) const
{
  const auto it = buttons_.find(key_code);
  return !gone_ && it != buttons_.end() && it->second;
}

void WiimoteDevice::set_led(unsigned int led_number, bool on)
{
  if (led_number >= 4) {
    throw std::range_error("LED number must be 0-3");
  }
  if (leds_[led_number] == on) {
    return;  // sysfs write only on change, not 200 times a second
  }
  const int err = xwii_iface_set_led(iface_, XWII_LED(led_number + 1), on);
  if (err) {
    throw std::runtime_error(
            "Cannot set Wiimote LED " + std::to_string(led_number) + " (error " +
            std::to_string(err) + "). Error -13 means no permission: see the Wiimote "
            "guide's udev step.");
  }
  leds_[led_number] = on;
}

uint8_t WiimoteDevice::battery() const
{
  uint8_t percent = 0;
  const int err = xwii_iface_get_battery(iface_, &percent);
  if (err) {
    throw std::runtime_error("Cannot read the Wiimote battery (error " + std::to_string(err) +
            ")");
  }
  return percent;
}

void WiimoteDevice::set_rumble(bool enabled)
{
  const int err = xwii_iface_rumble(iface_, enabled);
  if (err) {
    throw std::runtime_error("Cannot set rumble (error " + std::to_string(err) + ")");
  }
}
