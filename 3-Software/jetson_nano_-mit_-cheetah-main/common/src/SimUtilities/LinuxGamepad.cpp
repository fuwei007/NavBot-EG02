#ifdef linux

#include "SimUtilities/LinuxGamepad.h"

#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cstdio>

#include "Utilities/Log.h"

namespace {
float normalizeAxis(int16_t value) {
  constexpr float kDen = 32767.f;
  float out = static_cast<float>(value) / kDen;
  if (out > 1.f) out = 1.f;
  if (out < -1.f) out = -1.f;
  return out;
}
}  // namespace

LinuxGamepad::~LinuxGamepad() { closeDevice(); }

bool LinuxGamepad::openDevice(const std::string& device) {
  closeDevice();

  _fd = open(device.c_str(), O_RDONLY | O_NONBLOCK);
  if (_fd < 0) {
    LOG_ERROR("[LinuxGamepad] open: {}", std::strerror(errno));
    return false;
  }

  unsigned char axisCount = 0;
  unsigned char buttonCount = 0;
  if (ioctl(_fd, JSIOCGAXES, &axisCount) < 0) axisCount = 8;
  if (ioctl(_fd, JSIOCGBUTTONS, &buttonCount) < 0) buttonCount = 12;

  _axes.assign(axisCount, 0);
  _buttons.assign(buttonCount, 0);

  LOG_INFO("[LinuxGamepad] Opened {} with {} axes and {} buttons", device,
         axisCount, buttonCount);
  return true;
}

void LinuxGamepad::closeDevice() {
  if (_fd >= 0) {
    close(_fd);
    _fd = -1;
  }
  resetState();
}

bool LinuxGamepad::poll(GamepadCommand& command) {
  if (_fd < 0) return false;

  bool gotEvent = false;
  js_event event;
  while (true) {
    ssize_t bytes = read(_fd, &event, sizeof(event));
    if (bytes == sizeof(event)) {
      event.type &= ~JS_EVENT_INIT;
      if (event.type == JS_EVENT_AXIS && event.number < _axes.size()) {
        _axes[event.number] = event.value;
        gotEvent = true;
      } else if (event.type == JS_EVENT_BUTTON &&
                 event.number < _buttons.size()) {
        _buttons[event.number] = event.value;
        gotEvent = true;
      }
    } else {
      if (bytes < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        break;
      } else {
        // device disconnected?
        LOG_ERROR("[LinuxGamepad] read: {}", std::strerror(errno));
        closeDevice();
        return false;
      }
    }
  }

  if (!gotEvent) return false;

  convert(command);
  return true;
}

void LinuxGamepad::resetState() {
  _axes.clear();
  _buttons.clear();
}

void LinuxGamepad::convert(GamepadCommand& command) const {
  GamepadCommand out;
  if (_axes.size() > 0) out.leftStickAnalog[0] = normalizeAxis(_axes[0]);
  if (_axes.size() > 1) out.leftStickAnalog[1] = -normalizeAxis(_axes[1]);
  if (_axes.size() > 2) out.rightStickAnalog[0] = normalizeAxis(_axes[2]);
  if (_axes.size() > 3) out.rightStickAnalog[1] = -normalizeAxis(_axes[3]);

  // Triggers on F710 in D-mode appear as buttons.
  out.leftTriggerAnalog = 0.f;
  out.rightTriggerAnalog = 0.f;

  auto button = [&](size_t idx) -> bool {
    return idx < _buttons.size() ? (_buttons[idx] != 0) : false;
  };

  // Logitech F710 (D-input) button order differs from F310. Mapping based on jstest:
  out.x = button(0);
  out.a = button(1);
  out.b = button(2);
  out.y = button(3);
  out.leftBumper = button(4);
  out.rightBumper = button(5);
  out.leftTriggerButton = button(6);
  out.rightTriggerButton = button(7);
  out.back = button(8);
  out.start = button(9);
  out.leftStickButton = button(10);
  out.rightStickButton = button(11);
  out.logitechButton = button(12);

  command = out;
}

#endif  // linux
