#pragma once

#ifdef linux

#include <linux/joystick.h>

#include <string>
#include <vector>

#include "GamepadCommand.h"

/*!
 * Minimal Linux joystick reader for Logitech F310/F710 style gamepads.
 */
class LinuxGamepad {
 public:
  LinuxGamepad() = default;
  ~LinuxGamepad();

  LinuxGamepad(const LinuxGamepad&) = delete;
  LinuxGamepad& operator=(const LinuxGamepad&) = delete;

  //! Open joystick device, e.g. "/dev/input/js0"
  bool openDevice(const std::string& device);

  //! Close currently opened device
  void closeDevice();

  bool isOpen() const { return _fd >= 0; }

  //! Poll joystick and write results into command. Returns true if new data.
  bool poll(GamepadCommand& command);

 private:
  void resetState();
  void convert(GamepadCommand& command) const;

  int _fd{-1};
  std::vector<int16_t> _axes;
  std::vector<uint8_t> _buttons;
};

#endif  // linux
