// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gamepad_core/gamepad_rc_input.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <utility>

#include <libevdev/libevdev.h>
#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/flight_mode.hpp>
#include <tobas_math/core.hpp>
#include <tobas_std_tools/error.hpp>

namespace tobas
{
namespace gamepad
{
void GamepadDriver::LibevdevDeleter::operator()(libevdev* _dev) const
{
  if (_dev) {
    libevdev_free(_dev);
  }
}

GamepadDriver::GamepadDriver(GamepadConfig _config) : config_(std::move(_config))
{
}

GamepadDriver::~GamepadDriver()
{
  close();
}

std::expected<void, std::string> GamepadDriver::initialize(const std::string& _device_path)
{
  fd_ = open(_device_path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    return std::unexpected("Failed to open input device: " + st::strError());
  }

  libevdev* dev = nullptr;
  const auto rc = libevdev_new_from_fd(fd_, &dev);
  if (rc < 0) {
    close();
    return std::unexpected("Failed to initialize libevdev: " + st::strError(-rc));
  }
  dev_.reset(dev);

  state_ = {};
  state_.ok = true;
  state_.mode = FlightMode::kStabilize;

  return {};
}

void GamepadDriver::close()
{
  dev_.reset();

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  state_.ok = false;
}

bool GamepadDriver::isOpen() const
{
  return fd_ >= 0 && dev_;
}

std::expected<void, std::string> GamepadDriver::poll()
{
  if (!isOpen()) {
    state_.ok = false;
    return std::unexpected("Input device is not open.");
  }

  input_event event;

  // Ref: https://github.com/whot/libevdev/blob/master/tools/libevdev-events.c
  while (true) {
    const auto rc = libevdev_next_event(dev_.get(), LIBEVDEV_READ_FLAG_NORMAL, &event);
    // A new event was read successfully.
    if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
      if (event.type == EV_KEY) {
        applyButton(event.code, event.value);
      }
      else if (event.type == EV_ABS) {
        applyAbs(event.code, event.value);
      }
      continue;
    }

    // Resynchronize with the current device state because events may have been missed.
    if (rc == LIBEVDEV_READ_STATUS_SYNC) {
      do {
        if (event.type == EV_KEY) {
          applyButton(event.code, event.value);
        }
        else if (event.type == EV_ABS) {
          applyAbs(event.code, event.value);
        }
      } while (libevdev_next_event(dev_.get(), LIBEVDEV_READ_FLAG_SYNC, &event) == LIBEVDEV_READ_STATUS_SYNC);

      continue;
    }

    // No events are currently available to read.
    if (rc == -EAGAIN) {
      state_.ok = true;
      return {};
    }

    state_.ok = false;
    return std::unexpected("Failed to read input device: " + st::strError(-rc));
  }
}

std::expected<GamepadState, std::string> GamepadDriver::read()
{
  if (const auto result = poll(); !result) {
    return std::unexpected(result.error());
  }

  return state_;
}

double GamepadDriver::normalizeAbs(int _code, int _value, bool _invert) const
{
  if (!dev_) {
    return 0.0;
  }

  const auto info = libevdev_get_abs_info(dev_.get(), _code);

  if (!info || info->minimum >= 0 || info->maximum <= 0) {
    return 0.0;
  }

  auto normalized = math::remap<double>(_value, info->minimum, info->maximum, -1.0, 1.0);
  normalized = std::clamp(normalized, -1.0, 1.0);

  if (_invert) {
    return -normalized;
  }

  return normalized;
}

void GamepadDriver::applyButton(int _code, int _value)
{
  const bool pressed = (_value != 0);

  switch (_code) {
    // A button: enable the kill switch while pressed.
    case BTN_SOUTH:
      state_.kill = pressed;
      break;
    // B button: toggle the submode.
    case BTN_EAST:
      if (pressed) {
        state_.sub_mode = !state_.sub_mode;
      }
      break;
    // Y button: toggle `gpsw[1]`.
    case BTN_NORTH:
      if (pressed) {
        state_.gpsw[1] = !state_.gpsw[1];
      }
      break;

    // X button: toggle `gpsw[0]`.
    case BTN_WEST:
      if (pressed) {
        state_.gpsw[0] = !state_.gpsw[0];
      }
      break;
    // L1 button: switch the flight mode toward Acrobat.
    case BTN_TL:
      if (pressed) {
        constexpr auto kModes = magic_enum::enum_values<FlightMode>();
        const auto mode_index = magic_enum::enum_index(state_.mode).value_or(0);
        if (mode_index > 0) {
          state_.mode = (kModes[mode_index - 1]);
        }
      }
      break;
    // R1 button: switch the flight mode toward Loiter.
    case BTN_TR:
      if (pressed) {
        constexpr auto kModes = magic_enum::enum_values<FlightMode>();
        const auto mode_index = magic_enum::enum_index(state_.mode).value_or(0);
        if ((mode_index + 1) < kModes.size()) {
          state_.mode = (kModes[mode_index + 1]);
        }
      }
      break;
    // Back button: disable RC input.
    case BTN_SELECT:
      if (pressed) {
        state_.enable = false;
      }
      break;
    // START button: enable RC input.
    case BTN_START:
      if (pressed) {
        state_.enable = true;
      }
      break;
    default:
      break;
  }
}

void GamepadDriver::applyAbs(int _code, int _value)
{
  switch (_code) {
    // Left stick horizontal: yaw.
    case ABS_X:
      state_.yaw = normalizeAbs(_code, _value, config_.invert_yaw);
      break;
    // Left stick vertical: pitch.
    case ABS_Y:
      state_.pitch = normalizeAbs(_code, _value, config_.invert_pitch);
      break;
    // Right stick horizontal: roll.
    case ABS_RX:
      state_.roll = normalizeAbs(_code, _value, config_.invert_roll);
      break;
    // Right stick vertical: throttle.
    case ABS_RY:
      state_.throttle = normalizeAbs(_code, _value, config_.invert_throttle);
      break;
    default:
      break;
  }
}
}  // namespace gamepad
}  // namespace tobas
