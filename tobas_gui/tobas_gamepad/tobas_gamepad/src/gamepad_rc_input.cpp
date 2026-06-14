// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gamepad/gamepad_rc_input.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <iostream>
#include <utility>

#include <libevdev/libevdev.h>
#include <magic_enum/magic_enum.hpp>

#include <tobas_constants/flight_mode.hpp>
#include <tobas_math/core.hpp>

namespace tobas
{
namespace driver
{
void GamepadRcInput::LibevdevDeleter::operator()(libevdev* _dev) const
{
  if (_dev) {
    libevdev_free(_dev);
  }
}

GamepadRcInput::GamepadRcInput(GamepadRcInputConfig _config) : config_(std::move(_config))
{
}

GamepadRcInput::~GamepadRcInput()
{
  close();
}

bool GamepadRcInput::initialize(const std::string& _device_path)
{
  if (_device_path.empty()) {
    std::cerr << "Input device path is empty." << std::endl;
    return false;
  }

  device_path_ = _device_path;
  return openDevice();
}

bool GamepadRcInput::openDevice()
{
  close();

  fd_ = open(device_path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    std::cerr << "Failed to open input device." << std::endl;
    return false;
  }

  libevdev* dev = nullptr;
  const int rc = libevdev_new_from_fd(fd_, &dev);
  if (rc < 0) {
    std::cerr << "Failed to initialize libevdev." << std::endl;
    close();
    return false;
  }

  dev_.reset(dev);
  rc_input_ = {};
  rc_input_.ok = true;
  rc_input_.mode = FlightMode::kStabilize;
  return true;
}

void GamepadRcInput::close()
{
  dev_.reset();

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }

  rc_input_.ok = false;
}

bool GamepadRcInput::isOpen() const
{
  return fd_ >= 0 && dev_;
}

bool GamepadRcInput::poll()
{
  if (!isOpen()) {
    rc_input_.ok = false;
    return false;
  }

  input_event event;
  // ref: https://github.com/whot/libevdev/blob/master/tools/libevdev-events.c
  while (true) {
    const int rc = libevdev_next_event(dev_.get(), LIBEVDEV_READ_FLAG_NORMAL, &event);
    // 新しいイベントを正常に読めた．
    if (rc == LIBEVDEV_READ_STATUS_SUCCESS) {
      if (event.type == EV_KEY) {
        applyButton(event.code, event.value);
      }
      else if (event.type == EV_ABS) {
        applyAbs(event.code, event.value);
      }
      continue;
    }

    // イベントを取りこぼした可能性があるため，現在のデバイス状態に同期し直す．
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

    // 現時点で読めるイベントがない．
    if (rc == -EAGAIN) {
      rc_input_.ok = true;
      return true;
    }

    rc_input_.ok = false;
    std::cerr << "Failed to read input device." << std::endl;
    return false;
  }
}

bool GamepadRcInput::read(GamepadRcInputState& _rc_input)
{
  if (!poll()) {
    return false;
  }

  _rc_input = rc_input_;
  return true;
}

double GamepadRcInput::normalizeAbs(int _code, int _value, bool _invert) const
{
  if (!dev_) {
    return 0.0;
  }

  const input_absinfo* info = libevdev_get_abs_info(dev_.get(), _code);

  if (!info || info->minimum >= 0 || info->maximum <= 0) {
    return 0.0;
  }

  double normalized = math::remap<double>(
    static_cast<double>(_value), static_cast<double>(info->minimum), static_cast<double>(info->maximum), -1.0, 1.0);
  normalized = std::clamp(normalized, -1.0, 1.0);

  if (_invert) {
    return -normalized;
  }

  return normalized;
}

void GamepadRcInput::applyButton(int _code, int _value)
{
  const bool pressed = (_value != 0);

  // D-ROC RC入力のボタン割り当て．
  switch (_code) {
    // Aボタン: 押している間キルスイッチを有効にする．
    case BTN_SOUTH:
      rc_input_.kill = pressed;
      break;
    // Bボタン: サブモードを切り替える．
    case BTN_EAST:
      if (pressed) {
        rc_input_.sub_mode = !rc_input_.sub_mode;
      }
      break;
    // Yボタン: gpsw[1]を切り替える．
    case BTN_NORTH:
      if (pressed) {
        rc_input_.gpsw[1] = !rc_input_.gpsw[1];
      }
      break;
    
    // Xボタン: gpsw[0]を切り替える．
    case BTN_WEST:
      if (pressed) {
        rc_input_.gpsw[0] = !rc_input_.gpsw[0];
      }
      break;
    // L1ボタン: 飛行モードをAcrobat方向に切り替える．
    case BTN_TL:
      if (pressed) {
        constexpr auto kModes = magic_enum::enum_values<FlightMode>();
        const auto mode_index = magic_enum::enum_index(rc_input_.mode).value_or(0);
        if (mode_index > 0) {
          rc_input_.mode = (kModes[mode_index - 1]);
        }
      }
      break;
    // R1ボタン: 飛行モードをLoiter方向に切り替える．
    case BTN_TR:
      if (pressed) {
        constexpr auto kModes = magic_enum::enum_values<FlightMode>();
        const auto mode_index = magic_enum::enum_index(rc_input_.mode).value_or(0);
        if ((mode_index + 1) < kModes.size()) {
          rc_input_.mode = (kModes[mode_index + 1]);
        }
      }
      break;
    // Backボタン: RC入力を無効化
    case BTN_SELECT:
      if (pressed) {
        rc_input_.enable = false;
      }
      break;
    // STARTボタン: RC入力を有効化
    case BTN_START:
      if (pressed) {
        rc_input_.enable = true;
      }
      break;
    default:
      break;
  }
}

void GamepadRcInput::applyAbs(int _code, int _value)
{
  // D-ROC RC入力の軸割り当て．
  switch (_code) {
    // 左スティック左右: ヨー．
    case ABS_X:
      rc_input_.yaw = normalizeAbs(_code, _value, config_.invert_yaw);
      break;
    // 左スティック上下: ピッチ．
    case ABS_Y:
      rc_input_.pitch = normalizeAbs(_code, _value, config_.invert_pitch);
      break;
    // 右スティック左右: ロール．
    case ABS_RX:
      rc_input_.roll = normalizeAbs(_code, _value, config_.invert_roll);
      break;
    // 右スティック上下: スロットル．
    case ABS_RY:
      rc_input_.throttle = normalizeAbs(_code, _value, config_.invert_throttle);
      break;
    default:
      break;
  }
}
}  // namespace driver
}  // namespace tobas
