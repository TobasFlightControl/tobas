// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "d_roc_gamepad/gamepad_rc_input.hpp"

#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <iostream>
#include <utility>

#include <libevdev/libevdev.h>

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
  rc_input_.mode = static_cast<std::uint8_t>(FlightMode::kLoiter);
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
  return fd_ >= 0 && dev_ != nullptr;
}

double GamepadRcInput::normalizeSymmetric(int _value, int _minimum, int _maximum)
{
  if (_value >= 0) {
    return static_cast<double>(_value) / static_cast<double>(_maximum);
  }

  return static_cast<double>(_value) / -static_cast<double>(_minimum);
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

  double normalized = normalizeSymmetric(_value, info->minimum, info->maximum);
  normalized = std::clamp(normalized, -1.0, 1.0);

  if (std::abs(normalized) < config_.deadzone) {
    normalized = 0.0;
  }

  if (_invert) {
    return -normalized;
  }

  return normalized;
}

void GamepadRcInput::applyButton(int _code, int _value)
{
  const bool pressed = _value != 0;

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
    // Yボタン: 意味なし
    case BTN_NORTH:
      if (pressed) {
        rc_input_.gpsw[0] = !rc_input_.gpsw[0];
      }
      break;
    // L1ボタン: RC入力の有効/無効を切り替える．
    case BTN_TL:
      if (pressed) {
        rc_input_.enable = !rc_input_.enable;
      }
      break;
    // R1ボタン: 飛行モードをAcrobat，Stabilize，Loiterの順に切り替える．
    case BTN_TR:
      if (pressed) {
        constexpr auto kModeCount = static_cast<std::uint8_t>(FlightMode::kCount);
        rc_input_.mode = static_cast<std::uint8_t>((rc_input_.mode + 1) % kModeCount);
      }
      break;
    // SELECTボタン: 意味なし
    case BTN_SELECT:
      if (pressed) {
        rc_input_.gpsw[1] = !rc_input_.gpsw[1];
      }
      break;
    // STARTボタン: 意味なし
    case BTN_START:
      if (pressed) {
        rc_input_.gpsw[2] = !rc_input_.gpsw[2];
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
    // 十字キー左右: 意味なし
    case ABS_HAT0X:
      rc_input_.gpsw[4] = _value < 0;
      rc_input_.gpsw[5] = _value > 0;
      break;
    // 十字キー上下: 意味なし
    case ABS_HAT0Y:
      rc_input_.gpsw[6] = _value < 0;
      rc_input_.gpsw[7] = _value > 0;
      break;
    default:
      break;
  }
}
}  // namespace driver
}  // namespace tobas
