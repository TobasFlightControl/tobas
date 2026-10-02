// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <expected>
#include <memory>
#include <string>

#include <libevdev/libevdev.h>

#include <tobas_constants/flight_mode.hpp>

namespace tobas
{
namespace gamepad
{
/** RC input state generated from gamepad input. */
struct GamepadState
{
  bool ok = false;        ///< Whether the input device is being read correctly.
  double roll = 0.0;      ///< Roll command [-1, 1].
  double pitch = 0.0;     ///< Pitch command [-1, 1].
  double throttle = 0.0;  ///< Throttle command [-1, 1].
  double yaw = 0.0;       ///< Yaw command [-1, 1].
  FlightMode mode = FlightMode::kStabilize;
  bool sub_mode = false;  ///< Submode toggle.
  bool enable = false;    ///< RC input enable switch.
  bool kill = false;
  std::array<bool, 8> gpsw = {};  ///< General-purpose switch.
};

/** Settings for converting gamepad input to RC input. */
struct GamepadConfig
{
  bool invert_roll = false;
  bool invert_pitch = true;
  bool invert_throttle = true;
  bool invert_yaw = true;
};

/**
 * Driver that reads gamepad input with libevdev and converts it to RC input state.
 *
 * Reads a Linux input event device and converts button input to switches and absolute-axis input
 * to normalized RC command values.
 */
class GamepadDriver
{
public:
  struct LibevdevDeleter
  {
    void operator()(libevdev* _dev) const;
  };

  explicit GamepadDriver(GamepadConfig _config = {});

  GamepadDriver(const GamepadDriver& _other) = delete;
  GamepadDriver(GamepadDriver&& _other) = delete;
  GamepadDriver& operator=(const GamepadDriver& _other) = delete;
  GamepadDriver& operator=(GamepadDriver&& _other) = delete;

  ~GamepadDriver();

  /** Open the input device and make it readable. */
  std::expected<void, std::string> initialize(const std::string& _device_path);

  /** Close the input device. */
  void close();

  /** Return true if the input device is open. */
  bool isOpen() const;

  /** Read the current RC input state. */
  std::expected<GamepadState, std::string> read();

private:
  std::expected<void, std::string> poll();
  double normalizeAbs(int _code, int _value, bool _invert) const;
  void applyButton(int _code, int _value);
  void applyAbs(int _code, int _value);

  GamepadState state_;
  GamepadConfig config_;
  int fd_ = -1;
  std::unique_ptr<libevdev, LibevdevDeleter> dev_ = {};
};
}  // namespace gamepad
}  // namespace tobas
