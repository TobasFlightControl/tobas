// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include <libevdev/libevdev.h>

#include <tobas_constants/flight_mode.hpp>

namespace tobas
{
namespace gamepad
{
/* RC input state generated from gamepad input.*/
struct GamepadRcInputState
{
  /* Whether the input device is being read correctly.*/
  bool ok = false;
  /* Roll command [-1, 1]. */
  double roll = 0.0;
  /* Pitch command [-1, 1]. */
  double pitch = 0.0;
  /* Throttle command [-1, 1]. */
  double throttle = 0.0;
  /* Yaw command [-1, 1]. */
  double yaw = 0.0;
  /* Flight mode. */
  FlightMode mode = FlightMode::kStabilize;
  /* Submode toggle. */
  bool sub_mode = false;
  /* RC input enable switch. */
  bool enable = false;
  /* Kill switch. */
  bool kill = false;
  /* General-purpose switch. */
  std::array<bool, 8> gpsw = {};
};

/* Settings for converting gamepad input to RC input.*/
struct GamepadRcInputConfig
{
  /* Invert the roll axis. */
  bool invert_roll = false;
  /* Invert the pitch axis. */
  bool invert_pitch = true;
  /* Invert the throttle axis. */
  bool invert_throttle = true;
  /* Invert the yaw axis. */
  bool invert_yaw = true;
};

/**
 * @brief Driver that reads gamepad input with libevdev and converts it to RC input state.
 *
 * Reads a Linux input event device and converts button input to switches and absolute-axis input to normalized RC command values.
 */
class GamepadRcInput
{
public:
  struct LibevdevDeleter
  {
    void operator()(libevdev* _dev) const;
  };

  explicit GamepadRcInput(GamepadRcInputConfig _config = {});

  GamepadRcInput(const GamepadRcInput& _other) = delete;
  GamepadRcInput(GamepadRcInput&& _other) = delete;
  GamepadRcInput& operator=(const GamepadRcInput& _other) = delete;
  GamepadRcInput& operator=(GamepadRcInput&& _other) = delete;

  ~GamepadRcInput();

  /* Open the input device and make it readable. */
  bool initialize(const std::string& _device_path);

  /* Close the input device.*/
  void close();

  /* Return whether the input device is open.*/
  bool isOpen() const;

  /* Read the current RC input state.*/
  bool read(GamepadRcInputState& _rc_input);

private:
  bool openDevice();
  bool poll();
  double normalizeAbs(int _code, int _value, bool _invert) const;
  void applyButton(int _code, int _value);
  void applyAbs(int _code, int _value);

  GamepadRcInputConfig config_;
  std::string device_path_;
  GamepadRcInputState rc_input_;
  int fd_ = -1;
  std::unique_ptr<libevdev, LibevdevDeleter> dev_ = {};
};
}  // namespace gamepad
}  // namespace tobas
