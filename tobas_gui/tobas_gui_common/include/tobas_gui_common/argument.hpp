// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

namespace tobas
{
namespace gui
{
namespace cmn
{
/**
 * @brief Edit command-line arguments.
 *
 * cf. [rviz2/src/main.cpp](https://github.com/ros2/rviz/blob/rolling/rviz2/src/main.cpp)
 */
class NonRosArgumentParser
{
public:
  explicit NonRosArgumentParser(int argc, char** argv);

  int& argc();
  char** argv();

  /**
   * @brief Configure the display server to use X11.
   */
  bool setPlatformXcb();

private:
  std::vector<std::string> args_;

  int argc_;                 // Memory allocation is required for `argc`.
  std::vector<char*> argv_;  // Memory allocation is required for `argv`.
};
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
