// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <string>
#include <vector>

namespace tobas::gui::cmn
{
/**
 * Edit command-line arguments.
 *
 * @see [rviz2/src/main.cpp](https://github.com/ros2/rviz/blob/rolling/rviz2/src/main.cpp)
 */
class NonRosArgumentParser
{
public:
  explicit NonRosArgumentParser(int _argc, char** _argv);

  int& argc();
  char** argv();

  /** Configure the display server to use X11. */
  bool setPlatformXcb();

private:
  std::vector<std::string> args_;

  int argc_;                 ///< Memory allocation is required for `argc`.
  std::vector<char*> argv_;  ///< Memory allocation is required for `argv`.
};
}  // namespace tobas::gui::cmn
