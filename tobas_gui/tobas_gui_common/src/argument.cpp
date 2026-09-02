// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gui_common/argument.hpp"

#include <QProcessEnvironment>
#include <rclcpp/utilities.hpp>

#include <tobas_std_tools/vector.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
NonRosArgumentParser::NonRosArgumentParser(int argc, char** argv)
{
  args_ = rclcpp::remove_ros_arguments(argc, argv);
}

int& NonRosArgumentParser::argc()
{
  argc_ = static_cast<int>(args_.size());
  return argc_;
}

char** NonRosArgumentParser::argv()
{
  argv_.clear();
  for (auto& arg : args_) {
    argv_.push_back(&arg.front());
  }
  return argv_.data();
}

bool NonRosArgumentParser::setPlatformXcb()
{
  const auto env = QProcessEnvironment::systemEnvironment();
  const auto session_type = env.value("XDG_SESSION_TYPE");

  if (session_type == "x11") {
    return true;
  }

  constexpr char kQtQpaPlatform[] = "QT_QPA_PLATFORM";
  if (env.contains(kQtQpaPlatform)) {
    std::cerr << "Cannot set display platform because \"" << kQtQpaPlatform << "\" is set." << std::endl;
    return false;
  }

  if (st::contains<std::string>(args_, "-platform")) {
    std::cerr << "The display platform has already been specified via the arguments." << std::endl;
    return false;
  }

  args_.push_back("-platform");
  args_.push_back("xcb");

  return true;
}
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
