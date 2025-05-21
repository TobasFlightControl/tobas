#include "tobas_gui_common/argument.hpp"

#include <QProcessEnvironment>
#include <rclcpp/utilities.hpp>

#include <tobas_std_tools/vector.hpp>

#define QT_QPA_PLATFORM "QT_QPA_PLATFORM"

namespace gui
{
namespace common
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

  if (env.contains(QT_QPA_PLATFORM)) {
    std::cerr << "Cannot set display platform because \"" << QT_QPA_PLATFORM << "\" is set." << std::endl;
    return false;
  }

  if (tobas_std::contains<std::string>(args_, "-platform")) {
    std::cerr << "The display platform has already been specified via the arguments." << std::endl;
    return false;
  }

  args_.push_back("-platform");
  args_.push_back("xcb");

  return true;
}
}  // namespace common
}  // namespace gui
