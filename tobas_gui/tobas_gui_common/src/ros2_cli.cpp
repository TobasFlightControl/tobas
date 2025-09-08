#include "tobas_gui_common/ros2_cli.hpp"

#include <tobas_linux/subprocess.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace cmn
{
pid_t rosrun(const std::string& pkg, const std::string& exec, const std::string& name)
{
  auto command = "ros2 run " + pkg + " " + exec;
  if (!name.empty()) {
    command += " --ros-args --name " + name;
  }

  return linux::createSubprocess(command);
}

pid_t roslaunch(const std::string& pkg, const std::string& name, const std::map<std::string, std::string>& args)
{
  auto command = "ros2 launch " + pkg + " " + name;
  for (const auto& [arg_name, arg_value] : args) {
    command += " " + arg_name + ":=" + arg_value;
  }

  return linux::createSubprocess(command);
}
}  // namespace cmn
}  // namespace gui
