#include "tobas_gui_common/ros2_cli.hpp"

#include <tobas_linux/subprocess.hpp>

namespace fs = std::filesystem;

namespace gui
{
namespace common
{
namespace
{
std::string sourceCommand(const fs::path& install_dir)
{
  const auto setup_bash_path = install_dir / "local_setup.bash";
  return "source " + setup_bash_path.string();
}
}  // namespace

pid_t rosrun(const fs::path& install_dir, const std::string& pkg, const std::string& exec, const std::string& name)
{
  auto command = sourceCommand(install_dir) + " && ros2 run " + pkg + " " + exec;
  if (!name.empty()) {
    command += " --ros-args --name " + name;
  }

  return linux::createSubprocess(command);
}

pid_t roslaunch(
  const fs::path& install_dir,
  const std::string& pkg,
  const std::string& name,
  const std::map<std::string, std::string>& args)
{
  auto command = sourceCommand(install_dir) + " && ros2 launch " + pkg + " " + name;
  for (const auto& [arg_name, arg_value] : args) {
    command += " " + arg_name + ":=" + arg_value;
  }

  return linux::createSubprocess(command);
}
}  // namespace common
}  // namespace gui
