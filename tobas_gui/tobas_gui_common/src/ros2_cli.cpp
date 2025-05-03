#include <tobas_linux/subprocess.hpp>

#include "../include/tobas_gui_common/ros2_cli.hpp"

using namespace std;
namespace fs = filesystem;

namespace gui
{
namespace common
{
namespace util
{
string sourceCommand(const fs::path& install_dir)
{
  const auto setup_bash_path = install_dir / "setup.bash";
  return "source " + setup_bash_path.string();
}
}  // namespace util

pid_t rosrun(const fs::path& install_dir, const string& pkg, const string& exec, const string& name)
{
  auto command = util::sourceCommand(install_dir) + " && ros2 run " + pkg + " " + exec;
  if (!name.empty()) {
    command += " --ros-args --name " + name;
  }

  return linux::createSubprocess(command);
}

pid_t roslaunch(const fs::path& install_dir, const string& pkg, const string& name, const map<string, string>& args)
{
  auto command = util::sourceCommand(install_dir) + " && ros2 launch " + pkg + " " + name;
  for (const auto& [arg_name, arg_value] : args) {
    command += " " + arg_name + ":=" + arg_value;
  }

  return linux::createSubprocess(command);
}
}  // namespace common
}  // namespace gui
