#pragma once

#include <string>
#include <filesystem>
#include <map>
#include <sys/types.h>

namespace gui
{
namespace common
{
/* 別プロセスで"ros2 run"を起動． */
pid_t rosrun(
  const std::filesystem::path& install_dir,
  const std::string& pkg,
  const std::string& exec,
  const std::string& name = "");

/* 別プロセスで"ros2 launch"を起動． */
pid_t roslaunch(
  const std::filesystem::path& install_dir,
  const std::string& pkg,
  const std::string& name,
  const std::map<std::string, std::string>& args = {});
}  // namespace common
}  // namespace gui
