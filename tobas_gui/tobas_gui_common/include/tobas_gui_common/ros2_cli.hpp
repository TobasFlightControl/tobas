#pragma once

#include <sys/types.h>

#include <filesystem>
#include <map>
#include <string>

namespace tobas
{
namespace gui
{
namespace cmn
{
/* 別プロセスで"ros2 run"を起動． */
pid_t rosrun(const std::string& pkg, const std::string& exec, const std::string& name = "");

/* 別プロセスで"ros2 launch"を起動． */
pid_t roslaunch(const std::string& pkg, const std::string& name, const std::map<std::string, std::string>& args = {});
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
