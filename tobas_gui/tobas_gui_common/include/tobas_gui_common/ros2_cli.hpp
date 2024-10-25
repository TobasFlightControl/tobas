#pragma once

#include <string>
#include <map>
#include <sys/types.h>

namespace gui
{
namespace common
{
pid_t rosrun(const std::string& pkg, const std::string& exec, const std::string& name = "");
pid_t roslaunch(const std::string& pkg, const std::string& name, const std::map<std::string, std::string>& args = {});
}  // namespace common
}  // namespace gui
