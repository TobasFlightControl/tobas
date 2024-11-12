#include <iostream>

#include <tobas_linux/subprocess.hpp>

#include "../include/tobas_gui_common/ros2_cli.hpp"

using namespace std;

namespace gui
{
namespace common
{
pid_t rosrun(const string& pkg, const string& exec, const string& name)
{
  vector<char*> command;
  command.push_back(const_cast<char*>("ros2"));
  command.push_back(const_cast<char*>("run"));
  command.push_back(const_cast<char*>(pkg.c_str()));
  command.push_back(const_cast<char*>(exec.c_str()));

  if (!name.empty())
  {
    const auto name_arg = "--ros-args --name " + name;
    command.push_back(const_cast<char*>(name_arg.c_str()));
  }

  return linux::createSubprocess(command);
}

pid_t roslaunch(const string& pkg, const string& name, const map<string, string>& args)
{
  vector<char*> command;
  command.push_back(const_cast<char*>("ros2"));
  command.push_back(const_cast<char*>("launch"));
  command.push_back(const_cast<char*>(pkg.c_str()));
  command.push_back(const_cast<char*>(name.c_str()));

  vector<string> arg_buf;  // const_castは文字列をコピーしないため，明示的に文字列のメモリを確保しておく必要がある．
  for (const auto& arg : args)
  {
    arg_buf.push_back(arg.first + ":=" + arg.second);
    command.push_back(const_cast<char*>(arg_buf.back().c_str()));
  }

  return linux::createSubprocess(command);
}
}  // namespace common
}  // namespace gui
